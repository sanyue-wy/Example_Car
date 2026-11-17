/**
 * @file    elevating.c
 * @brief   丝杆升降平台模块(实现层)
 *          编码器电机 -> 丝杆 -> 平台直线运动: 位置闭环 + 软限位
 * @version 1.0
 * @date    2026-11-17
 */

/* Includes ------------------------------------------------------------------*/
#include "elevating.h"
#include <string.h>

/*==============================================================================
 * ★★★  方向 / 机械参数(装配、接线、丝杆不同时, 只改这一段!)  ★★★
 *
 *  调试顺序:
 *    1. 先跑 encoder_motor_debug, 确认"正占空比 -> 编码器 rps 为正";
 *       若不对, 改下面的 ELEVATING_ENC_INVERT / ELEVATING_DIR_INVERT;
 *    2. 再上本模块, 给一个很小的 MoveTo(如 +2mm), 目视平台运动方向;
 *       若"位置增大时平台实际下降", 只把 ELEVATING_UP_SIGN 改成 -1.0f;
 *    3. 最后按实际丝杆改导程, 按机构改行程和 PID。
 *
 *  注意: 位置反馈来自电机编码器, 软件无法自行判断"上/下"的物理方向,
 *        ELEVATING_UP_SIGN 必须人工确认一次。
 *============================================================================*/

/* ---- 方向: 最容易装反的三处, 集中放在最上面 ------------------------------ */

/* 平台"向上"对应的电机转向(+1.0f = 电机正转(正占空比)时上升;
 *                        -1.0f = 电机正转时下降)。
 * 装反只改这一行, 不要去动 PID 符号或编码器取反! */
#define ELEVATING_UP_SIGN              (+1.0f)

/* 编码器 A/B 相接线对调: 0 = 正常, 1 = 取反
 * (仅在"电机正转时编码器计数变小"时置 1) */
#define ELEVATING_ENC_INVERT           0U

/* H 桥 IN1/IN2 或电机两根线对调: 0 = 正常, 1 = 对调
 * (仅在"正占空比时电机实际反转"时置 1; 与 ENC_INVERT 配合保证
 *  正占空比 <-> 正转速, 否则速度环会正反馈飞车) */
#define ELEVATING_DIR_INVERT           0U

/* ---- 机械参数 ------------------------------------------------------------ */

/* 丝杆导程: 电机输出轴(减速箱输出)转一圈, 平台移动的距离(mm)。
 * 常见 T8 丝杆: 2mm(自锁好/速度慢) 或 8mm(速度快); 按实际丝杆改! */
#define ELEVATING_SCREW_LEAD_MM        (2.0f)

/* 行程软限位(mm): 以 SetZero 的位置为 0, 向上为正。
 * 目标超出该范围直接拒绝, 运动中被越过则报错停车。 */
#define ELEVATING_MIN_MM               (0.0f)
#define ELEVATING_MAX_MM               (200.0f)
#define ELEVATING_LIMIT_MARGIN_MM      (0.5f)   /* 越界判定的容差 */

/* ---- 电机 / 编码器硬件(与 encoder_motor_debug 保持一致) ------------------ */

#define ELEVATING_ENC_TIM              (&htim2)       /* TIM2 Encoder_Interface */
#define ELEVATING_PWM_TIM              (&htim1)       /* TIM1_CH1 PWM */
#define ELEVATING_PWM_CH               TIM_CHANNEL_1
#define ELEVATING_PWM_FREQ_HZ          (20000U)
#define ELEVATING_DIR_PORT             GPIOB
#define ELEVATING_DIR_PIN_FWD          GPIO_PIN_0     /* H 桥 IN1 */
#define ELEVATING_DIR_PIN_REV          GPIO_PIN_1     /* H 桥 IN2 */

/* JCA25-370-103K: 11 线 x 4 倍频 x 103 减速比 = 4532 计数/输出轴圈 */
#define ELEVATING_COUNTS_PER_REV       (4532U)

/* ---- 控制参数 ------------------------------------------------------------ */

#define ELEVATING_PERIOD_S             (0.01f)        /* Task 调用周期 10ms */
#define ELEVATING_MOTOR_RPS_LIMIT      (1.35f)        /* 电机输出轴最大转速(rps) */

/* 速度环(内环, EncoderMotor 内部): 输出 ‰ 占空比 */
#define ELEVATING_SPEED_KP             (600.0f)
#define ELEVATING_SPEED_KI             (800.0f)
#define ELEVATING_SPEED_KD             (0.0f)

/* 位置环(外环, 本模块): 输入 mm 误差, 输出目标转速(rps) */
#define ELEVATING_POS_KP               (1.2f)
#define ELEVATING_POS_KI               (0.3f)
#define ELEVATING_POS_KD               (0.0f)

/* 到位判定: 位置误差和线速度都足够小才认为停稳(避免高速冲过目标) */
#define ELEVATING_ARRIVE_MM            (0.30f)
#define ELEVATING_ARRIVE_SPEED_MM_S    (0.40f)

/* 移动超时 = 理论最短时间 x 1.5 + 2s 余量(按目标距离动态计算) */
#define ELEVATING_TIMEOUT_SCALE        (1.5f)
#define ELEVATING_TIMEOUT_MARGIN_TICKS (200U)         /* 2s / 10ms */

/* 单个脉冲上限, 防止距离异常时除法溢出 */
#define ELEVATING_MAX_TIMEOUT_TICKS    (100000U)

/* Private function declarations ---------------------------------------------*/

static void Elevating_UpdateFeedback(Elevating_Object *lift);
static uint32_t Elevating_CalcTimeout(const Elevating_Object *lift, float target_mm);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  用编码器计数刷新位置/线速度(带零点偏置和方向符号)
  */
static void Elevating_UpdateFeedback(Elevating_Object *lift)
{
  int32_t count = EncoderMotor_GetCount(&lift->motor);
  float   revs  = (float)(count - lift->zero_count) / (float)ELEVATING_COUNTS_PER_REV;

  lift->position_mm = revs * ELEVATING_SCREW_LEAD_MM * ELEVATING_UP_SIGN;
  lift->speed_mm_s  = EncoderMotor_GetSpeedRps(&lift->motor)
                      * ELEVATING_SCREW_LEAD_MM * ELEVATING_UP_SIGN;
}

/**
  * @brief  按移动距离估算超时阈值(Task 次数)
  * @note   最慢情况按电机最大转速走完全程, 再留 1.5 倍与 2s 余量;
  *         目标已在范围内, 距离过大/参数异常时由 MAX_TIMEOUT 兜底
  */
static uint32_t Elevating_CalcTimeout(const Elevating_Object *lift, float target_mm)
{
  float max_speed_mm_s = lift->motor.cfg.rps_limit * ELEVATING_SCREW_LEAD_MM;
  float dist_mm        = FLOAT_ABS(target_mm - lift->position_mm);
  float seconds;
  float ticks;

  if (max_speed_mm_s < 0.01f)
  {
    max_speed_mm_s = 0.01f;
  }

  seconds = dist_mm / max_speed_mm_s;
  ticks   = seconds / ELEVATING_PERIOD_S * ELEVATING_TIMEOUT_SCALE
            + (float)ELEVATING_TIMEOUT_MARGIN_TICKS;

  if (ticks > (float)ELEVATING_MAX_TIMEOUT_TICKS)
  {
    return ELEVATING_MAX_TIMEOUT_TICKS;
  }
  if (ticks < 300.0f) /* 至少 3s, 覆盖短距离的加减速 */
  {
    return 300U;
  }
  return (uint32_t)ticks;
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  初始化升降平台(内部同时初始化编码器电机)
  * @param  lift 升降平台对象, 必须静态分配
  * @retval 0 成功; 1 参数错误; 2 编码器电机初始化失败(见 last_motor_ret)
  * @note   调用前 CubeMX 必须已配置 TIM2 编码器模式、TIM1_CH1 PWM、PB0/PB1 输出;
  *         初始化后当前位置即为 0(软件零点), 需要回零时再调 Elevating_SetZero
  */
uint8_t Elevating_Init(Elevating_Object *lift)
{
  EncoderMotor_Config_t cfg;

  if (lift == NULL)
  {
    return 1;
  }

  memset(lift, 0, sizeof(*lift));

  cfg.name             = "ELEV_MOTOR";
  cfg.enc_htim         = ELEVATING_ENC_TIM;
  cfg.pwm_htim         = ELEVATING_PWM_TIM;
  cfg.pwm_channel      = ELEVATING_PWM_CH;
  cfg.dir_port         = ELEVATING_DIR_PORT;
  cfg.dir_pin_fwd      = ELEVATING_DIR_PIN_FWD;
  cfg.dir_pin_rev      = ELEVATING_DIR_PIN_REV;
  cfg.pwm_freq_hz      = ELEVATING_PWM_FREQ_HZ;
  cfg.counts_per_rev   = ELEVATING_COUNTS_PER_REV;
  cfg.enc_invert       = ELEVATING_ENC_INVERT;
  cfg.dir_invert       = ELEVATING_DIR_INVERT;
  cfg.rps_limit        = ELEVATING_MOTOR_RPS_LIMIT;
  cfg.speed_kp         = ELEVATING_SPEED_KP;
  cfg.speed_ki         = ELEVATING_SPEED_KI;
  cfg.speed_kd         = ELEVATING_SPEED_KD;
  cfg.max_duty         = 1000.0f;
  cfg.control_period_s = ELEVATING_PERIOD_S;

  lift->last_motor_ret = EncoderMotor_Init(&lift->motor, &cfg);
  if (lift->last_motor_ret != 0U)
  {
    lift->initialized = 0U;
    return 2;
  }

  PID_Init(&lift->pos_pid, ELEVATING_POS_KP, ELEVATING_POS_KI, ELEVATING_POS_KD,
           ELEVATING_MOTOR_RPS_LIMIT, ELEVATING_MOTOR_RPS_LIMIT, ELEVATING_PERIOD_S);
  lift->pos_pid.Improve = PID_IMPROVE_INTEGRAL_LIMIT;

  lift->zero_count   = EncoderMotor_GetCount(&lift->motor);
  lift->target_mm    = 0.0f;
  lift->position_mm  = 0.0f;
  lift->speed_mm_s   = 0.0f;
  lift->move_ticks   = 0U;
  lift->timeout_ticks = ELEVATING_MAX_TIMEOUT_TICKS;
  lift->fault_ref    = lift->motor.stats.fault_count;
  lift->state        = ELEVATING_STATE_IDLE;
  lift->initialized  = 1U;

  EncoderMotor_Stop(&lift->motor);
  return 0;
}

/**
  * @brief  周期任务, 每 10ms 调用一次
  * @note   内部依次执行: 电机速度环 -> 采样位置 -> 位置环 -> 下发目标转速
  */
void Elevating_Task(Elevating_Object *lift)
{
  float err_mm;
  float speed_rps;

  if (lift == NULL || lift->initialized == 0U)
  {
    return;
  }

  EncoderMotor_Task(&lift->motor);
  Elevating_UpdateFeedback(lift);

  if (lift->state != ELEVATING_STATE_MOVING)
  {
    return;
  }

  /* 电机层飞车保护触发(CPR/方向配错) -> 本模块同步进入故障态 */
  if (lift->motor.stats.fault_count != lift->fault_ref)
  {
    EncoderMotor_Stop(&lift->motor);
    PID_Clear(&lift->pos_pid);
    lift->state = ELEVATING_STATE_ERROR;
    return;
  }

  /* 移动超时(卡死/堵转/断线) */
  lift->move_ticks++;
  if (lift->move_ticks > lift->timeout_ticks)
  {
    EncoderMotor_Stop(&lift->motor);
    PID_Clear(&lift->pos_pid);
    lift->state = ELEVATING_STATE_ERROR;
    return;
  }

  /* 越过软限位(编码器漂移或方向配错导致的失控) */
  if (lift->position_mm < (ELEVATING_MIN_MM - ELEVATING_LIMIT_MARGIN_MM) ||
      lift->position_mm > (ELEVATING_MAX_MM + ELEVATING_LIMIT_MARGIN_MM))
  {
    EncoderMotor_Stop(&lift->motor);
    PID_Clear(&lift->pos_pid);
    lift->state = ELEVATING_STATE_ERROR;
    return;
  }

  err_mm = lift->target_mm - lift->position_mm;

  /* 到位: 误差小且速度低, 避免高速冲过目标后反复修正 */
  if (FLOAT_ABS(err_mm) <= ELEVATING_ARRIVE_MM &&
      FLOAT_ABS(lift->speed_mm_s) <= ELEVATING_ARRIVE_SPEED_MM_S)
  {
    EncoderMotor_Stop(&lift->motor);
    PID_Clear(&lift->pos_pid);
    lift->state = ELEVATING_STATE_ARRIVED;
    return;
  }

  /* 位置环输出"向上为正"的目标转速, 再按 UP_SIGN 换成电机转向 */
  speed_rps = PID_Calculate(&lift->pos_pid, lift->position_mm, lift->target_mm);
  LIMIT_MIN_MAX(speed_rps, -ELEVATING_MOTOR_RPS_LIMIT, ELEVATING_MOTOR_RPS_LIMIT);
  EncoderMotor_SetSpeedRps(&lift->motor, speed_rps * ELEVATING_UP_SIGN);
}

/**
  * @brief  绝对位置移动(mm)
  * @param  lift      升降平台对象
  * @param  target_mm 目标位置, 以零点为 0、向上为正
  * @retval 0 已接受; 1 未初始化/参数错误; 2 目标超出软限位(不会动作)
  * @note   非阻塞: 接受后由 Elevating_Task 闭环执行, 用 IsArrived/GetState 查询
  */
uint8_t Elevating_MoveTo(Elevating_Object *lift, float target_mm)
{
  if (lift == NULL || lift->initialized == 0U)
  {
    return 1;
  }

  /* 写成 !(>= && <=) 可同时挡住 NaN */
  if (!(target_mm >= ELEVATING_MIN_MM && target_mm <= ELEVATING_MAX_MM))
  {
    return 2;
  }

  lift->target_mm     = target_mm;
  lift->move_ticks    = 0U;
  lift->timeout_ticks = Elevating_CalcTimeout(lift, target_mm);
  lift->fault_ref     = lift->motor.stats.fault_count;
  PID_Clear(&lift->pos_pid);

  /* 切到速度模式并把目标置 0, 由 Task 每拍更新 */
  EncoderMotor_SetSpeedRps(&lift->motor, 0.0f);

  lift->state = ELEVATING_STATE_MOVING;
  return 0;
}

/**
  * @brief  相对当前位置移动(mm), 正数向上
  * @retval 同 Elevating_MoveTo
  */
uint8_t Elevating_MoveRelative(Elevating_Object *lift, float delta_mm)
{
  if (lift == NULL || lift->initialized == 0U)
  {
    return 1;
  }

  return Elevating_MoveTo(lift, lift->position_mm + delta_mm);
}

/**
  * @brief  立即停车并取消当前目标(位置保持由丝杆自锁/摩擦完成)
  */
void Elevating_Stop(Elevating_Object *lift)
{
  if (lift == NULL || lift->initialized == 0U)
  {
    return;
  }

  EncoderMotor_Stop(&lift->motor);
  PID_Clear(&lift->pos_pid);
  lift->target_mm = lift->position_mm;
  lift->state     = ELEVATING_STATE_IDLE;
}

/**
  * @brief  把当前位置设为软件零点并停车
  * @note   建议在平台位于机械下限位时调用; 不依赖限位开关
  */
void Elevating_SetZero(Elevating_Object *lift)
{
  if (lift == NULL || lift->initialized == 0U)
  {
    return;
  }

  EncoderMotor_Stop(&lift->motor);
  PID_Clear(&lift->pos_pid);

  lift->zero_count  = EncoderMotor_GetCount(&lift->motor);
  lift->position_mm = 0.0f;
  lift->speed_mm_s  = 0.0f;
  lift->target_mm   = 0.0f;
  lift->move_ticks  = 0U;
  lift->state       = ELEVATING_STATE_IDLE;
}

float Elevating_GetPositionMm(const Elevating_Object *lift)
{
  return (lift != NULL) ? lift->position_mm : 0.0f;
}

float Elevating_GetTargetMm(const Elevating_Object *lift)
{
  return (lift != NULL) ? lift->target_mm : 0.0f;
}

float Elevating_GetSpeedMmS(const Elevating_Object *lift)
{
  return (lift != NULL) ? lift->speed_mm_s : 0.0f;
}

float Elevating_GetMinMm(const Elevating_Object *lift)
{
  (void)lift;
  return ELEVATING_MIN_MM;
}

float Elevating_GetMaxMm(const Elevating_Object *lift)
{
  (void)lift;
  return ELEVATING_MAX_MM;
}

Elevating_State_e Elevating_GetState(const Elevating_Object *lift)
{
  return (lift != NULL) ? lift->state : ELEVATING_STATE_IDLE;
}

uint8_t Elevating_IsArrived(const Elevating_Object *lift)
{
  return (lift != NULL && lift->state == ELEVATING_STATE_ARRIVED) ? 1U : 0U;
}