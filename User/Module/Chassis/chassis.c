/**
 * @file    chassis.c
 * @brief   四轮差速底盘运动控制模块(实现层)
 * @version 1.0
 * @date    2026-11-17
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis.h"

#include <math.h>
#include <string.h>

#if CHASSIS_ENABLE_AT8236
#include "at8236.h"
#endif

/* Private macros ------------------------------------------------------------*/

#define CHASSIS_PI            3.14159265358979323846f
#define CHASSIS_DEG2RAD       (CHASSIS_PI / 180.0f)
#define CHASSIS_RAD2DEG       (180.0f / CHASSIS_PI)
#define CHASSIS_STOP_EPS      0.5f   /* 斜坡回零判定(mm/s 或 °/s) */
#define CHASSIS_TASK_DT_MAX   100U   /* 单拍最大积分时间, 防止调度卡顿导致跳变 */
#define CHASSIS_AT8236_ONLINE_MS 1000U

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 分层与取舍:
  * 1. 本文件前半部分是"纯底盘": 只做 (v,ω) -> 左右轮 -> 四轮, 斜坡/限幅/
  *    里程计/目标租约, 不包含 at8236.h、UART、HAL_GetTick —— 换电机总线时
  *    一行不用改;
  * 2. 电机总线用 Chassis_MotorOps_t 函数指针注入, 本模块不持有具体设备对象,
  *    只在 motor_ctx 里保存一个 void*。AT8236 适配器放在文件末尾, 由
  *    CHASSIS_ENABLE_AT8236 控制是否编译, 是唯一包含 at8236.h 的地方;
  * 3. 时间全部由 Chassis_Task(now_ms) 注入, 不在核心逻辑里调用 HAL; 目标
  *    刷新时间也由 Task 登记(target_dirty), 使核心可在主机上做单元测试;
  * 4. 限幅时保持 v/ω 比例(整体缩放), 不单独砍左右轮, 避免转弯半径畸变;
  * 5. 里程计用编码器增量(而不是速度积分), 按左右轮平均做圆弧积分;
  *    第一帧反馈只建立基准, 避免上电时把绝对计数当成位移跳变;
  * 6. 驱动板是保持型速度指令, Task 按 command_period_ms 周期重发; 可选
  *    command_timeout_ms 租约在业务层忘记刷新目标时自动减速停车。
  ******************************************************************************
  */

/* Private function declarations ---------------------------------------------*/

static float   Chassis_Abs(float value);
static float   Chassis_WheelSign(const Chassis_Object *chassis, uint8_t index);
static float   Chassis_MoveToward(float current, float target, float step);
static void    Chassis_ApplyLimits(Chassis_Object *chassis, float *v_mmps, float *w_dps);
static void    Chassis_ComputeWheels(const Chassis_Object *chassis, float v_mmps,
                                     float w_dps, float wheel_mmps[CHASSIS_MOTOR_NUM]);
static uint8_t Chassis_SendWheels(Chassis_Object *chassis, uint32_t now_ms);
static uint8_t Chassis_SendHold(Chassis_Object *chassis, uint32_t now_ms);
static void    Chassis_UpdateFeedback(Chassis_Object *chassis, uint32_t now_ms);
static void    Chassis_WrapYaw(float *yaw_rad);
static uint8_t Chassis_WheelMapValid(const Chassis_Config_t *cfg);
static void    Chassis_MapToChannel(const Chassis_Config_t *cfg,
                                    const float logical[CHASSIS_MOTOR_NUM],
                                    float physical[CHASSIS_MOTOR_NUM]);

/* Private functions ----------------------------------------------------------*/

static float Chassis_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

/**
  * @brief  由 invert_mask 得到该轮的"物理正向系数"
  * @note   底盘约定正值=前进; 若某轮机械镜像安装, 驱动板需要反向指令,
  *         反馈计数也要同乘 -1, 保证输入/输出在同一坐标系
  */
static float Chassis_WheelSign(const Chassis_Object *chassis, uint8_t index)
{
  return (((chassis->cfg.invert_mask >> index) & 0x01U) != 0U) ? -1.0f : 1.0f;
}

/**
  * @brief  wheel_map 必须是 0~3 的一个排列, 否则两侧轮会互相覆盖
  */
static uint8_t Chassis_WheelMapValid(const Chassis_Config_t *cfg)
{
  uint8_t seen = 0U;

  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    uint8_t ch = cfg->wheel_map[i];

    if (ch >= CHASSIS_MOTOR_NUM)
    {
      return 0U;
    }
    if ((seen & (uint8_t)(1U << ch)) != 0U)
    {
      return 0U;
    }
    seen |= (uint8_t)(1U << ch);
  }
  return 1U;
}

/**
  * @brief  逻辑轮序 -> 物理通道: physical[wheel_map[i]] = logical[i]
  */
static void Chassis_MapToChannel(const Chassis_Config_t *cfg,
                                 const float logical[CHASSIS_MOTOR_NUM],
                                 float physical[CHASSIS_MOTOR_NUM])
{
  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    physical[cfg->wheel_map[i]] = logical[i];
  }
}

/**
  * @brief  按最大步长逼近目标(线性斜坡)
  */
static float Chassis_MoveToward(float current, float target, float step)
{
  float delta = target - current;

  if (delta > step)
  {
    return current + step;
  }
  if (delta < -step)
  {
    return current - step;
  }
  return target;
}

/**
  * @brief  (v,ω) 限幅: 先独立限 v/ω, 再按单轮上限整体缩放(保转弯半径)
  */
static void Chassis_ApplyLimits(Chassis_Object *chassis, float *v_mmps, float *w_dps)
{
  const Chassis_Config_t *cfg = &chassis->cfg;
  uint8_t limited = 0U;
  float w_rad;
  float left;
  float right;
  float max_wheel;

  if (*v_mmps > cfg->max_linear_mmps)
  {
    *v_mmps = cfg->max_linear_mmps;
    limited = 1U;
  }
  else if (*v_mmps < -cfg->max_linear_mmps)
  {
    *v_mmps = -cfg->max_linear_mmps;
    limited = 1U;
  }

  if (*w_dps > cfg->max_angular_dps)
  {
    *w_dps = cfg->max_angular_dps;
    limited = 1U;
  }
  else if (*w_dps < -cfg->max_angular_dps)
  {
    *w_dps = -cfg->max_angular_dps;
    limited = 1U;
  }

  w_rad = (*w_dps) * CHASSIS_DEG2RAD;
  left  = (*v_mmps) - (w_rad * cfg->track_width_mm * 0.5f);
  right = (*v_mmps) + (w_rad * cfg->track_width_mm * 0.5f);
  max_wheel = (Chassis_Abs(left) > Chassis_Abs(right)) ? Chassis_Abs(left) : Chassis_Abs(right);

  if (max_wheel > cfg->max_wheel_mmps)
  {
    float scale = cfg->max_wheel_mmps / max_wheel;

    *v_mmps *= scale;
    *w_dps  *= scale;
    limited = 1U;
  }

  if (limited != 0U)
  {
    chassis->stats.limit_count++;
  }
}

/**
  * @brief  差速运动学: (v,ω) -> 左右轮线速度 -> 四轮(含逐轮换向)
  * @note   ω 逆时针为正: 左轮 = v - ω*B/2, 右轮 = v + ω*B/2
  */
static void Chassis_ComputeWheels(const Chassis_Object *chassis, float v_mmps,
                                  float w_dps, float wheel_mmps[CHASSIS_MOTOR_NUM])
{
  float w_rad = w_dps * CHASSIS_DEG2RAD;
  float left  = v_mmps - (w_rad * chassis->cfg.track_width_mm * 0.5f);
  float right = v_mmps + (w_rad * chassis->cfg.track_width_mm * 0.5f);

  wheel_mmps[0] = left  * Chassis_WheelSign(chassis, 0U);
  wheel_mmps[1] = left  * Chassis_WheelSign(chassis, 1U);
  wheel_mmps[2] = right * Chassis_WheelSign(chassis, 2U);
  wheel_mmps[3] = right * Chassis_WheelSign(chassis, 3U);

  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    if (wheel_mmps[i] > chassis->cfg.max_wheel_mmps)
    {
      wheel_mmps[i] = chassis->cfg.max_wheel_mmps;
    }
    else if (wheel_mmps[i] < -chassis->cfg.max_wheel_mmps)
    {
      wheel_mmps[i] = -chassis->cfg.max_wheel_mmps;
    }
  }
}

/**
  * @brief  下发四轮速度(失败不更新 last_tx, 下一拍自动重试)
  */
static uint8_t Chassis_SendWheels(Chassis_Object *chassis, uint32_t now_ms)
{
  float wheel_mmps[CHASSIS_MOTOR_NUM];
  float channel_mmps[CHASSIS_MOTOR_NUM];

  if ((chassis->ops == NULL) || (chassis->ops->set_wheel_speeds == NULL))
  {
    return 1U;
  }

  Chassis_ComputeWheels(chassis, chassis->cmd_v_mmps, chassis->cmd_w_dps, wheel_mmps);

  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    chassis->wheel_cmd[i] = wheel_mmps[i];
  }

  /* 逻辑轮序 -> 驱动板通道(实车接线与板子丝印不一致时由 wheel_map 吸收) */
  Chassis_MapToChannel(&chassis->cfg, wheel_mmps, channel_mmps);

  if (chassis->ops->set_wheel_speeds(chassis->motor_ctx, channel_mmps) != 0U)
  {
    chassis->stats.cmd_busy++;
    return 1U;
  }

  chassis->last_tx_ms = now_ms;
  chassis->stats.cmd_sent++;
  return 0U;
}

/**
  * @brief  主动保持(驱动板 $spd:0, PID 锁轮)
  */
static uint8_t Chassis_SendHold(Chassis_Object *chassis, uint32_t now_ms)
{
  if ((chassis->ops == NULL) || (chassis->ops->hold == NULL))
  {
    return 1U;
  }

  if (chassis->ops->hold(chassis->motor_ctx) != 0U)
  {
    chassis->stats.cmd_busy++;
    return 1U;
  }

  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    chassis->wheel_cmd[i] = 0.0f;
  }

  chassis->last_tx_ms = now_ms;
  return 0U;
}

/**
  * @brief  反馈换算 + 编码器里程计
  */
static void Chassis_UpdateFeedback(Chassis_Object *chassis, uint32_t now_ms)
{
  const Chassis_Config_t *cfg = &chassis->cfg;
  Chassis_Feedback_t fb;
  float s0;
  float s1;
  float s2;
  float s3;
  float left;
  float right;

  if ((chassis->ops == NULL) || (chassis->ops->get_feedback == NULL))
  {
    chassis->feedback_valid = 0U;
    chassis->stats.feedback_lost++;
    return;
  }

  if (chassis->ops->get_feedback(chassis->motor_ctx, &fb) == 0U)
  {
    chassis->feedback_valid = 0U;
    chassis->stats.feedback_lost++;
    return;
  }

  /* 反馈按物理通道上报, 先转回逻辑轮序, 后续里程计/遥测所有下标统一 */
  {
    Chassis_Feedback_t raw = fb;

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
    {
      uint8_t ch = chassis->cfg.wheel_map[i];

      fb.wheel_speed_mmps[i] = raw.wheel_speed_mmps[ch];
      fb.wheel_count[i]      = raw.wheel_count[ch];
    }
  }

  chassis->feedback = fb;
  chassis->feedback_valid = 1U;

  s0 = Chassis_WheelSign(chassis, 0U);
  s1 = Chassis_WheelSign(chassis, 1U);
  s2 = Chassis_WheelSign(chassis, 2U);
  s3 = Chassis_WheelSign(chassis, 3U);

  left  = ((fb.wheel_speed_mmps[0] * s0) + (fb.wheel_speed_mmps[1] * s1)) * 0.5f;
  right = ((fb.wheel_speed_mmps[2] * s2) + (fb.wheel_speed_mmps[3] * s3)) * 0.5f;

  chassis->measured_left_mmps  = left;
  chassis->measured_right_mmps = right;
  chassis->measured_v_mmps = (left + right) * 0.5f;
  chassis->measured_w_dps  = (right - left) / cfg->track_width_mm * CHASSIS_RAD2DEG;

  chassis->odom.linear_mmps  = chassis->measured_v_mmps;
  chassis->odom.angular_dps  = chassis->measured_w_dps;
  chassis->odom.update_ms    = now_ms;
  chassis->odom.valid        = 1U;

  if (chassis->feedback_seen == 0U)
  {
    /* 第一帧只建立计数基准, 不当作位移 */
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
    {
      chassis->last_count[i] = fb.wheel_count[i];
    }
    chassis->feedback_seen = 1U;
  }
  else
  {
    float mm_per_count = (CHASSIS_PI * cfg->wheel_diameter_mm) / (float)cfg->counts_per_rev;
    float d_left;
    float d_right;
    float ds;
    float dtheta;
    float mid_yaw;

    d_left = (((float)(fb.wheel_count[0] - chassis->last_count[0]) * s0) +
              ((float)(fb.wheel_count[1] - chassis->last_count[1]) * s1)) * 0.5f * mm_per_count;
    d_right = (((float)(fb.wheel_count[2] - chassis->last_count[2]) * s2) +
               ((float)(fb.wheel_count[3] - chassis->last_count[3]) * s3)) * 0.5f * mm_per_count;

    ds     = (d_left + d_right) * 0.5f;
    dtheta = (d_right - d_left) / cfg->track_width_mm; /* rad */
    mid_yaw = chassis->odom.yaw_rad + (dtheta * 0.5f);

    chassis->odom.x_mm += ds * cosf(mid_yaw);
    chassis->odom.y_mm += ds * sinf(mid_yaw);
    chassis->odom.yaw_rad += dtheta;
    Chassis_WrapYaw(&chassis->odom.yaw_rad);

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
    {
      chassis->last_count[i] = fb.wheel_count[i];
    }
  }

  chassis->odom.yaw_deg = chassis->odom.yaw_rad * CHASSIS_RAD2DEG;
}

/**
  * @brief  航向角归一化到 [-π, π]
  */
static void Chassis_WrapYaw(float *yaw_rad)
{
  while (*yaw_rad > CHASSIS_PI)
  {
    *yaw_rad -= (2.0f * CHASSIS_PI);
  }
  while (*yaw_rad < -CHASSIS_PI)
  {
    *yaw_rad += (2.0f * CHASSIS_PI);
  }
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  本工程默认配置(JCA25-370-103K, 67mm 轮, 150mm 轮距)
  */
void Chassis_DefaultConfig(Chassis_Config_t *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  memset(cfg, 0, sizeof(*cfg));
  cfg->name                 = "CHASSIS";
  cfg->track_width_mm       = 150.0f;
  cfg->wheel_diameter_mm    = 67.0f;
  cfg->counts_per_rev       = 4532U; /* 11 线 × 4 倍频 × 103 减速比 */
  cfg->max_linear_mmps      = 300.0f;
  cfg->max_angular_dps      = 180.0f;
  cfg->max_wheel_mmps       = 1000.0f; /* 驱动板 $spd 上限 */
  cfg->linear_accel_mmps2   = 600.0f;
  cfg->angular_accel_dps2   = 360.0f;
  cfg->linear_deadband_mmps = 5.0f;
  cfg->angular_deadband_dps = 2.0f;
  cfg->invert_mask          = 0x00U;
  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    cfg->wheel_map[i] = i;   /* 默认: 逻辑轮序 = 驱动板 M1~M4 */
  }
  cfg->command_period_ms    = 50U;
  cfg->command_timeout_ms   = 0U;   /* 默认关闭租约, 需要时由上层打开 */
  cfg->feedback_timeout_ms  = 500U;
}

/**
  * @brief  初始化底盘并绑定电机总线
  * @param  chassis   底盘对象(必须静态分配)
  * @param  cfg       配置
  * @param  ops       电机总线接口(至少提供 set_wheel_speeds)
  * @param  motor_ctx 接口上下文, 原样传给 ops
  * @retval CHASSIS_OK / CHASSIS_ERR_PARAM
  * @note   Init 会立即下发一次 hold, 防止上电继承驱动板旧指令
  */
uint8_t Chassis_Init(Chassis_Object *chassis, const Chassis_Config_t *cfg,
                     const Chassis_MotorOps_t *ops, void *motor_ctx)
{
  if ((chassis == NULL) || (cfg == NULL) || (ops == NULL) ||
      (ops->set_wheel_speeds == NULL))
  {
    return (uint8_t)CHASSIS_ERR_PARAM;
  }

  if (!(cfg->track_width_mm > 0.0f) || !(cfg->wheel_diameter_mm > 0.0f) ||
      (cfg->counts_per_rev == 0U) ||
      !(cfg->max_linear_mmps > 0.0f) || !(cfg->max_angular_dps > 0.0f) ||
      !(cfg->max_wheel_mmps > 0.0f) ||
      !(cfg->linear_accel_mmps2 > 0.0f) || !(cfg->angular_accel_dps2 > 0.0f) ||
      (cfg->linear_deadband_mmps < 0.0f) || (cfg->angular_deadband_dps < 0.0f) ||
      (cfg->command_period_ms == 0U) || (Chassis_WheelMapValid(cfg) == 0U))
  {
    return (uint8_t)CHASSIS_ERR_PARAM;
  }

  memset(chassis, 0, sizeof(*chassis));
  chassis->cfg = *cfg;
  chassis->ops = ops;
  chassis->motor_ctx = motor_ctx;
  chassis->mode = CHASSIS_MODE_HOLD;
  chassis->initialized = 1U;

  (void)Chassis_SendHold(chassis, 0U);
  return (uint8_t)CHASSIS_OK;
}

/**
  * @brief  设置底盘目标速度(v, ω)
  * @param  v_mmps    线速度, 正=前进
  * @param  omega_dps 角速度, 正=左转(逆时针)
  * @retval CHASSIS_OK / CHASSIS_ERR_STATE / CHASSIS_ERR_PARAM
  * @note   只更新目标, 斜坡/下发在 Chassis_Task; 重复调用即刷新目标租约
  */
uint8_t Chassis_SetVelocity(Chassis_Object *chassis, float v_mmps, float omega_dps)
{
  if ((chassis == NULL) || (chassis->initialized == 0U))
  {
    return (uint8_t)CHASSIS_ERR_STATE;
  }

  if (!(v_mmps == v_mmps) || !(omega_dps == omega_dps))
  {
    return (uint8_t)CHASSIS_ERR_PARAM; /* 拒绝 NaN */
  }

  if (Chassis_Abs(v_mmps) < chassis->cfg.linear_deadband_mmps)
  {
    v_mmps = 0.0f;
  }
  if (Chassis_Abs(omega_dps) < chassis->cfg.angular_deadband_dps)
  {
    omega_dps = 0.0f;
  }

  Chassis_ApplyLimits(chassis, &v_mmps, &omega_dps);

  chassis->target_v_mmps = v_mmps;
  chassis->target_w_dps  = omega_dps;
  chassis->mode = ((v_mmps == 0.0f) && (omega_dps == 0.0f)) ?
                  CHASSIS_MODE_STOPPING : CHASSIS_MODE_VELOCITY;
  chassis->target_dirty = 1U;
  return (uint8_t)CHASSIS_OK;
}

/**
  * @brief  直接给左右轮线速度, 内部反解成 (v,ω)
  */
uint8_t Chassis_SetWheelSpeeds(Chassis_Object *chassis, float left_mmps, float right_mmps)
{
  float v;
  float w;

  if ((chassis == NULL) || (chassis->initialized == 0U))
  {
    return (uint8_t)CHASSIS_ERR_STATE;
  }

  v = (left_mmps + right_mmps) * 0.5f;
  w = (right_mmps - left_mmps) / chassis->cfg.track_width_mm * CHASSIS_RAD2DEG;
  return Chassis_SetVelocity(chassis, v, w);
}

/**
  * @brief  按加速度斜坡减速到 0, 到零后转主动保持
  */
uint8_t Chassis_Stop(Chassis_Object *chassis)
{
  if ((chassis == NULL) || (chassis->initialized == 0U))
  {
    return (uint8_t)CHASSIS_ERR_STATE;
  }

  chassis->target_v_mmps = 0.0f;
  chassis->target_w_dps  = 0.0f;
  chassis->mode = CHASSIS_MODE_STOPPING;
  chassis->target_dirty = 1U;
  return (uint8_t)CHASSIS_OK;
}

/**
  * @brief  立即主动保持(硬停, 不经过斜坡)
  * @note   P0-1: 只置请求标志与模式, **不在本函数里发串口**。
  *         所有运行期发送集中在 Chassis_Task(中断上下文), 避免主循环与
  *         Task 争抢 USART2 导致 $spd:0 被 BUSY 丢弃。
  */
uint8_t Chassis_Hold(Chassis_Object *chassis)
{
  if ((chassis == NULL) || (chassis->initialized == 0U))
  {
    return (uint8_t)CHASSIS_ERR_STATE;
  }

  chassis->target_v_mmps = 0.0f;
  chassis->target_w_dps  = 0.0f;
  chassis->cmd_v_mmps    = 0.0f;
  chassis->cmd_w_dps     = 0.0f;
  chassis->target_dirty  = 0U;
  chassis->mode = CHASSIS_MODE_HOLD;
  chassis->stats.hold_count++;

  /* 指令先置 pending 再置 mode, 保证 Task 看到 HOLD 时一定会补发 */
  chassis->hold_pending  = 1U;
  chassis->coast_pending = 0U;
  return (uint8_t)CHASSIS_OK;
}

/**
  * @brief  立即断开输出(滑行)
  * @note   P0-1: 同 Chassis_Hold, 只置标志, 由 Chassis_Task 发送。
  */
uint8_t Chassis_Coast(Chassis_Object *chassis)
{
  if ((chassis == NULL) || (chassis->initialized == 0U))
  {
    return (uint8_t)CHASSIS_ERR_STATE;
  }

  chassis->target_v_mmps = 0.0f;
  chassis->target_w_dps  = 0.0f;
  chassis->cmd_v_mmps    = 0.0f;
  chassis->cmd_w_dps     = 0.0f;
  chassis->target_dirty  = 0U;
  chassis->mode = CHASSIS_MODE_COAST;

  if ((chassis->ops == NULL) || (chassis->ops->coast == NULL))
  {
    return (uint8_t)CHASSIS_ERR_BUSY;
  }

  chassis->coast_pending = 1U;
  chassis->hold_pending  = 0U;
  return (uint8_t)CHASSIS_OK;
}

/**
  * @brief  急停: 清目标并立即主动保持
  */
uint8_t Chassis_EmergencyStop(Chassis_Object *chassis)
{
  return Chassis_Hold(chassis);
}

/**
  * @brief  周期任务(建议 10ms): 租约、斜坡、限幅下发、反馈、里程计
  */
void Chassis_Task(Chassis_Object *chassis, uint32_t now_ms)
{
  uint32_t dt_ms;
  float dt_s;

  if ((chassis == NULL) || (chassis->initialized == 0U))
  {
    return;
  }

  dt_ms = now_ms - chassis->last_task_ms;
  if (chassis->last_task_ms == 0U)
  {
    dt_ms = 0U; /* 第一拍只登记时间 */
  }
  if (dt_ms > CHASSIS_TASK_DT_MAX)
  {
    dt_ms = CHASSIS_TASK_DT_MAX;
  }
  chassis->last_task_ms = now_ms;
  chassis->stats.task_count++;

  /* 目标刷新租约: 只有 SetVelocity 类调用才会置 dirty */
  if (chassis->target_dirty != 0U)
  {
    chassis->last_target_ms = now_ms;
    chassis->target_dirty = 0U;
  }

  if ((chassis->mode == CHASSIS_MODE_VELOCITY) &&
      (chassis->cfg.command_timeout_ms > 0U) &&
      ((now_ms - chassis->last_target_ms) > chassis->cfg.command_timeout_ms))
  {
    chassis->stats.timeout_count++;
    (void)Chassis_Stop(chassis);
    chassis->last_target_ms = now_ms; /* 避免同一次超时反复触发 */
  }

  dt_s = (float)dt_ms * 0.001f;

  /* P0-1: 停车/急停请求一律在这里落地。
     驱动板是"保持型速度指令", 丢一帧 $spd:0 车就按旧速度一直跑,
     所以 pending 标志只在**发送成功**时清除, 失败下一拍自动重试。 */
  if (chassis->hold_pending != 0U)
  {
    if (Chassis_SendHold(chassis, now_ms) == 0U)
    {
      chassis->hold_pending = 0U;
    }
    Chassis_UpdateFeedback(chassis, now_ms);
    return;      /* 本拍不再发速度帧, 保证停车帧优先 */
  }
  if (chassis->coast_pending != 0U)
  {
    if ((chassis->ops == NULL) || (chassis->ops->coast == NULL) ||
        (chassis->ops->coast(chassis->motor_ctx) == 0U))
    {
      chassis->coast_pending = 0U;
      for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
      {
        chassis->wheel_cmd[i] = 0.0f;
      }
      chassis->last_tx_ms = now_ms;
    }
    else
    {
      chassis->stats.cmd_busy++;
    }
    Chassis_UpdateFeedback(chassis, now_ms);
    return;
  }

  /* P2-1: 未使能时强制 HOLD, 拒绝一切速度指令 */
  if (chassis->enabled == 0U)
  {
    if (chassis->mode != CHASSIS_MODE_HOLD)
    {
      chassis->mode = CHASSIS_MODE_HOLD;
      chassis->cmd_v_mmps = 0.0f;
      chassis->cmd_w_dps  = 0.0f;
      chassis->target_v_mmps = 0.0f;
      chassis->target_w_dps  = 0.0f;
    }
    /* 周期重发: 驱动板丢帧/重启后能自己回到静止 */
    if ((now_ms - chassis->last_tx_ms) >= chassis->cfg.command_period_ms)
    {
      (void)Chassis_SendHold(chassis, now_ms);
    }
    Chassis_UpdateFeedback(chassis, now_ms);
    return;
  }

  switch (chassis->mode)
  {
    case CHASSIS_MODE_VELOCITY:
      chassis->cmd_v_mmps = Chassis_MoveToward(chassis->cmd_v_mmps,
                                               chassis->target_v_mmps,
                                               chassis->cfg.linear_accel_mmps2 * dt_s);
      chassis->cmd_w_dps = Chassis_MoveToward(chassis->cmd_w_dps,
                                              chassis->target_w_dps,
                                              chassis->cfg.angular_accel_dps2 * dt_s);
      if ((now_ms - chassis->last_tx_ms) >= chassis->cfg.command_period_ms)
      {
        (void)Chassis_SendWheels(chassis, now_ms);
      }
      break;

    case CHASSIS_MODE_STOPPING:
      chassis->cmd_v_mmps = Chassis_MoveToward(chassis->cmd_v_mmps, 0.0f,
                                               chassis->cfg.linear_accel_mmps2 * dt_s);
      chassis->cmd_w_dps = Chassis_MoveToward(chassis->cmd_w_dps, 0.0f,
                                              chassis->cfg.angular_accel_dps2 * dt_s);
      if ((Chassis_Abs(chassis->cmd_v_mmps) <= CHASSIS_STOP_EPS) &&
          (Chassis_Abs(chassis->cmd_w_dps) <= CHASSIS_STOP_EPS))
      {
        chassis->cmd_v_mmps = 0.0f;
        chassis->cmd_w_dps  = 0.0f;
        /* 降完速后转 HOLD: 交给上面的 hold_pending 通道统一发送 */
        chassis->mode = CHASSIS_MODE_HOLD;
        chassis->hold_pending = 1U;
        chassis->stats.hold_count++;
      }
      else if ((now_ms - chassis->last_tx_ms) >= chassis->cfg.command_period_ms)
      {
        (void)Chassis_SendWheels(chassis, now_ms);
      }
      break;

    case CHASSIS_MODE_HOLD:
      /* P0-1: HOLD 期间也按周期重发 $spd:0, 防驱动板丢一帧就失控 */
      if ((now_ms - chassis->last_tx_ms) >= chassis->cfg.command_period_ms)
      {
        (void)Chassis_SendHold(chassis, now_ms);
      }
      break;

    case CHASSIS_MODE_COAST:
    case CHASSIS_MODE_IDLE:
    default:
      break;
  }

  Chassis_UpdateFeedback(chassis, now_ms);
}

/* Getters --------------------------------------------------------------------*/

Chassis_Mode_e Chassis_GetMode(const Chassis_Object *chassis)
{
  return (chassis == NULL) ? CHASSIS_MODE_IDLE : chassis->mode;
}

/* P2-1: 使能门控 ------------------------------------------------------------*/

void Chassis_SetEnabled(Chassis_Object *chassis, uint8_t enabled)
{
  if ((chassis == NULL) || (chassis->initialized == 0U))
  {
    return;
  }

  chassis->enabled = (enabled != 0U) ? 1U : 0U;

  if (chassis->enabled == 0U)
  {
    /* 置 HOLD + 停车请求; 不在这里发串口(见 P0-1) */
    (void)Chassis_Hold(chassis);
  }
}

uint8_t Chassis_IsEnabled(const Chassis_Object *chassis)
{
  return (chassis == NULL) ? 0U : chassis->enabled;
}

float Chassis_GetTargetLinearMmps(const Chassis_Object *chassis)
{
  return (chassis == NULL) ? 0.0f : chassis->target_v_mmps;
}

float Chassis_GetTargetAngularDps(const Chassis_Object *chassis)
{
  return (chassis == NULL) ? 0.0f : chassis->target_w_dps;
}

float Chassis_GetLinearMmps(const Chassis_Object *chassis)
{
  return (chassis == NULL) ? 0.0f : chassis->measured_v_mmps;
}

float Chassis_GetAngularDps(const Chassis_Object *chassis)
{
  return (chassis == NULL) ? 0.0f : chassis->measured_w_dps;
}

float Chassis_GetWheelCommand(const Chassis_Object *chassis, uint8_t index)
{
  if ((chassis == NULL) || (index >= CHASSIS_MOTOR_NUM))
  {
    return 0.0f;
  }
  return chassis->wheel_cmd[index];
}

void Chassis_GetFeedback(const Chassis_Object *chassis, Chassis_Feedback_t *feedback)
{
  if ((chassis == NULL) || (feedback == NULL))
  {
    return;
  }
  *feedback = chassis->feedback;
}

void Chassis_GetOdometry(const Chassis_Object *chassis, Chassis_Odometry_t *odom)
{
  if ((chassis == NULL) || (odom == NULL))
  {
    return;
  }
  *odom = chassis->odom;
}

void Chassis_ResetOdometry(Chassis_Object *chassis)
{
  if (chassis == NULL)
  {
    return;
  }

  memset(&chassis->odom, 0, sizeof(chassis->odom));
  chassis->odom.update_ms = chassis->last_task_ms;
  chassis->odom.valid = chassis->feedback_valid;

  if (chassis->feedback_valid != 0U)
  {
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
    {
      chassis->last_count[i] = chassis->feedback.wheel_count[i];
    }
    chassis->feedback_seen = 1U;
  }
  else
  {
    chassis->feedback_seen = 0U;
  }
}

uint8_t Chassis_IsOnline(const Chassis_Object *chassis, uint32_t now_ms)
{
  if ((chassis == NULL) || (chassis->feedback_valid == 0U))
  {
    return 0U;
  }
  return ((now_ms - chassis->odom.update_ms) <= chassis->cfg.feedback_timeout_ms) ? 1U : 0U;
}

/* --------------------------------------------------------------------------- */
/*  AT8236 串口驱动板适配器(唯一包含 at8236.h 的地方, 可整体裁剪)               */
/* --------------------------------------------------------------------------- */
#if CHASSIS_ENABLE_AT8236

static int16_t ChassisAt8236_Round(float value)
{
  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static uint8_t ChassisAt8236_SetWheels(void *ctx, const float wheel_mmps[CHASSIS_MOTOR_NUM])
{
  AT8236_Object *dev = (AT8236_Object *)ctx;
  int16_t cmd[CHASSIS_MOTOR_NUM];

  if (dev == NULL)
  {
    return 1U;
  }

  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    cmd[i] = ChassisAt8236_Round(wheel_mmps[i]);
  }

  return (uint8_t)((AT8236_SetSpeedArray(dev, cmd) == 0U) ? 0U : 1U);
}

static uint8_t ChassisAt8236_Hold(void *ctx)
{
  AT8236_Object *dev = (AT8236_Object *)ctx;

  if (dev == NULL)
  {
    return 1U;
  }
  return (uint8_t)((AT8236_Stop(dev) == 0U) ? 0U : 1U);
}

static uint8_t ChassisAt8236_Coast(void *ctx)
{
  AT8236_Object *dev = (AT8236_Object *)ctx;

  if (dev == NULL)
  {
    return 1U;
  }
  return (uint8_t)((AT8236_Coast(dev) == 0U) ? 0U : 1U);
}

static uint8_t ChassisAt8236_GetFeedback(void *ctx, Chassis_Feedback_t *feedback)
{
  AT8236_Object *dev = (AT8236_Object *)ctx;

  if ((dev == NULL) || (feedback == NULL))
  {
    return 0U;
  }

  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    feedback->wheel_speed_mmps[i] = AT8236_GetSpeedMmps(dev, i);
    feedback->wheel_count[i]      = AT8236_GetEncoderTotal(dev, i);
  }

  /* 驱动板只有打开 $upload 才会周期上报, 1s 内没有帧视为离线 */
  return AT8236_IsOnline(dev, HAL_GetTick(), CHASSIS_AT8236_ONLINE_MS);
}

static const Chassis_MotorOps_t s_at8236_ops =
{
  .set_wheel_speeds = ChassisAt8236_SetWheels,
  .hold             = ChassisAt8236_Hold,
  .coast            = ChassisAt8236_Coast,
  .get_feedback     = ChassisAt8236_GetFeedback
};

const Chassis_MotorOps_t *Chassis_GetAT8236Ops(void)
{
  return &s_at8236_ops;
}

#endif /* CHASSIS_ENABLE_AT8236 */
