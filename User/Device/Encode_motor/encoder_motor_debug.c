#include "debug_config.h"

#if encoder_motor_debug == 1

#include "encoder_motor_debug.h"
#include "encoder_motor.h"
#include "drv_uart.h"
#include "main.h"
#include "tim.h"

/* ---- 第17层 encoder_motor_debug: 编码器电机自检(电机会转! 先架空车轮) ----
   编码器电机有执行器也有反馈, 可以闭环自动判定, 流程:
     开环 +500‰ 正转 1.2s  -> 转速为正且计数增加
     开环 -500‰ 反转 1.2s  -> 转速为负且计数减少
     停车滑行 1.5s         -> 转速回落到 0 附近
     闭环 +0.5rps 1.8s     -> 稳态误差在容差内
     闭环 -0.5rps 1.8s     -> 稳态误差在容差内
     闭环 0rps 1.5s        -> 停住
     目标限幅              -> SetSpeedRps(999) 被钳到 rps_limit
   每项 [PASS]/[FAIL], 结束后停车并汇总; 只跑一轮, 不复位重跑。
   注意: 方向取反 / PID 增益必须按实车微调; 电机参数按 JCA25-370-103K 资料填。 */

/* ---- 本工程电机 JCA25-370-103K(12V 4W 减速电机 + 霍尔编码器) ------------
   资料要点:
   - 编码器: 电机轴每转 11 个信号/相(黄/绿 = A/B), TIM2 TI12 四倍频
     -> 44 计数/电机轴转;
   - 减速比 103 -> 输出轴一圈 11 × 4 × 103 = 4532 计数;
   - 12V 空载 85rpm ≈ 1.42rps, 额定 55rpm ≈ 0.92rps,
     额定转矩 8kg·cm, 额定电流 ≤750mA;
   - 资料明确"禁止堵转或瞬间负载反转": 本测试正反转之间先滑行停车, 不要改成
     直接由 +500‰ 切 -500‰;
   - 接线: 红/白 = 电机电源(接 H 桥, 对调可换向); 蓝 = 编码器 VCC,
     黑 = 编码器 GND; 黄/绿 = A/B 相信号 -> PA0/PA1。编码器标称 3.3~5V,
     但 PA0/PA1 不是 5V 容忍引脚, 必须把编码器 VCC 接 3.3V。 ------------- */

/* ---- 硬件绑定(换板/换引脚只改这里) ------------------------------------- */
#define EMT_ENC_TIM        (&htim2)      /* 编码器: TIM2 Encoder_Interface */
#define EMT_PWM_TIM        (&htim1)      /* 调速: TIM1_CH1(PA8) */
#define EMT_PWM_CH         TIM_CHANNEL_1
#define EMT_DIR_PORT       GPIOB          /* H 桥 IN1/IN2: PB0/PB1 */
#define EMT_DIR_PIN_FWD    GPIO_PIN_0
#define EMT_DIR_PIN_REV    GPIO_PIN_1

/* ---- 电机参数(按 JCA25-370-103K 资料) -------------------------------- */
#define EMT_ENCODER_LINES  11U       /* 每相每转信号数 */
#define EMT_GEAR_RATIO     103U      /* 减速比(型号 -103K) */
#define EMT_CPR            (EMT_ENCODER_LINES * 4U * EMT_GEAR_RATIO) /* 4532 */
#define EMT_PWM_FREQ_HZ    20000U
#define EMT_ENC_INVERT     0U
#define EMT_DIR_INVERT     0U
/* 空载 85rpm(100:1 标称值), 按 103:1 折算约 82.5rpm = 1.38rps, 取 1.35 留余量 */
#define EMT_RPS_LIMIT      1.35f
#define EMT_MAX_DUTY       1000.0f
#define EMT_PERIOD_S       0.01f     /* Task 周期 10ms */
/* PID 默认值仅作起点, 必须按实车整定(Kp 单位: ‰占空比 / rps);
   0.5rps 稳态约需 370‰, 故 Kp≈600 提供初值、Ki 补足静差 */
#define EMT_SPEED_KP       600.0f
#define EMT_SPEED_KI       800.0f
#define EMT_SPEED_KD       0.0f

/* ---- 测试参数 ----------------------------------------------------------- */
#define EMT_PERIOD_MS      10U
#define EMT_OPEN_DUTY      500      /* 开环测试占空比(‰) */
#define EMT_RUN_MS         1200U    /* 开环每方向观察时间 */
#define EMT_STOP_MS        1500U    /* 滑行/闭环回零观察时间 */
#define EMT_CLOSE_MS       1800U    /* 闭环稳定时间 */
#define EMT_STATUS_MS      500U     /* 实时状态打印周期 */
#define EMT_TARGET_RPS     0.5f     /* 闭环测试目标转速 */
#define EMT_CLOSE_TOL      0.25f    /* 闭环稳态误差容差(rps) */
#define EMT_MOVE_MIN       0.05f    /* 开环转动判定下限(rps) */
#define EMT_STOP_TOL       0.08f    /* 开环滑行停车判定 */
#define EMT_ZERO_TOL       0.12f    /* 闭环回零判定 */

typedef enum
{
  EMT_ST_INIT_FAIL = 0, /* Init 失败, 不输出 */
  EMT_ST_FWD,           /* 开环正转 */
  EMT_ST_REV,           /* 开环反转 */
  EMT_ST_STOP,          /* 滑行停车 */
  EMT_ST_CLOSE_FWD,     /* 闭环 +0.5rps */
  EMT_ST_CLOSE_REV,     /* 闭环 -0.5rps */
  EMT_ST_CLOSE_STOP,    /* 闭环 0rps */
  EMT_ST_CLAMP,         /* 目标限幅核对 */
  EMT_ST_DONE           /* 测试结束, 已停车 */
} EmtState_e;

static EncoderMotor_Object s_em_motor;
static EmtState_e s_em_state = EMT_ST_INIT_FAIL;
static uint32_t s_em_tick = 0U;
static uint32_t s_em_ctrl_tick = 0U;
static uint32_t s_em_status_tick = 0U;
static uint32_t s_em_checks = 0U;
static uint32_t s_em_fails = 0U;
static int32_t  s_em_count_ref = 0;
static uint32_t s_em_fault_ref = 0U;

static void EmtResult(const char *tag, uint8_t ok)
{
  s_em_checks++;
  if (!ok)
  {
    s_em_fails++;
  }
  UART_Port_Printf(&uart1_port, "[em] [%s] %s\r\n", (ok != 0U) ? "PASS" : "FAIL", tag);
}

/**
  * @brief  核对当前转速是否落在 [lo, hi]
  * @retval 1 在范围内
  */
static uint8_t EmtCheckRps(const char *tag, float lo, float hi)
{
  float act = EncoderMotor_GetSpeedRps(&s_em_motor);
  uint8_t ok = (act >= lo) && (act <= hi);

  s_em_checks++;
  if (!ok)
  {
    s_em_fails++;
  }
  UART_Port_Printf(&uart1_port, "[em] [%s] %s rps=%.2f range=[%.2f, %.2f]\r\n",
                   (ok != 0U) ? "PASS" : "FAIL", tag, act, lo, hi);
  return ok;
}

static void EmtEnter(EmtState_e st, uint32_t now)
{
  s_em_state = st;
  s_em_tick = now;

  switch (st)
  {
    case EMT_ST_FWD:
      s_em_count_ref = EncoderMotor_GetCount(&s_em_motor);
      EncoderMotor_SetDuty(&s_em_motor, EMT_OPEN_DUTY);
      break;

    case EMT_ST_REV:
      s_em_count_ref = EncoderMotor_GetCount(&s_em_motor);
      EncoderMotor_SetDuty(&s_em_motor, -EMT_OPEN_DUTY);
      break;

    case EMT_ST_STOP:
      EncoderMotor_Stop(&s_em_motor);
      break;

    case EMT_ST_CLOSE_FWD:
      s_em_fault_ref = s_em_motor.stats.fault_count;
      EncoderMotor_SetSpeedRps(&s_em_motor, EMT_TARGET_RPS);
      break;

    case EMT_ST_CLOSE_REV:
      s_em_fault_ref = s_em_motor.stats.fault_count;
      EncoderMotor_SetSpeedRps(&s_em_motor, -EMT_TARGET_RPS);
      break;

    case EMT_ST_CLOSE_STOP:
      EncoderMotor_SetSpeedRps(&s_em_motor, 0.0f);
      break;

    case EMT_ST_CLAMP:
      /* 999 应被钳到 rps_limit, 且不会真的去追 999 */
      EncoderMotor_SetSpeedRps(&s_em_motor, 999.0f);
      break;

    case EMT_ST_DONE:
      EncoderMotor_Stop(&s_em_motor);
      break;

    default:
      break;
  }
}

void EncoderMotor_Debug_Init(void)
{
  EncoderMotor_Config_t cfg;
  uint8_t ret;
  uint8_t ok;
  uint32_t now;

  cfg.name = "MOTOR1";
  cfg.enc_htim = EMT_ENC_TIM;
  cfg.pwm_htim = EMT_PWM_TIM;
  cfg.pwm_channel = EMT_PWM_CH;
  cfg.dir_port = EMT_DIR_PORT;
  cfg.dir_pin_fwd = EMT_DIR_PIN_FWD;
  cfg.dir_pin_rev = EMT_DIR_PIN_REV;
  cfg.pwm_freq_hz = EMT_PWM_FREQ_HZ;
  cfg.counts_per_rev = EMT_CPR;
  cfg.enc_invert = EMT_ENC_INVERT;
  cfg.dir_invert = EMT_DIR_INVERT;
  cfg.rps_limit = EMT_RPS_LIMIT;
  cfg.speed_kp = EMT_SPEED_KP;
  cfg.speed_ki = EMT_SPEED_KI;
  cfg.speed_kd = EMT_SPEED_KD;
  cfg.max_duty = EMT_MAX_DUTY;
  cfg.control_period_s = EMT_PERIOD_S;

  UART_Port_Printf(&uart1_port,
                   "[em] init: TIM2 ENC + TIM1_CH1 %luHz PWM + PB0/PB1 dir | cpr=%lu limit=%.2frps\r\n",
                   (unsigned long)EMT_PWM_FREQ_HZ, (unsigned long)EMT_CPR, EMT_RPS_LIMIT);

  ret = EncoderMotor_Init(&s_em_motor, &cfg);
  if (ret != 0U)
  {
    s_em_state = EMT_ST_INIT_FAIL;
    UART_Port_Printf(&uart1_port,
                     "[em] [FAIL] init ret=%u (1=参数, 2=编码器, 3=PWM) enc=%u pwm=%u\r\n",
                     (unsigned)ret, (unsigned)s_em_motor.last_enc_ret,
                     (unsigned)s_em_motor.last_pwm_ret);
    return;
  }

  /* 开环占空比限幅: 1500‰ 应被钳到 max_duty, 只维持一个调用瞬间后归零 */
  EncoderMotor_SetDuty(&s_em_motor, 1500);
  ok = (EncoderMotor_GetDuty(&s_em_motor) == (int16_t)EMT_MAX_DUTY) ? 1U : 0U;
  EmtResult("duty clamp +1500 -> max_duty", ok);
  EncoderMotor_SetDuty(&s_em_motor, 0);

  now = HAL_GetTick();
  s_em_ctrl_tick = now;
  s_em_status_tick = now;
  UART_Port_Printf(&uart1_port, "[em] test start: motor WILL spin, keep wheels free!\r\n");
  EmtEnter(EMT_ST_FWD, now);
}

void EncoderMotor_Debug_Run(uint32_t now)
{
  if (s_em_state == EMT_ST_INIT_FAIL || s_em_state == EMT_ST_DONE)
  {
    return;
  }

  /* 控制环: 严格按 10ms 周期跑 Task(状态机与打印不阻塞) */
  if ((now - s_em_ctrl_tick) >= EMT_PERIOD_MS)
  {
    s_em_ctrl_tick = now;
    EncoderMotor_Task(&s_em_motor);
  }

  /* 实时状态: 便于观察 PID 收敛过程 */
  if ((now - s_em_status_tick) >= EMT_STATUS_MS)
  {
    s_em_status_tick = now;
    UART_Port_Printf(&uart1_port,
                     "[em] status | mode=%u target=%.2f rps=%.2f duty=%d count=%ld | tasks=%lu faults=%lu\r\n",
                     (unsigned)EncoderMotor_GetMode(&s_em_motor),
                     EncoderMotor_GetTargetRps(&s_em_motor),
                     EncoderMotor_GetSpeedRps(&s_em_motor),
                     (int)EncoderMotor_GetDuty(&s_em_motor),
                     (long)EncoderMotor_GetCount(&s_em_motor),
                     (unsigned long)s_em_motor.stats.task_count,
                     (unsigned long)s_em_motor.stats.fault_count);
  }

  switch (s_em_state)
  {
    case EMT_ST_FWD:
      if ((now - s_em_tick) >= EMT_RUN_MS)
      {
        int32_t delta = EncoderMotor_GetCount(&s_em_motor) - s_em_count_ref;

        (void)EmtCheckRps("open-loop +500 duty", EMT_MOVE_MIN, 10.0f);
        UART_Port_Printf(&uart1_port, "[em] fwd count delta=%ld\r\n", (long)delta);
        EmtResult("fwd count increasing", (delta > 0) ? 1U : 0U);
        EmtEnter(EMT_ST_REV, now);
      }
      break;

    case EMT_ST_REV:
      if ((now - s_em_tick) >= EMT_RUN_MS)
      {
        int32_t delta = EncoderMotor_GetCount(&s_em_motor) - s_em_count_ref;

        (void)EmtCheckRps("open-loop -500 duty", -10.0f, -EMT_MOVE_MIN);
        UART_Port_Printf(&uart1_port, "[em] rev count delta=%ld\r\n", (long)delta);
        EmtResult("rev count decreasing", (delta < 0) ? 1U : 0U);
        EmtEnter(EMT_ST_STOP, now);
      }
      break;

    case EMT_ST_STOP:
      if ((now - s_em_tick) >= EMT_STOP_MS)
      {
        (void)EmtCheckRps("coast to stop", -EMT_STOP_TOL, EMT_STOP_TOL);
        EmtEnter(EMT_ST_CLOSE_FWD, now);
      }
      break;

    case EMT_ST_CLOSE_FWD:
      if ((now - s_em_tick) >= EMT_CLOSE_MS)
      {
        (void)EmtCheckRps("closed-loop +0.5rps", EMT_TARGET_RPS - EMT_CLOSE_TOL,
                          EMT_TARGET_RPS + EMT_CLOSE_TOL);
        EmtResult("no overspeed fault (fwd)",
                  (s_em_motor.stats.fault_count == s_em_fault_ref) ? 1U : 0U);
        EmtEnter(EMT_ST_CLOSE_REV, now);
      }
      break;

    case EMT_ST_CLOSE_REV:
      if ((now - s_em_tick) >= EMT_CLOSE_MS)
      {
        (void)EmtCheckRps("closed-loop -0.5rps", -EMT_TARGET_RPS - EMT_CLOSE_TOL,
                          -EMT_TARGET_RPS + EMT_CLOSE_TOL);
        EmtResult("no overspeed fault (rev)",
                  (s_em_motor.stats.fault_count == s_em_fault_ref) ? 1U : 0U);
        EmtEnter(EMT_ST_CLOSE_STOP, now);
      }
      break;

    case EMT_ST_CLOSE_STOP:
      if ((now - s_em_tick) >= EMT_STOP_MS)
      {
        (void)EmtCheckRps("closed-loop 0rps", -EMT_ZERO_TOL, EMT_ZERO_TOL);
        EmtEnter(EMT_ST_CLAMP, now);
      }
      break;

    case EMT_ST_CLAMP:
    {
      float target = EncoderMotor_GetTargetRps(&s_em_motor);
      uint8_t ok = (FLOAT_ABS(target - EMT_RPS_LIMIT) < 0.001f) ? 1U : 0U;

      UART_Port_Printf(&uart1_port, "[em] clamp target=%.2f (expect %.2f)\r\n",
                       target, EMT_RPS_LIMIT);
      EmtResult("target clamp 999 -> rps_limit", ok);
      EmtResult("stats task_count > 0", (s_em_motor.stats.task_count > 0U) ? 1U : 0U);
      EmtResult("stats stop_count > 0", (s_em_motor.stats.stop_count > 0U) ? 1U : 0U);

      UART_Port_Printf(&uart1_port,
                       "[em] test done: %lu checks, %lu failed%s | tasks=%lu stops=%lu faults=%lu\r\n",
                       (unsigned long)s_em_checks, (unsigned long)s_em_fails,
                       (s_em_fails == 0U) ? " [PASS]" : " [FAIL]",
                       (unsigned long)s_em_motor.stats.task_count,
                       (unsigned long)s_em_motor.stats.stop_count,
                       (unsigned long)s_em_motor.stats.fault_count);
      EmtEnter(EMT_ST_DONE, now);
      break;
    }

    default:
      break;
  }
}

#endif /* encoder_motor_debug == 1 */