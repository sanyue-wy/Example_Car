#include "at8236_debug.h"

#if at8236_debug == 1

#include "at8236.h"
#include "drv_uart.h"
#include "usart.h"
#include "main.h"
#include <stdio.h>

/* ---- 第18层 at8236_debug: 四路 AT8236 串口驱动板自检(电机会转! 先架空车轮) ----
   驱动板通过 USART2(PA2/PA3, 115200 8N1) 通讯, 板上固件已实现 4 路电机控制与
   编码器换算, 本文件验证"配置 -> 上报 -> 开环 -> 闭环"整条链路:
     配置 5 条 -> 检查回显 $<指令>OK
     打开累计编码器上报 -> 检查 $MAll 帧
     PWM +1500 1.2s -> 4 路计数变化且方向一致, 记录各自符号
     滑行 1.0s       -> 避免正反转直接切换(电机手册禁止瞬间反向)
     PWM -1500 1.2s -> 4 路计数方向与上一段相反
     滑行 1.0s
     打开速度上报   -> 检查 $MSPD 帧
     速度 +150mm/s 2s -> 4 路速度落在 (40, 400) mm/s
     速度 0 1s       -> 4 路速度回落到 ±50 mm/s 内
     收尾: 关上报 + 滑行停车 + 汇总
   每项 [PASS]/[FAIL], 只跑一轮, 不复位重跑。

   本工程电机 JCA25-370-103K(12V 4W 减速电机 + 霍尔编码器):
   - 编码器 11 线, 减速比 103, 输出轴一圈 11×4×103 = 4532 计数;
   - 12V 空载 85rpm, 67mm 轮对应约 0.30m/s, 所以速度测试用 150mm/s;
   - 资料明确"禁止堵转或瞬间负载反转", 所以正反转之间必须滑行;
   - 速度闭环依赖 $mline/$mphase/$wdiameter 配置正确, 否则轮子会飞车或
     速度读数不对 —— 本自检先写配置再使能上报。

   注意:
   1. CubeMX 必须给 USART2 打开 RX/TX DMA(drv_uart 用 DMA + 空闲中断),
      否则 Init 返回 AT8236_ERR_UART, 本文件会打印具体步骤;
   2. 配置指令写驱动板 flash, 两次之间留 150ms; PID 指令会让驱动板重启,
      本自检不下发 PID, 保持出厂值;
   3. M1~M4 的物理安装方向可能左右镜像, 因此方向判定不用绝对正负, 而是
      "PWM 反向后计数方向与正向相反";
   4. 与 bt_vision_debug 互斥(同占 USART2), 已在 at8236_debug.h 编译期检查。 */

/* ---- 硬件绑定(换串口只改这里) ------------------------------------------ */
#define ATD_UART           (&huart2)   /* PA2=TX -> 驱动板 RX2, PA3=RX <- 驱动板 TX2 */
#define ATD_NAME           "AT8236"

/* ---- 测试参数 ----------------------------------------------------------- */
#define ATD_CFG_GAP_MS     150U     /* 两条配置指令间隔(驱动板要写 flash) */
#define ATD_ACK_WAIT_MS    300U     /* 配置回显等待时间 */
#define ATD_UP_WAIT_MS     500U     /* 打开上报后等首帧 */
#define ATD_PWM            1500     /* 开环测试 PWM(‰, 约 42%) */
#define ATD_PWM_MS         1200U    /* 每个开环方向观察时间 */
#define ATD_COAST_MS       1000U    /* 滑行/换向前等待 */
#define ATD_SPEED_MMPS     150      /* 闭环测试目标速度 mm/s */
#define ATD_SPEED_MS       2000U    /* 闭环稳定时间 */
#define ATD_ZERO_MS        1000U    /* 回零观察时间 */
#define ATD_STATUS_MS      500U     /* 实时状态打印周期 */
#define ATD_MOVE_COUNTS    200L     /* 1.2s 内至少变化的编码器计数 */
#define ATD_SPEED_MIN      40.0f    /* 闭环速度下限 */
#define ATD_SPEED_MAX      400.0f   /* 闭环速度上限(超过 1000 指令也无效) */
#define ATD_STOP_TOL       50.0f    /* 回零容差 mm/s */

typedef enum
{
  ATD_ST_INIT_FAIL = 0, /* Init 失败, 不动作 */
  ATD_ST_CFG_TYPE,      /* $mtype:2# */
  ATD_ST_CFG_LINE,      /* $mline:11# */
  ATD_ST_CFG_RATIO,     /* $mphase:103# */
  ATD_ST_CFG_DIA,       /* $wdiameter:67.000# */
  ATD_ST_CFG_DEAD,      /* $deadzone:1600# */
  ATD_ST_CFG_VERIFY,    /* 等 5 条配置回显 */
  ATD_ST_UP_TOTAL,      /* $upload:1,0,0# */
  ATD_ST_PWM_FWD,       /* $pwm:+1500# */
  ATD_ST_COAST_FWD,     /* $pwm:0# 换向前滑行 */
  ATD_ST_PWM_REV,       /* $pwm:-1500# */
  ATD_ST_COAST,         /* $pwm:0# 进闭环前滑行 */
  ATD_ST_UP_SPEED,      /* $upload:0,0,1# */
  ATD_ST_SPD_FWD,       /* $spd:+150# */
  ATD_ST_SPD_ZERO,      /* $spd:0# */
  ATD_ST_STOP,          /* 收尾滑行停车 */
  ATD_ST_UPLOAD_OFF,    /* 收尾关上报 */
  ATD_ST_DONE           /* 收尾完成 */
} AtdState_e;

static AT8236_Object s_dev;
static AtdState_e    s_state = ATD_ST_INIT_FAIL;
static uint32_t      s_tick = 0U;
static uint32_t      s_status_tick = 0U;
static uint32_t      s_checks = 0U;
static uint32_t      s_fails = 0U;
static uint32_t      s_ack_ref = 0U;
static uint32_t      s_frame_ref = 0U;
static uint8_t       s_last_ret = AT8236_OK;
static int32_t       s_total_ref[AT8236_MOTOR_NUM];
static int8_t        s_dir_ref[AT8236_MOTOR_NUM];

/* ---- 结果输出 ----------------------------------------------------------- */
static void AtResult(const char *tag, uint8_t ok)
{
  s_checks++;
  if (ok == 0U)
  {
    s_fails++;
  }
  UART_Port_Printf(&uart1_port, "[at8236] [%s] %s\r\n",
                   (ok != 0U) ? "PASS" : "FAIL", tag);
}

static void AtResultMotor(const char *what, uint8_t index, uint8_t ok)
{
  s_checks++;
  if (ok == 0U)
  {
    s_fails++;
  }
  UART_Port_Printf(&uart1_port, "[at8236] [%s] %s M%u\r\n",
                   (ok != 0U) ? "PASS" : "FAIL", what, (unsigned)(index + 1U));
}

/* newlib-nano 无 float printf: 四舍五入到整数 */
static long AtdRound(float value)
{
  return (long)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void AtdCmd(const char *what, uint8_t ret)
{
  s_last_ret = ret;
  UART_Port_Printf(&uart1_port, "[at8236] %s -> ret=%u\r\n", what, (unsigned)ret);
}

static uint32_t AtdMag(int32_t value)
{
  return (uint32_t)((value >= 0) ? value : -value);
}

/* ---- 状态入口动作 ------------------------------------------------------- */
static void AtdEnter(AtdState_e state, uint32_t now)
{
  s_state = state;
  s_tick = now;

  switch (state)
  {
    case ATD_ST_CFG_TYPE:
      AtdCmd("config $mtype", AT8236_ConfigMotorType(&s_dev, s_dev.cfg.motor_type));
      break;

    case ATD_ST_CFG_LINE:
      AtdCmd("config $mline", AT8236_ConfigEncoderLines(&s_dev, s_dev.cfg.encoder_lines));
      break;

    case ATD_ST_CFG_RATIO:
      AtdCmd("config $mphase", AT8236_ConfigGearRatio(&s_dev, s_dev.cfg.gear_ratio));
      break;

    case ATD_ST_CFG_DIA:
      AtdCmd("config $wdiameter",
             AT8236_ConfigWheelDiameter(&s_dev, s_dev.cfg.wheel_diameter_mm));
      break;

    case ATD_ST_CFG_DEAD:
      AtdCmd("config $deadzone", AT8236_ConfigDeadzone(&s_dev, s_dev.cfg.deadzone));
      break;

    case ATD_ST_CFG_VERIFY:
      /* 只等待, 检查在 Run 里做 */
      break;

    case ATD_ST_UP_TOTAL:
      s_frame_ref = s_dev.stats.total_frames;
      AtdCmd("upload total encoder", AT8236_SetUpload(&s_dev, 1U, 0U, 0U));
      break;

    case ATD_ST_PWM_FWD:
      for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
      {
        s_total_ref[i] = AT8236_GetEncoderTotal(&s_dev, i);
      }
      AtdCmd("pwm +1500 all", AT8236_SetPwm(&s_dev, ATD_PWM, ATD_PWM, ATD_PWM, ATD_PWM));
      break;

    case ATD_ST_COAST_FWD:
      AtdCmd("coast before reverse", AT8236_Coast(&s_dev));
      break;

    case ATD_ST_PWM_REV:
      for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
      {
        s_total_ref[i] = AT8236_GetEncoderTotal(&s_dev, i);
      }
      AtdCmd("pwm -1500 all", AT8236_SetPwm(&s_dev, -ATD_PWM, -ATD_PWM, -ATD_PWM, -ATD_PWM));
      break;

    case ATD_ST_COAST:
      AtdCmd("coast before close loop", AT8236_Coast(&s_dev));
      break;

    case ATD_ST_UP_SPEED:
      s_frame_ref = s_dev.stats.speed_frames;
      AtdCmd("upload speed", AT8236_SetUpload(&s_dev, 0U, 0U, 1U));
      break;

    case ATD_ST_SPD_FWD:
      AtdCmd("speed +150mm/s all", AT8236_SetSpeed(&s_dev, ATD_SPEED_MMPS,
                                                   ATD_SPEED_MMPS,
                                                   ATD_SPEED_MMPS,
                                                   ATD_SPEED_MMPS));
      break;

    case ATD_ST_SPD_ZERO:
      AtdCmd("speed 0 all", AT8236_SetSpeed(&s_dev, 0, 0, 0, 0));
      break;

    case ATD_ST_STOP:
      AtdCmd("final coast", AT8236_Coast(&s_dev));
      break;

    case ATD_ST_UPLOAD_OFF:
      AtdCmd("upload off", AT8236_SetUpload(&s_dev, 0U, 0U, 0U));
      break;

    case ATD_ST_DONE:
      UART_Port_Printf(&uart1_port,
                       "[at8236] done | checks=%lu fails=%lu | frames=%lu ack=%lu "
                       "total=%lu speed=%lu bat=%lu parse_err=%lu ovf=%lu "
                       "tx_ok=%lu tx_busy=%lu sat=%lu timeout=%lu\r\n",
                       (unsigned long)s_checks, (unsigned long)s_fails,
                       (unsigned long)s_dev.stats.rx_frames,
                       (unsigned long)s_dev.stats.ack_frames,
                       (unsigned long)s_dev.stats.total_frames,
                       (unsigned long)s_dev.stats.speed_frames,
                       (unsigned long)s_dev.stats.battery_frames,
                       (unsigned long)s_dev.stats.parse_errors,
                       (unsigned long)s_dev.stats.overflow,
                       (unsigned long)s_dev.stats.tx_ok,
                       (unsigned long)s_dev.stats.tx_busy,
                       (unsigned long)s_dev.stats.sat_count,
                       (unsigned long)s_dev.stats.timeout_stops);
      UART_Port_Printf(&uart1_port,
                       "[at8236] uart rx=%lu err=%lu tx_err=%lu | last_resp=\"%s\"\r\n",
                       (unsigned long)s_dev.uart.stats.rx_count,
                       (unsigned long)s_dev.uart.stats.err_count,
                       (unsigned long)s_dev.uart.stats.tx_error,
                       AT8236_GetLastResponse(&s_dev));
      break;

    default:
      break;
  }
}

/* ---- 初始化 ------------------------------------------------------------- */
void AT8236_Debug_Init(void)
{
  AT8236_Config_t cfg;
  uint32_t now = HAL_GetTick();
  uint8_t ret;

  s_checks = 0U;
  s_fails = 0U;

  AT8236_DefaultConfig(&cfg, ATD_UART, ATD_NAME);
  cfg.command_timeout_ms = 0U; /* 自检自己负责停车 */

  ret = AT8236_Init(&s_dev, &cfg);
  if (ret != (uint8_t)AT8236_OK)
  {
    s_state = ATD_ST_INIT_FAIL;
    UART_Port_Printf(&uart1_port,
                     "[at8236] Init FAIL ret=%u uart_init_ret=%u\r\n",
                     (unsigned)ret, (unsigned)s_dev.uart_init_ret);
    UART_Port_Printf(&uart1_port,
                     "[at8236] drv_uart needs USART2 RX/TX DMA. CubeMX: USART2 -> "
                     "DMA Settings -> add USART2_RX (DMA1 Ch6) and USART2_TX "
                     "(DMA1 Ch7); NVIC enable DMA1_Channel6/7 IRQ; regenerate.\r\n");
    UART_Port_Printf(&uart1_port,
                     "[at8236] wiring: PA2->board RX2, PA3<-board TX2, GND common, "
                     "board motor supply 12V.\r\n");
    return;
  }

  UART_Port_Printf(&uart1_port,
                   "[at8236] test start: 4 motors WILL spin, keep wheels free!\r\n");
  UART_Port_Printf(&uart1_port,
                   "[at8236] JCA25-370-103K: 11 lines x 103 ratio, wheel=%ldmm, "
                   "deadzone=%u; config writes board flash, wait %ums between cmds\r\n",
                   AtdRound(s_dev.cfg.wheel_diameter_mm),
                   (unsigned)s_dev.cfg.deadzone,
                   (unsigned)ATD_CFG_GAP_MS);

  s_ack_ref = s_dev.stats.ack_frames;
  s_status_tick = now;
  AtdEnter(ATD_ST_CFG_TYPE, now);
}

/* ---- 周期运行 ----------------------------------------------------------- */
void AT8236_Debug_Run(uint32_t now)
{
  if ((s_state == ATD_ST_INIT_FAIL) || (s_state == ATD_ST_DONE))
  {
    return;
  }

  /* 指令看门狗(自检里 timeout=0, 仅保持接口调用习惯) */
  AT8236_Task(&s_dev, now);

  if ((now - s_status_tick) >= ATD_STATUS_MS)
  {
    s_status_tick = now;
    UART_Port_Printf(&uart1_port,
                     "[at8236] status | total=%ld,%ld,%ld,%ld "
                     "delta=%ld,%ld,%ld,%ld spd=%ld,%ld,%ld,%ld | "
                     "frames=%lu ack=%lu bat_mV=%ld online=%u\r\n",
                     (long)AT8236_GetEncoderTotal(&s_dev, 0U),
                     (long)AT8236_GetEncoderTotal(&s_dev, 1U),
                     (long)AT8236_GetEncoderTotal(&s_dev, 2U),
                     (long)AT8236_GetEncoderTotal(&s_dev, 3U),
                     (long)AT8236_GetEncoderDelta(&s_dev, 0U),
                     (long)AT8236_GetEncoderDelta(&s_dev, 1U),
                     (long)AT8236_GetEncoderDelta(&s_dev, 2U),
                     (long)AT8236_GetEncoderDelta(&s_dev, 3U),
                     AtdRound(AT8236_GetSpeedMmps(&s_dev, 0U)),
                     AtdRound(AT8236_GetSpeedMmps(&s_dev, 1U)),
                     AtdRound(AT8236_GetSpeedMmps(&s_dev, 2U)),
                     AtdRound(AT8236_GetSpeedMmps(&s_dev, 3U)),
                     (unsigned long)s_dev.stats.rx_frames,
                     (unsigned long)s_dev.stats.ack_frames,
                     AtdRound(AT8236_GetBatteryVoltage(&s_dev) * 1000.0f),
                     (unsigned)AT8236_IsOnline(&s_dev, now, 1000U));
  }

  switch (s_state)
  {
    case ATD_ST_CFG_TYPE:
      if ((now - s_tick) >= ATD_CFG_GAP_MS)
      {
        AtResult("config mtype accepted by UART", s_last_ret == (uint8_t)AT8236_OK);
        AtdEnter(ATD_ST_CFG_LINE, now);
      }
      break;

    case ATD_ST_CFG_LINE:
      if ((now - s_tick) >= ATD_CFG_GAP_MS)
      {
        AtResult("config mline accepted by UART", s_last_ret == (uint8_t)AT8236_OK);
        AtdEnter(ATD_ST_CFG_RATIO, now);
      }
      break;

    case ATD_ST_CFG_RATIO:
      if ((now - s_tick) >= ATD_CFG_GAP_MS)
      {
        AtResult("config mphase accepted by UART", s_last_ret == (uint8_t)AT8236_OK);
        AtdEnter(ATD_ST_CFG_DIA, now);
      }
      break;

    case ATD_ST_CFG_DIA:
      if ((now - s_tick) >= ATD_CFG_GAP_MS)
      {
        AtResult("config wdiameter accepted by UART", s_last_ret == (uint8_t)AT8236_OK);
        AtdEnter(ATD_ST_CFG_DEAD, now);
      }
      break;

    case ATD_ST_CFG_DEAD:
      if ((now - s_tick) >= ATD_CFG_GAP_MS)
      {
        AtResult("config deadzone accepted by UART", s_last_ret == (uint8_t)AT8236_OK);
        AtdEnter(ATD_ST_CFG_VERIFY, now);
      }
      break;

    case ATD_ST_CFG_VERIFY:
      if ((now - s_tick) >= ATD_ACK_WAIT_MS)
      {
        uint32_t acks = s_dev.stats.ack_frames - s_ack_ref;

        UART_Port_Printf(&uart1_port, "[at8236] config acks=%lu last_resp=\"%s\"\r\n",
                         (unsigned long)acks, AT8236_GetLastResponse(&s_dev));
        AtResult("5 config commands echoed OK", (acks >= 5U) ? 1U : 0U);
        AtdEnter(ATD_ST_UP_TOTAL, now);
      }
      break;

    case ATD_ST_UP_TOTAL:
      if ((now - s_tick) >= ATD_UP_WAIT_MS)
      {
        AtResult("total encoder upload frames", (s_dev.stats.total_frames > s_frame_ref) ? 1U : 0U);
        AtdEnter(ATD_ST_PWM_FWD, now);
      }
      break;

    case ATD_ST_PWM_FWD:
      if ((now - s_tick) >= ATD_PWM_MS)
      {
        for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
        {
          int32_t d = AT8236_GetEncoderTotal(&s_dev, i) - s_total_ref[i];

          s_dir_ref[i] = (int8_t)((d >= 0) ? 1 : -1);
          UART_Port_Printf(&uart1_port, "[at8236] pwm+ M%u delta=%ld\r\n",
                           (unsigned)(i + 1U), (long)d);
          AtResultMotor("pwm +1500 moves", i, (AtdMag(d) >= (uint32_t)ATD_MOVE_COUNTS) ? 1U : 0U);
        }
        AtdEnter(ATD_ST_COAST_FWD, now);
      }
      break;

    case ATD_ST_COAST_FWD:
      if ((now - s_tick) >= ATD_COAST_MS)
      {
        AtdEnter(ATD_ST_PWM_REV, now);
      }
      break;

    case ATD_ST_PWM_REV:
      if ((now - s_tick) >= ATD_PWM_MS)
      {
        for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
        {
          int32_t d = AT8236_GetEncoderTotal(&s_dev, i) - s_total_ref[i];
          uint8_t moved = (AtdMag(d) >= (uint32_t)ATD_MOVE_COUNTS) ? 1U : 0U;
          uint8_t dir_ok = 0U;

          if (d != 0)
          {
            dir_ok = (((d > 0) ? 1 : -1) == (int)-s_dir_ref[i]) ? 1U : 0U;
          }

          UART_Port_Printf(&uart1_port, "[at8236] pwm- M%u delta=%ld\r\n",
                           (unsigned)(i + 1U), (long)d);
          AtResultMotor("pwm -1500 reverses", i, (moved != 0U) && (dir_ok != 0U));
        }
        AtdEnter(ATD_ST_COAST, now);
      }
      break;

    case ATD_ST_COAST:
      if ((now - s_tick) >= ATD_COAST_MS)
      {
        AtdEnter(ATD_ST_UP_SPEED, now);
      }
      break;

    case ATD_ST_UP_SPEED:
      if ((now - s_tick) >= ATD_UP_WAIT_MS)
      {
        AtResult("speed upload frames", (s_dev.stats.speed_frames > s_frame_ref) ? 1U : 0U);
        AtdEnter(ATD_ST_SPD_FWD, now);
      }
      break;

    case ATD_ST_SPD_FWD:
      if ((now - s_tick) >= ATD_SPEED_MS)
      {
        for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
        {
          float v = AT8236_GetSpeedMmps(&s_dev, i);

          UART_Port_Printf(&uart1_port, "[at8236] speed M%u = %ld mm/s\r\n",
                           (unsigned)(i + 1U), AtdRound(v));
          AtResultMotor("speed +150 closed loop", i,
                        ((v >= ATD_SPEED_MIN) && (v <= ATD_SPEED_MAX)) ? 1U : 0U);
        }
        AtdEnter(ATD_ST_SPD_ZERO, now);
      }
      break;

    case ATD_ST_SPD_ZERO:
      if ((now - s_tick) >= ATD_ZERO_MS)
      {
        for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
        {
          float v = AT8236_GetSpeedMmps(&s_dev, i);

          UART_Port_Printf(&uart1_port, "[at8236] zero  M%u = %ld mm/s\r\n",
                           (unsigned)(i + 1U), AtdRound(v));
          AtResultMotor("speed 0 holds", i,
                        ((v >= -ATD_STOP_TOL) && (v <= ATD_STOP_TOL)) ? 1U : 0U);
        }
        AtdEnter(ATD_ST_STOP, now);
      }
      break;

    case ATD_ST_STOP:
      if ((now - s_tick) >= ATD_CFG_GAP_MS)
      {
        AtdEnter(ATD_ST_UPLOAD_OFF, now);
      }
      break;

    case ATD_ST_UPLOAD_OFF:
      if ((now - s_tick) >= ATD_CFG_GAP_MS)
      {
        AtdEnter(ATD_ST_DONE, now);
      }
      break;

    default:
      break;
  }
}

#endif /* at8236_debug == 1 */
