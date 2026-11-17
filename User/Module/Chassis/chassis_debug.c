#include "chassis_debug.h"

#if chassis_debug == 1

#include "chassis.h"
#include "at8236.h"
#include "drv_uart.h"
#include "usart.h"
#include "main.h"

/* ---- 第19层 chassis_debug: 四轮差速底盘 + AT8236 驱动板自检 ----------------
   本层验证"底盘语义 -> 差速运动学 -> 四轮速度 -> 驱动板 -> 电机/编码器"
   整条链路, 同时检查里程计与目标租约:
     配置驱动板 5 条 -> 检查回显 OK
     打开总编码器 + 速度上报 -> 检查 $MAll/$MSPD 帧
     前进 150mm/s 1.5s -> 四轮同向转动 + 里程计 x 增加 + 实测速度 >60mm/s
     减速停车 1.2s   -> 模式进 HOLD, 实测速度回零
     左转 60°/s 1.5s -> 左轮反转/右轮正转 + 航向角增加
     右转 60°/s 1.5s -> 左轮正转/右轮反转 + 航向角减小
     目标租约 400ms  -> 不再刷新目标, 应自动减速到 HOLD
   每项 [PASS]/[FAIL], 只跑一轮; 结束主动保持 + 关上报。

   注意:
   1. 电机会转! 必须架空车轮, 并确保四周没有人员/线缆;
   2. 方向检查以"四轮同向/左右差速"为准, 不假设某侧绝对正负。若 FAIL:
      - 某一路反向: 对应位写进 chassis cfg.invert_mask(bit0=M1 ... bit3=M4);
      - 左右差速相反: 先核对 M1/M2 是否都在左侧、M3/M4 是否都在右侧;
      - 航向角符号相反: 通常是左右轮接反或 invert_mask 配反;
   3. 里程计依赖 $mline=11、$mphase=103、$wdiameter=67 与实际轮径/轮距,
      本自检会写入 67mm, 装车后请实测修改 ATD_WHEEL_MM / ATD_TRACK_MM;
   4. 无 float printf(newlib-nano), 本文件所有浮点按四舍五入成整数打印。 */

/* ---- 硬件/机械参数(装车实测后修改) ------------------------------------- */
#define CD_UART           (&huart2)
#define CD_NAME           "AT8236"
#define CD_TRACK_MM       150.0f   /* 左右轮中心距 */
#define CD_WHEEL_MM       67.0f    /* 轮径 */
#define CD_COUNTS_REV     4532U    /* 11 线 × 4 倍频 × 103 减速比 */
#define CD_INVERT_MASK    0x00U    /* 逐轮换向, bit0=M1 ... bit3=M4 */

/* ---- 测试参数 ----------------------------------------------------------- */
#define CD_CFG_GAP_MS     150U     /* 两条配置指令间隔(写 flash) */
#define CD_ACK_WAIT_MS    300U
#define CD_UP_WAIT_MS     500U
#define CD_CTRL_PERIOD_MS 10U      /* Chassis_Task 周期 */
#define CD_FWD_MMPS       150.0f
#define CD_FWD_MS         1500U
#define CD_TURN_DPS       60.0f
#define CD_TURN_MS        1500U
#define CD_STOP_MS        1200U
#define CD_WD_TIMEOUT_MS  400U     /* 目标租约 */
#define CD_WD_TEST_MS     900U
#define CD_STATUS_MS      500U
#define CD_MOVE_COUNTS    150L     /* 单轮最少计数变化 */
#define CD_SPEED_MIN      60.0f    /* 实测线速度下限 */
#define CD_SPEED_MAX      280.0f
#define CD_WS_MIN         15.0f    /* 实测角速度下限 */
#define CD_STOP_TOL       40.0f    /* 停车容差 mm/s */
#define CD_STOP_W_TOL     20.0f    /* 停车容差 °/s */
#define CD_YAW_MIN        0.10f    /* 转向航向角最小变化 rad */
#define CD_RAD2DEG        57.2957795f

typedef enum
{
  CD_ST_INIT_FAIL = 0,
  CD_ST_CFG_TYPE,
  CD_ST_CFG_LINE,
  CD_ST_CFG_RATIO,
  CD_ST_CFG_DIA,
  CD_ST_CFG_DEAD,
  CD_ST_CFG_VERIFY,
  CD_ST_UP,
  CD_ST_FWD,
  CD_ST_STOP1,
  CD_ST_TURN_L,
  CD_ST_STOP2,
  CD_ST_TURN_R,
  CD_ST_STOP3,
  CD_ST_WATCHDOG,
  CD_ST_STOP4,
  CD_ST_DONE
} CdState_e;

static AT8236_Object s_board;
static Chassis_Object s_chassis;
static CdState_e s_state = CD_ST_INIT_FAIL;
static uint32_t s_tick;
static uint32_t s_ctrl_tick;
static uint32_t s_status_tick;
static uint32_t s_checks;
static uint32_t s_fails;
static uint32_t s_ack_ref;
static uint32_t s_total_ref;
static uint32_t s_speed_ref;
static uint32_t s_timeout_ref;
static uint8_t  s_last_ret;
static int32_t  s_count_ref[CHASSIS_MOTOR_NUM];
static Chassis_Odometry_t s_odom_ref;

/* ---- 打印/判定工具 ------------------------------------------------------ */

/* newlib-nano 无 float printf: 四舍五入到整数 */
static long CdRound(float value)
{
  return (long)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static uint32_t CdMag(int32_t value)
{
  return (uint32_t)((value >= 0) ? value : -value);
}

static int8_t CdSign(int32_t value)
{
  return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
}

static float CdAbsF(float value)
{
  return (value < 0.0f) ? -value : value;
}

static void CdResult(const char *tag, uint8_t ok)
{
  s_checks++;
  if (ok == 0U)
  {
    s_fails++;
  }
  UART_Port_Printf(&uart1_port, "[cd] [%s] %s\r\n",
                   (ok != 0U) ? "PASS" : "FAIL", tag);
}

static void CdResultMotor(const char *what, uint8_t index, uint8_t ok)
{
  s_checks++;
  if (ok == 0U)
  {
    s_fails++;
  }
  UART_Port_Printf(&uart1_port, "[cd] [%s] %s M%u\r\n",
                   (ok != 0U) ? "PASS" : "FAIL", what, (unsigned)(index + 1U));
}

static void CdCmd(const char *what, uint8_t ret)
{
  s_last_ret = ret;
  UART_Port_Printf(&uart1_port, "[cd] %s -> ret=%u\r\n", what, (unsigned)ret);
}

static void CdSnapshotCounts(void)
{
  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    s_count_ref[i] = AT8236_GetEncoderTotal(&s_board, i);
  }
}

static void CdSnapshotOdom(void)
{
  Chassis_GetOdometry(&s_chassis, &s_odom_ref);
}

/* 前进方向检查: 四轮同向 + 幅值; 同时给出 invert_mask 建议 */
static void CdCheckForward(const char *what)
{
  int32_t delta[CHASSIS_MOTOR_NUM];
  uint8_t neg = 0U;
  uint8_t all_same = 1U;
  uint8_t suggest = 0U;
  int8_t majority;

  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    delta[i] = AT8236_GetEncoderTotal(&s_board, i) - s_count_ref[i];
    if (CdSign(delta[i]) < 0)
    {
      neg++;
    }
    UART_Port_Printf(&uart1_port, "[cd] %s M%u delta=%ld\r\n",
                     what, (unsigned)(i + 1U), (long)delta[i]);
  }

  majority = (neg >= 2U) ? -1 : 1;

  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    if (CdSign(delta[i]) != majority)
    {
      all_same = 0U;
      suggest |= (uint8_t)(1U << i);
    }
    CdResultMotor(what, i,
                  ((CdMag(delta[i]) >= (uint32_t)CD_MOVE_COUNTS) &&
                   (CdSign(delta[i]) == majority)) ? 1U : 0U);
  }

  CdResult("forward: four wheels same direction", all_same);

  if (suggest != 0U)
  {
    UART_Port_Printf(&uart1_port,
                     "[cd] hint: wheel direction mismatch, try invert_mask=0x%02X "
                     "(or check motor wiring)\r\n", (unsigned)suggest);
  }
}

/* 原地转向检查: 左侧/右侧符号相反 */
static void CdCheckTurn(const char *what, int8_t left_sign, int8_t right_sign)
{
  int32_t delta[CHASSIS_MOTOR_NUM];
  uint8_t left_ok = 1U;
  uint8_t right_ok = 1U;

  for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
  {
    delta[i] = AT8236_GetEncoderTotal(&s_board, i) - s_count_ref[i];
    UART_Port_Printf(&uart1_port, "[cd] %s M%u delta=%ld\r\n",
                     what, (unsigned)(i + 1U), (long)delta[i]);
  }

  for (uint8_t i = 0U; i < 2U; i++)
  {
    if ((CdSign(delta[i]) != left_sign) ||
        (CdMag(delta[i]) < (uint32_t)CD_MOVE_COUNTS))
    {
      left_ok = 0U;
    }
  }
  for (uint8_t i = 2U; i < 4U; i++)
  {
    if ((CdSign(delta[i]) != right_sign) ||
        (CdMag(delta[i]) < (uint32_t)CD_MOVE_COUNTS))
    {
      right_ok = 0U;
    }
  }

  CdResult("turn: left pair direction", left_ok);
  CdResult("turn: right pair direction", right_ok);

  if ((left_ok == 0U) || (right_ok == 0U))
  {
    uint8_t suggest = s_chassis.cfg.invert_mask;

    if (left_ok == 0U)
    {
      suggest ^= 0x03U; /* M1/M2 */
    }
    if (right_ok == 0U)
    {
      suggest ^= 0x0CU; /* M3/M4 */
    }
    UART_Port_Printf(&uart1_port,
                     "[cd] hint: try invert_mask=0x%02X, and check M1/M2 on left, "
                     "M3/M4 on right\r\n", (unsigned)suggest);
  }
}

/* ---- 状态入口 ----------------------------------------------------------- */
static void CdEnter(CdState_e state, uint32_t now)
{
  s_state = state;
  s_tick = now;

  switch (state)
  {
    case CD_ST_CFG_TYPE:
      CdCmd("config $mtype", AT8236_ConfigMotorType(&s_board, s_board.cfg.motor_type));
      break;

    case CD_ST_CFG_LINE:
      CdCmd("config $mline", AT8236_ConfigEncoderLines(&s_board, s_board.cfg.encoder_lines));
      break;

    case CD_ST_CFG_RATIO:
      CdCmd("config $mphase", AT8236_ConfigGearRatio(&s_board, s_board.cfg.gear_ratio));
      break;

    case CD_ST_CFG_DIA:
      CdCmd("config $wdiameter",
            AT8236_ConfigWheelDiameter(&s_board, s_board.cfg.wheel_diameter_mm));
      break;

    case CD_ST_CFG_DEAD:
      CdCmd("config $deadzone", AT8236_ConfigDeadzone(&s_board, s_board.cfg.deadzone));
      break;

    case CD_ST_CFG_VERIFY:
      break;

    case CD_ST_UP:
      s_total_ref = s_board.stats.total_frames;
      s_speed_ref = s_board.stats.speed_frames;
      CdCmd("upload total + speed", AT8236_SetUpload(&s_board, 1U, 0U, 1U));
      break;

    case CD_ST_FWD:
      Chassis_ResetOdometry(&s_chassis);
      CdSnapshotCounts();
      CdSnapshotOdom();
      CdCmd("chassis forward 150mm/s",
            Chassis_SetVelocity(&s_chassis, CD_FWD_MMPS, 0.0f));
      break;

    case CD_ST_STOP1:
    case CD_ST_STOP2:
    case CD_ST_STOP3:
    case CD_ST_STOP4:
      CdCmd("chassis stop", Chassis_Stop(&s_chassis));
      break;

    case CD_ST_TURN_L:
      Chassis_ResetOdometry(&s_chassis);
      CdSnapshotCounts();
      CdSnapshotOdom();
      CdCmd("chassis turn left 60dps",
            Chassis_SetVelocity(&s_chassis, 0.0f, CD_TURN_DPS));
      break;

    case CD_ST_TURN_R:
      Chassis_ResetOdometry(&s_chassis);
      CdSnapshotCounts();
      CdSnapshotOdom();
      CdCmd("chassis turn right 60dps",
            Chassis_SetVelocity(&s_chassis, 0.0f, -CD_TURN_DPS));
      break;

    case CD_ST_WATCHDOG:
      s_timeout_ref = s_chassis.stats.timeout_count;
      s_chassis.cfg.command_timeout_ms = CD_WD_TIMEOUT_MS;
      CdCmd("lease 400ms, speed 100mm/s",
            Chassis_SetVelocity(&s_chassis, 100.0f, 0.0f));
      break;

    case CD_ST_DONE:
      /* 前一步已经处于 HOLD, 这里只关上报, 避免与停车指令抢 DMA */
      (void)AT8236_SetUpload(&s_board, 0U, 0U, 0U);
      UART_Port_Printf(&uart1_port,
                       "[cd] done | checks=%lu fails=%lu | chassis task=%lu tx=%lu "
                       "busy=%lu timeout=%lu limit=%lu fblost=%lu hold=%lu\r\n",
                       (unsigned long)s_checks, (unsigned long)s_fails,
                       (unsigned long)s_chassis.stats.task_count,
                       (unsigned long)s_chassis.stats.cmd_sent,
                       (unsigned long)s_chassis.stats.cmd_busy,
                       (unsigned long)s_chassis.stats.timeout_count,
                       (unsigned long)s_chassis.stats.limit_count,
                       (unsigned long)s_chassis.stats.feedback_lost,
                       (unsigned long)s_chassis.stats.hold_count);
      UART_Port_Printf(&uart1_port,
                       "[cd] board rx=%lu ack=%lu total=%lu speed=%lu parse=%lu ovf=%lu "
                       "tx=%lu busy=%lu\r\n",
                       (unsigned long)s_board.stats.rx_frames,
                       (unsigned long)s_board.stats.ack_frames,
                       (unsigned long)s_board.stats.total_frames,
                       (unsigned long)s_board.stats.speed_frames,
                       (unsigned long)s_board.stats.parse_errors,
                       (unsigned long)s_board.stats.overflow,
                       (unsigned long)s_board.stats.tx_ok,
                       (unsigned long)s_board.stats.tx_busy);
      UART_Port_Printf(&uart1_port,
                       "[cd] odom x=%ldmm y=%ldmm yaw=%ld.%lddeg\r\n",
                       CdRound(s_chassis.odom.x_mm), CdRound(s_chassis.odom.y_mm),
                       CdRound(s_chassis.odom.yaw_deg),
                       (long)(CdMag((int32_t)CdRound(s_chassis.odom.yaw_deg * 10.0f)) % 10L));
      break;

    default:
      break;
  }
}

/* ---- 初始化 ------------------------------------------------------------- */
void Chassis_Debug_Init(void)
{
  AT8236_Config_t board_cfg;
  Chassis_Config_t chassis_cfg;
  uint32_t now = HAL_GetTick();
  uint8_t ret;

  s_checks = 0U;
  s_fails = 0U;

  AT8236_DefaultConfig(&board_cfg, CD_UART, CD_NAME);
  board_cfg.wheel_diameter_mm = CD_WHEEL_MM;

  ret = AT8236_Init(&s_board, &board_cfg);
  if (ret != (uint8_t)AT8236_OK)
  {
    s_state = CD_ST_INIT_FAIL;
    UART_Port_Printf(&uart1_port,
                     "[cd] AT8236 Init FAIL ret=%u uart_init_ret=%u\r\n",
                     (unsigned)ret, (unsigned)s_board.uart_init_ret);
    UART_Port_Printf(&uart1_port,
                     "[cd] enable USART2 RX/TX DMA in CubeMX (DMA1 Ch6/Ch7), "
                     "wire PA2->board RX2, PA3<-board TX2, GND common\r\n");
    return;
  }

  Chassis_DefaultConfig(&chassis_cfg);
  chassis_cfg.track_width_mm      = CD_TRACK_MM;
  chassis_cfg.wheel_diameter_mm   = CD_WHEEL_MM;
  chassis_cfg.counts_per_rev      = CD_COUNTS_REV;
  chassis_cfg.invert_mask         = CD_INVERT_MASK;

  if (Chassis_Init(&s_chassis, &chassis_cfg,
                   Chassis_GetAT8236Ops(), &s_board) != (uint8_t)CHASSIS_OK)
  {
    s_state = CD_ST_INIT_FAIL;
    UART_Port_Printf(&uart1_port, "[cd] Chassis Init FAIL\r\n");
    return;
  }

  UART_Port_Printf(&uart1_port,
                   "[cd] test start: chassis WILL move, lift wheels off the ground!\r\n");
  UART_Port_Printf(&uart1_port,
                   "[cd] track=%ldmm wheel=%ldmm cpr=%lu invert=0x%02X "
                   "maxV=%ldmm/s maxW=%lddps\r\n",
                   CdRound(CD_TRACK_MM), CdRound(CD_WHEEL_MM),
                   (unsigned long)CD_COUNTS_REV, (unsigned)CD_INVERT_MASK,
                   CdRound(s_chassis.cfg.max_linear_mmps),
                   CdRound(s_chassis.cfg.max_angular_dps));
  UART_Port_Printf(&uart1_port,
                   "[cd] measured params must match the real car; wrong wheel/track "
                   "only scales odometry, wrong invert/direction fails the checks\r\n");

  s_ack_ref = s_board.stats.ack_frames;
  s_status_tick = now;
  s_ctrl_tick = now;
  CdEnter(CD_ST_CFG_TYPE, now);
}

/* ---- 周期运行 ----------------------------------------------------------- */
void Chassis_Debug_Run(uint32_t now)
{
  if ((s_state == CD_ST_INIT_FAIL) || (s_state == CD_ST_DONE))
  {
    return;
  }

  /* 底盘控制环: 10ms 固定周期(状态机与打印都不阻塞) */
  if ((now - s_ctrl_tick) >= CD_CTRL_PERIOD_MS)
  {
    s_ctrl_tick = now;
    Chassis_Task(&s_chassis, now);
  }

  /* 实时状态 */
  if ((now - s_status_tick) >= CD_STATUS_MS)
  {
    Chassis_Odometry_t od;

    s_status_tick = now;
    Chassis_GetOdometry(&s_chassis, &od);
    UART_Port_Printf(&uart1_port,
                     "[cd] status | mode=%u tgt=%ld,%ld cmd=%ld,%ld meas=%ld,%ld | "
                     "whl=%ld,%ld,%ld,%ld | x=%ld y=%ld yaw10=%ld | "
                     "tx=%lu busy=%lu tmo=%lu on=%u\r\n",
                     (unsigned)Chassis_GetMode(&s_chassis),
                     CdRound(Chassis_GetTargetLinearMmps(&s_chassis)),
                     CdRound(Chassis_GetTargetAngularDps(&s_chassis)),
                     CdRound(s_chassis.cmd_v_mmps), CdRound(s_chassis.cmd_w_dps),
                     CdRound(Chassis_GetLinearMmps(&s_chassis)),
                     CdRound(Chassis_GetAngularDps(&s_chassis)),
                     CdRound(Chassis_GetWheelCommand(&s_chassis, 0U)),
                     CdRound(Chassis_GetWheelCommand(&s_chassis, 1U)),
                     CdRound(Chassis_GetWheelCommand(&s_chassis, 2U)),
                     CdRound(Chassis_GetWheelCommand(&s_chassis, 3U)),
                     CdRound(od.x_mm), CdRound(od.y_mm), CdRound(od.yaw_deg * 10.0f),
                     (unsigned long)s_chassis.stats.cmd_sent,
                     (unsigned long)s_chassis.stats.cmd_busy,
                     (unsigned long)s_chassis.stats.timeout_count,
                     (unsigned)Chassis_IsOnline(&s_chassis, now));
  }

  switch (s_state)
  {
    case CD_ST_CFG_TYPE:
      if ((now - s_tick) >= CD_CFG_GAP_MS)
      {
        CdResult("config mtype accepted", s_last_ret == (uint8_t)AT8236_OK);
        CdEnter(CD_ST_CFG_LINE, now);
      }
      break;

    case CD_ST_CFG_LINE:
      if ((now - s_tick) >= CD_CFG_GAP_MS)
      {
        CdResult("config mline accepted", s_last_ret == (uint8_t)AT8236_OK);
        CdEnter(CD_ST_CFG_RATIO, now);
      }
      break;

    case CD_ST_CFG_RATIO:
      if ((now - s_tick) >= CD_CFG_GAP_MS)
      {
        CdResult("config mphase accepted", s_last_ret == (uint8_t)AT8236_OK);
        CdEnter(CD_ST_CFG_DIA, now);
      }
      break;

    case CD_ST_CFG_DIA:
      if ((now - s_tick) >= CD_CFG_GAP_MS)
      {
        CdResult("config wdiameter accepted", s_last_ret == (uint8_t)AT8236_OK);
        CdEnter(CD_ST_CFG_DEAD, now);
      }
      break;

    case CD_ST_CFG_DEAD:
      if ((now - s_tick) >= CD_CFG_GAP_MS)
      {
        CdResult("config deadzone accepted", s_last_ret == (uint8_t)AT8236_OK);
        CdEnter(CD_ST_CFG_VERIFY, now);
      }
      break;

    case CD_ST_CFG_VERIFY:
      if ((now - s_tick) >= CD_ACK_WAIT_MS)
      {
        uint32_t acks = s_board.stats.ack_frames - s_ack_ref;

        UART_Port_Printf(&uart1_port, "[cd] config acks=%lu last_resp=\"%s\"\r\n",
                         (unsigned long)acks, AT8236_GetLastResponse(&s_board));
        CdResult("5 config commands echoed OK", (acks >= 5U) ? 1U : 0U);
        CdEnter(CD_ST_UP, now);
      }
      break;

    case CD_ST_UP:
      if ((now - s_tick) >= CD_UP_WAIT_MS)
      {
        CdResult("total encoder frames", (s_board.stats.total_frames > s_total_ref) ? 1U : 0U);
        CdResult("speed frames", (s_board.stats.speed_frames > s_speed_ref) ? 1U : 0U);
        CdResult("chassis feedback online", Chassis_IsOnline(&s_chassis, now));
        CdEnter(CD_ST_FWD, now);
      }
      break;

    case CD_ST_FWD:
      if ((now - s_tick) >= CD_FWD_MS)
      {
        Chassis_Odometry_t od;
        float dx;

        CdCheckForward("forward wheel");
        Chassis_GetOdometry(&s_chassis, &od);
        dx = od.x_mm - s_odom_ref.x_mm;
        UART_Port_Printf(&uart1_port, "[cd] forward odom dx=%ldmm v=%ldmm/s\r\n",
                         CdRound(dx), CdRound(Chassis_GetLinearMmps(&s_chassis)));
        CdResult("forward odometry +x", (dx > 50.0f) ? 1U : 0U);
        CdResult("forward measured speed",
                 ((Chassis_GetLinearMmps(&s_chassis) >= CD_SPEED_MIN) &&
                  (Chassis_GetLinearMmps(&s_chassis) <= CD_SPEED_MAX)) ? 1U : 0U);
        CdEnter(CD_ST_STOP1, now);
      }
      break;

    case CD_ST_STOP1:
      if ((now - s_tick) >= CD_STOP_MS)
      {
        CdResult("forward stop -> HOLD",
                 (Chassis_GetMode(&s_chassis) == CHASSIS_MODE_HOLD) ? 1U : 0U);
        CdResult("forward stop speed ~0",
                 (CdAbsF(Chassis_GetLinearMmps(&s_chassis)) <= CD_STOP_TOL) ? 1U : 0U);
        CdEnter(CD_ST_TURN_L, now);
      }
      break;

    case CD_ST_TURN_L:
      if ((now - s_tick) >= CD_TURN_MS)
      {
        Chassis_Odometry_t od;
        float dyaw;

        CdCheckTurn("turn-left wheel", -1, 1);
        Chassis_GetOdometry(&s_chassis, &od);
        dyaw = od.yaw_rad - s_odom_ref.yaw_rad;
        UART_Port_Printf(&uart1_port, "[cd] turn-left odom dyaw10=%ld w=%lddps\r\n",
                         CdRound(dyaw * CD_RAD2DEG * 10.0f),
                         CdRound(Chassis_GetAngularDps(&s_chassis)));
        CdResult("turn-left yaw increases", (dyaw > CD_YAW_MIN) ? 1U : 0U);
        CdResult("turn-left measured angular",
                 ((Chassis_GetAngularDps(&s_chassis) >= CD_WS_MIN) &&
                  (Chassis_GetAngularDps(&s_chassis) <= 120.0f)) ? 1U : 0U);
        CdEnter(CD_ST_STOP2, now);
      }
      break;

    case CD_ST_STOP2:
      if ((now - s_tick) >= CD_STOP_MS)
      {
        CdResult("turn-left stop -> HOLD",
                 (Chassis_GetMode(&s_chassis) == CHASSIS_MODE_HOLD) ? 1U : 0U);
        CdResult("turn-left stop speed ~0",
                 (CdAbsF(Chassis_GetAngularDps(&s_chassis)) <= CD_STOP_W_TOL) ? 1U : 0U);
        CdEnter(CD_ST_TURN_R, now);
      }
      break;

    case CD_ST_TURN_R:
      if ((now - s_tick) >= CD_TURN_MS)
      {
        Chassis_Odometry_t od;
        float dyaw;

        CdCheckTurn("turn-right wheel", 1, -1);
        Chassis_GetOdometry(&s_chassis, &od);
        dyaw = od.yaw_rad - s_odom_ref.yaw_rad;
        UART_Port_Printf(&uart1_port, "[cd] turn-right odom dyaw10=%ld w=%lddps\r\n",
                         CdRound(dyaw * CD_RAD2DEG * 10.0f),
                         CdRound(Chassis_GetAngularDps(&s_chassis)));
        CdResult("turn-right yaw decreases", (dyaw < -CD_YAW_MIN) ? 1U : 0U);
        CdResult("turn-right measured angular",
                 ((Chassis_GetAngularDps(&s_chassis) <= -CD_WS_MIN) &&
                  (Chassis_GetAngularDps(&s_chassis) >= -120.0f)) ? 1U : 0U);
        CdEnter(CD_ST_STOP3, now);
      }
      break;

    case CD_ST_STOP3:
      if ((now - s_tick) >= CD_STOP_MS)
      {
        CdResult("turn-right stop -> HOLD",
                 (Chassis_GetMode(&s_chassis) == CHASSIS_MODE_HOLD) ? 1U : 0U);
        CdResult("turn-right stop speed ~0",
                 (CdAbsF(Chassis_GetAngularDps(&s_chassis)) <= CD_STOP_W_TOL) ? 1U : 0U);
        CdEnter(CD_ST_WATCHDOG, now);
      }
      break;

    case CD_ST_WATCHDOG:
      if ((now - s_tick) >= CD_WD_TEST_MS)
      {
        CdResult("lease timeout fired",
                 (s_chassis.stats.timeout_count > s_timeout_ref) ? 1U : 0U);
        CdResult("lease timeout -> HOLD",
                 (Chassis_GetMode(&s_chassis) == CHASSIS_MODE_HOLD) ? 1U : 0U);
        s_chassis.cfg.command_timeout_ms = 0U;
        CdEnter(CD_ST_STOP4, now);
      }
      break;

    case CD_ST_STOP4:
      if ((now - s_tick) >= CD_STOP_MS)
      {
        CdResult("final state HOLD", (Chassis_GetMode(&s_chassis) == CHASSIS_MODE_HOLD) ? 1U : 0U);
        CdEnter(CD_ST_DONE, now);
      }
      break;

    default:
      break;
  }
}

#endif /* chassis_debug == 1 */
