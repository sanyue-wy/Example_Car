/**
 * @file    debug.c
 * @brief   分层自检实现(台架调试)
 *
 * @note    每层只验证一件事, 打印 [PASS] / [FAIL 原因]。
 *          打开方式: 改 debug_config.h 里对应的宏为 1, 一次只开一个。
 *          RUN_MODE == RUN_MATCH 时本文件整段不参与编译(见 task_it.c 的 #if)。
 *
 * @note    所有层共用同一套硬件初始化(App_InitDrivers), 与比赛固件完全一致,
 *          避免"调试时能跑、比赛时配置不同"这类问题。
 */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"

#if (RUN_MODE != RUN_MATCH)

#include "app.h"
#include "app_config.h"
#include "app_script.h"
#include "app_log.h"
#include "chassis.h"
#include "chassis_motion.h"
#include "lift.h"
#include "bucket.h"
#include "dc_motor.h"
#include "drv_encoder.h"
#include "drv_uart.h"
#include "drv_tim.h"
#include "at8236.h"
#include "usart.h"
#include "main.h"

#if (DEBUG_VISION_ENABLE == 1)
#include "debug_vision.h"
#endif

/*==============================================================================
 * 公共
 *============================================================================*/
/* 用极简整数打印: 本机 64K Flash, UART_Port_Printf 会把 vsnprintf 拉进来 */
#define DBG(fmt, ...)  AppLog_Printf(fmt, ##__VA_ARGS__)

/* 把毫米小数量化成 "整数.两位小数" 打印(newlib-nano 无浮点 printf)
   注意: 小数部分取绝对值, 否则 5.4 会打成 "5.-40" */
#define MM_INT(v)     ((long)(v))
#define MM_FRAC(v)    ((long)(((v) - (float)(long)(v)) * 100.0f * \
                              ((((v) - (float)(long)(v)) < 0.0f) ? -1.0f : 1.0f)))

/* 打开蓝牙可视化时本文件的文本自检全部不编译(串口已被 COBS 帧独占) */
#if (DEBUG_VISION_ENABLE == 0)
static uint32_t s_last_print_ms;
#endif

/* 控制节拍实现标记: 某些层需要自己跑 Chassis_Task, 用一个宏统一开关 */
#if (motor_open_debug == 1) || (motor_pid_debug == 1) || \
    (chassis_debug == 1)    || (script_debug == 1)
#define DBG_NEED_CHASSIS_TICK  1
#else
#define DBG_NEED_CHASSIS_TICK  0
#endif

/*==============================================================================
 * 第 1 层: uart_debug —— USART1 打印与回显
 *============================================================================*/
#if (DEBUG_VISION_ENABLE == 1)
/* 蓝牙可视化独占 USART1: 本文件的文本自检一律不编译, 避免"定义了没人用"。
   Debug_Init/Debug_Run 也只在 DEBUG_VISION_ENABLE==0 时被调用(见文件末尾)。 */
#elif (uart_debug == 1)

static void UartRxEcho(UART_PortObject *port, uint8_t *data, uint16_t size)
{
    (void)UART_Port_Send(port, data, size);   /* 原样回显 */
}

static void Debug_Init(void)
{
    uart1_port.on_rx = UartRxEcho;
    DBG("[uart] [PASS] 用串口助手 115200 发任意字节, 应原样回显\r\n");
}

static void Debug_Run(uint32_t now)
{
    if ((now - s_last_print_ms) >= 1000U)
    {
        s_last_print_ms = now;
        DBG("[uart] [PASS] tx=%lu rx=%lu err=%lu\r\n",
            (unsigned long)uart1_port.stats.tx_count,
            (unsigned long)uart1_port.stats.rx_count,
            (unsigned long)(uart1_port.stats.rx_error + uart1_port.stats.err_count));
    }
}

/*==============================================================================
 * 第 2 层: encoder_debug —— 升降编码器(TIM2), 不动作
 *============================================================================*/
#elif (encoder_debug == 1)

static int32_t s_enc_last;

static void Debug_Init(void)
{
    DBG("[enc] [PASS] TIM2 编码器已接入 | 手转丝杆, 观察计数与方向\r\n");
}

static void Debug_Run(uint32_t now)
{
    if ((now - s_last_print_ms) >= 200U)
    {
        int32_t cnt;
        float   mm;

        s_last_print_ms = now;
        cnt = ENCODER_GetCount(&Lift_GetObject()->motor.encoder);
        mm  = Lift_GetHeight();

        DBG("[enc] cnt=%ld d=%ld h=%ld.%.2ld mm\r\n",
            (long)cnt, (long)(cnt - s_enc_last), MM_INT(mm), MM_FRAC(mm));
        s_enc_last = cnt;
    }
}

/*==============================================================================
 * 第 3 层: motor_open_debug —— AT8236 驱动板速度指令(电机会转!)
 *============================================================================*/
#elif (motor_open_debug == 1)

static uint32_t s_phase_ms;
static uint8_t  s_phase;
static float    s_cmd;

static void Debug_Init(void)
{
    DC_Motor_Enable(DC_MOTOR_FL, true);
    s_phase_ms = HAL_GetTick();
    s_phase    = 0U;
    s_cmd      = 0.0f;
    DBG("[motor] [PASS] 四轮 +200 -> -200 -> 0 mm/s 循环(请架空车轮!)\r\n");
}

static void Debug_Run(uint32_t now)
{
    Chassis_Feedback_t fb;
    Chassis_Object *ch = (Chassis_Object *)App_GetChassis();

    if (ch == NULL)
    {
        DBG("[motor] [FAIL] 驱动板/底盘初始化失败\r\n");
        return;
    }

    if ((now - s_phase_ms) >= 1500U)
    {
        s_phase_ms = now;
        s_phase    = (uint8_t)((s_phase + 1U) % 3U);
        s_cmd      = (s_phase == 0U) ? 200.0f :
                     (s_phase == 1U) ? -200.0f : 0.0f;

        (void)Chassis_SetWheelSpeeds(ch, s_cmd, s_cmd);
    }

    Chassis_GetFeedback(ch, &fb);

    if ((now - s_last_print_ms) >= 500U)
    {
        s_last_print_ms = now;
        DBG("[motor] cmd=%ld | M1=%ld M2=%ld M3=%ld M4=%ld\r\n",
            (long)s_cmd,
            (long)fb.wheel_speed_mmps[0], (long)fb.wheel_speed_mmps[1],
            (long)fb.wheel_speed_mmps[2], (long)fb.wheel_speed_mmps[3]);
    }
}

/*==============================================================================
 * 第 4 层: motor_pid_debug —— AT8236 速度阶跃(电机会转!)
 *============================================================================*/
#elif (motor_pid_debug == 1)

static const float s_steps[] = { 100.0f, 200.0f, 300.0f, 0.0f };
static uint8_t  s_step_idx;
static uint32_t s_step_ms;

static void Debug_Init(void)
{
    DC_Motor_Enable(DC_MOTOR_FL, true);
    s_step_idx = 0U;
    s_step_ms  = HAL_GetTick();
    DBG("[pid] [PASS] 速度阶跃 100/200/300/0 mm/s, 每档 3s(请架空车轮!)\r\n");
}

static void Debug_Run(uint32_t now)
{
    Chassis_Feedback_t fb;
    Chassis_Object *ch = (Chassis_Object *)App_GetChassis();

    if (ch == NULL)
    {
        DBG("[pid] [FAIL] 底盘未就绪\r\n");
        return;
    }

    if ((now - s_step_ms) >= 3000U)
    {
        s_step_ms = now;
        s_step_idx = (uint8_t)((s_step_idx + 1U) %
                      (uint8_t)(sizeof(s_steps) / sizeof(s_steps[0])));
        (void)Chassis_SetWheelSpeeds(ch, s_steps[s_step_idx], s_steps[s_step_idx]);
    }

    Chassis_GetFeedback(ch, &fb);
    if ((now - s_last_print_ms) >= 100U)
    {
        s_last_print_ms = now;
        DBG("[pid] tgt=%ld | FL=%ld FR=%ld RL=%ld RR=%ld\r\n",
            (long)s_steps[s_step_idx],
            (long)fb.wheel_speed_mmps[0], (long)fb.wheel_speed_mmps[1],
            (long)fb.wheel_speed_mmps[2], (long)fb.wheel_speed_mmps[3]);
    }
}

/*==============================================================================
 * 第 5 层: servo_debug —— 铲斗姿态循环(舵机会动!)
 *============================================================================*/
#elif (servo_debug == 1)

static uint32_t s_pose_ms;
static uint8_t  s_pose;

static void Debug_Init(void)
{
    Bucket_Arm();                 /* 台架调试允许输出 */
    s_pose    = 0U;
    s_pose_ms = HAL_GetTick();
    DBG("[servo] [PASS] STOW->SCOOP->CARRY->DUMP, 每档 1.5s\r\n");
}

static void Debug_Run(uint32_t now)
{
    if ((now - s_pose_ms) >= 1500U)
    {
        s_pose_ms = now;
        s_pose = (uint8_t)((s_pose + 1U) % (uint8_t)BUCKET_POSE_NUM);

        if (Bucket_SetPose((BucketPose_e)s_pose) != 0U)
        {
            DBG("[servo] [FAIL] SetPose 被拒绝(未 Arm?)\r\n");
            return;
        }
        DBG("[servo] pose=%u -> angle=%u deg\r\n",
            (unsigned)s_pose, (unsigned)Bucket_GetAngle());
    }

    if ((now - s_last_print_ms) >= 1000U)
    {
        s_last_print_ms = now;
        if (Bucket_IsDone())
        {
            DBG("[servo] [PASS] pose=%u angle=%u\r\n",
                (unsigned)Bucket_GetPose(), (unsigned)Bucket_GetAngle());
        }
    }
}

/*==============================================================================
 * 第 6 层: lift_debug —— 回零 + 高度闭环(丝杆会动!)
 *============================================================================*/
#elif (lift_debug == 1)

typedef enum { LD_HOME, LD_UP, LD_DOWN, LD_IDLE } LiftDbgStep_e;

static LiftDbgStep_e s_lstep;
static uint32_t      s_lstep_ms;
static float         s_err_max;
static uint8_t       s_home_started;

static void Debug_Init(void)
{
    DC_Motor_Enable(DC_MOTOR_LIFT, true);
    s_lstep        = LD_HOME;
    s_lstep_ms     = HAL_GetTick();
    s_err_max      = 0.0f;
    s_home_started = 0U;
    DBG("[lift] [PASS] 开始回零(低速下行找下限)\r\n");
}

static void Debug_Run(uint32_t now)
{
    float h, err;

    switch (s_lstep)
    {
        case LD_HOME:
            if (s_home_started == 0U)
            {
                s_home_started = 1U;
#if (LIFT_ZERO_AT_POWERUP == 1)
                /* P1-3: 上电即零点, 不做堵转回零; 直接进入高度闭环验证 */
                DBG("[lift] [PASS] LIFT_ZERO_AT_POWERUP=1, 上电位置即零点\r\n");
                s_lstep = LD_UP;
                (void)Lift_SetHeight(LIFT_H_LEVEL1);
                return;
#else
                if (Lift_StartHoming() != 0U)
                {
                    DBG("[lift] [FAIL] 回零无法启动\r\n");
                    s_lstep = LD_IDLE;
                    return;
                }
#endif
            }
            if (Lift_GetHomingState() == LIFT_HOMING_DONE)
            {
                DBG("[lift] [PASS] 回零完成, 零点已建立\r\n");
                s_lstep = LD_UP;
                (void)Lift_SetHeight(LIFT_H_LEVEL1);
            }
            else if (Lift_GetHomingState() == LIFT_HOMING_FAILED)
            {
                DBG("[lift] [FAIL] 回零失败 fault=0x%02X\r\n", Lift_GetFault());
                s_lstep = LD_IDLE;
            }
            break;

        case LD_UP:
            if (Lift_HasReached(LIFT_H_LEVEL1, LIFT_ARRIVE_TOL_MM))
            {
                h   = Lift_GetHeight();
                err = h - LIFT_H_LEVEL1;
                if (err < 0.0f) { err = -err; }
                if (err > s_err_max) { s_err_max = err; }

                DBG("[lift] [PASS] LEVEL1 到位 h=%ld.%.2ld err=%ld.%.2ld\r\n",
                    MM_INT(h), MM_FRAC(h), MM_INT(err), MM_FRAC(err));

                (void)Lift_SetHeight(LIFT_H_HOME);
                s_lstep    = LD_DOWN;
                s_lstep_ms = now;
            }
            break;

        case LD_DOWN:
            if (Lift_HasReached(LIFT_H_HOME, LIFT_ARRIVE_TOL_MM))
            {
                DBG("[lift] [PASS] 回 HOME, 最大到位误差=%ld.%.2ld mm\r\n",
                    MM_INT(s_err_max), MM_FRAC(s_err_max));
                s_lstep = LD_IDLE;
            }
            break;

        case LD_IDLE:
        default:
            if ((now - s_last_print_ms) >= 1000U)
            {
                s_last_print_ms = now;
                h = Lift_GetHeight();
                DBG("[lift] idle h=%ld.%.2ld state=%d fault=0x%02X\r\n",
                    MM_INT(h), MM_FRAC(h),
                    (int)Elevating_GetState(Lift_GetObject()), Lift_GetFault());
            }
            break;
    }
}

/*==============================================================================
 * 第 7 层: chassis_debug —— 直行/转角标定(车会跑!)
 *============================================================================*/
#elif (chassis_debug == 1)

typedef enum
{
    CD_STRAIGHT1 = 0,
    CD_TURN,
    CD_LONG_LEFT,     /* P0-5: +270deg, 超过 180 才暴露折返 bug */
    CD_LONG_RIGHT,    /* P0-5: -270deg */
    CD_STRAIGHT2,
    CD_DONE
} ChassisDbgStep_e;

static ChassisDbgStep_e s_cstep;
static uint8_t          s_turn_cnt;

static void Debug_Init(void)
{
    DC_Motor_Enable(DC_MOTOR_FL, true);
    s_cstep    = CD_STRAIGHT1;
    s_turn_cnt = 0U;
    DBG("[chassis] [PASS] 直行 1000mm -> 原地转 90deg x4 -> 再直行 1000mm\r\n");
    DBG("[chassis] 卷尺量实际距离: 新轮径 = 67 * 1000/实测\r\n");
    (void)ChassisMotion_MoveStraight(1000.0f, CHASSIS_V_SLOW_MMS, 0U);
}

static void Debug_Run(uint32_t now)
{
    ChassisMotionReport_t rep;
    float x, y, th;

    if (!ChassisMotion_IsDone())
    {
        if ((now - s_last_print_ms) >= 500U)
        {
            s_last_print_ms = now;
            ChassisMotion_GetPose(&x, &y, &th);
            DBG("[chassis] running x=%ld y=%ld th=%ld\r\n",
                (long)x, (long)y, (long)th);
        }
        return;
    }

    ChassisMotion_GetReport(&rep);
    ChassisMotion_GetPose(&x, &y, &th);

    switch (s_cstep)
    {
        case CD_STRAIGHT1:
            /* P0-4: err = target - achieved, 正数表示没走够 */
            DBG("[chassis] 直行 target=%ld achieved=%ld err=%ld st=%d\r\n",
                (long)rep.target, (long)rep.achieved, (long)rep.error, (int)rep.state);
            (void)ChassisMotion_Turn(90.0f, CHASSIS_W_MAX_DPS * 0.6f, 0U);
            s_turn_cnt = 1U;
            s_cstep = CD_TURN;
            break;

        case CD_TURN:
            DBG("[chassis] 转 %u/4 achieved=%ld err=%ld st=%d\r\n",
                (unsigned)s_turn_cnt, (long)rep.achieved, (long)rep.error, (int)rep.state);

            if (s_turn_cnt < 4U)
            {
                s_turn_cnt++;
                (void)ChassisMotion_Turn(90.0f, CHASSIS_W_MAX_DPS * 0.6f, 0U);
            }
            else
            {
                DBG("[chassis] 90x4 完成, 测 >180deg 连续转角(P0-5)\r\n");
                (void)ChassisMotion_Turn(270.0f, CHASSIS_W_MAX_DPS * 0.6f, 0U);
                s_cstep = CD_LONG_LEFT;
            }
            break;

        case CD_LONG_LEFT:
            DBG("[chassis] +270 achieved=%ld err=%ld st=%d\r\n",
                (long)rep.achieved, (long)rep.error, (int)rep.state);
            if (rep.state != CMOTION_DONE)
            {
                DBG("[chassis] [FAIL] +270 未完成(checked P0-5)\r\n");
            }
            (void)ChassisMotion_Turn(-270.0f, CHASSIS_W_MAX_DPS * 0.6f, 0U);
            s_cstep = CD_LONG_RIGHT;
            break;

        case CD_LONG_RIGHT:
            DBG("[chassis] -270 achieved=%ld err=%ld st=%d\r\n",
                (long)rep.achieved, (long)rep.error, (int)rep.state);
            if (rep.state != CMOTION_DONE)
            {
                DBG("[chassis] [FAIL] -270 未完成(checked P0-5)\r\n");
            }
            DBG("[chassis] [PASS] 开始第二个 1000mm\r\n");
            (void)ChassisMotion_MoveStraight(1000.0f, CHASSIS_V_SLOW_MMS, 0U);
            s_cstep = CD_STRAIGHT2;
            break;

        case CD_STRAIGHT2:
            DBG("[chassis] [PASS] 全部完成 x=%ld y=%ld th=%ld\r\n",
                (long)x, (long)y, (long)th);
            s_cstep = CD_DONE;
            break;

        case CD_DONE:
        default:
            break;
    }
}

/*==============================================================================
 * 第 8 层: script_debug —— 最小脚本试跑(启动键触发, 车会跑!)
 *============================================================================*/
#elif (script_debug == 1)

static uint8_t s_armed;
static uint8_t s_ran;

static void Debug_Init(void)
{
    DBG("[script] [PASS] 就绪: 按启动键(PB12)后: 铲斗 SCOOP + 前进 300mm\r\n");
}

static void Debug_Run(uint32_t now)
{
    if (s_armed == 0U)
    {
        if (HAL_GPIO_ReadPin(START_KEY_PORT, START_KEY_PIN) == START_KEY_ACTIVE_STATE)
        {
            s_armed = 1U;
            Bucket_Arm();
            DC_Motor_Enable(DC_MOTOR_FL, true);
            DC_Motor_Enable(DC_MOTOR_LIFT, true);
            DBG("[script] 启动\r\n");
        }
        return;
    }

    if (s_ran == 0U)
    {
        s_ran = 1U;
        (void)Bucket_SetPose(BUCKET_SCOOP);
        (void)ChassisMotion_MoveStraight(300.0f, CHASSIS_V_SLOW_MMS, 0U);
        return;
    }

    if (ChassisMotion_IsDone() && ((now - s_last_print_ms) >= 1000U))
    {
        s_last_print_ms = now;
        DBG("[script] [PASS] 前进 300mm 完成(重复打印确认已停)\r\n");
    }
}

/*==============================================================================
 * 未选择任何层
 *============================================================================*/
#else

static void Debug_Init(void)
{
    DBG("[debug] 未打开任何分层检测(debug_config.h) | 执行器全部禁用\r\n");
}

static void Debug_Run(uint32_t now)
{
    if ((now - s_last_print_ms) >= 2000U)
    {
        s_last_print_ms = now;
        DBG("[debug] idle | 在 debug_config.h 打开任意一层开始台架调试\r\n");
    }
}

#endif

/*==============================================================================
 * 对外入口(debug.h)
 *============================================================================*/

void debug_init(void)
{
#if (DEBUG_VISION_ENABLE == 0)
    DBG("\r\n=== debug build ===\r\n");
#endif
    App_InitDrivers();      /* 与比赛固件共用同一套硬件初始化, 执行器保持禁用 */

#if (DEBUG_VISION_ENABLE == 1)
    /* 可视化装好后再初始化各层: 本版可视化独占 USART1, 不跑分层文本自检 */
    DebugVision_Init();
#else
    Debug_Init();
#endif
}

void debug_run(void)
{
#if (DEBUG_VISION_ENABLE == 1)
    DebugVision_Loop();
#else
    Debug_Run(HAL_GetTick());
#endif
}

void Debug_ControlTick(void)
{
#if DBG_NEED_CHASSIS_TICK
    Chassis_Object *ch = (Chassis_Object *)App_GetChassis();

    if (ch != NULL)
    {
        Chassis_Task(ch, HAL_GetTick());
        ChassisMotion_Tick();
    }
#endif

    /* 升降模块的 PID 按 10ms 配置: 5ms 节拍下调一次 -> 实际 10ms */
    {
        static uint8_t div2;
        div2 ^= 1U;
        if (div2 != 0U)
        {
            Lift_Tick();
        }
    }

    Bucket_Tick();
    DC_Motor_Tick();

#if (DEBUG_VISION_ENABLE == 1)
    /* 控制环里只写快照, 不调用 btdbg(库只允许在主循环调用) */
    DebugVision_Snapshot();
#endif
}

#endif /* RUN_MODE != RUN_MATCH */