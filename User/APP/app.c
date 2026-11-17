/**
 * @file    app.c
 * @brief   比赛主状态机 + 动作脚本执行器 + 安全监控
 *
 * @note    线程模型:
 *            - App_ControlTick()  : TIM4 1ms 节拍上分频 5ms, 中断上下文。
 *                                   只做采样/闭环/下发, 不做打印、不做动态内存。
 *            - App_Run()          : 主循环, 推进状态机与脚本, 负责打印。
 *            - 两者共享的只有"目标值"与"快照", 用简单的 32 位赋值交换,
 *              Cortex-M3 上单字对齐赋值是原子的。
 */

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include "app_script.h"
#include "debug.h"
#include "app_log.h"
#include "task_it.h"

#include "chassis.h"
#include "chassis_motion.h"
#include "at8236.h"
#include "dc_motor.h"
#include "lift.h"
#include "bucket.h"
#include "drv_uart.h"
#include "drv_tim.h"
#include "usart.h"
#include "main.h"

#include <stddef.h>
#include <stdarg.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/

#define APP_TICK_MS            5U       /* 控制周期 */
#define APP_TICK_DIV           5U       /* TIM4 1ms -> 5ms */

/* 比赛程序最多包含的段数(COLLECT/TRAVEL/PLACE_Lx/RETREAT 共 4 段, 留余量) */
#define AppScriptProgramSlots   (8U)

/* Private types -------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/**
  * @brief  收尾阶段(APP_FINISH 与 MATCH 下的 APP_FAULT 共用)
  */
typedef enum
{
    FIN_PARK_CHASSIS = 0,  /* 底盘减速到 0 */
    FIN_PARK_LIFT,         /* 升降降回 HOME(可选) */
    FIN_WAIT_STOP,         /* 等所有执行器确实停稳 */
    FIN_DISABLE,           /* 禁用电机(舵机保持) */
    FIN_SETTLE,            /* 等 FINISH_SETTLE_MS */
    FIN_DONE
} AppFinishPhase_e;

typedef struct
{
    AppState_e state;
    uint8_t    fault;

    uint32_t   state_ms;        /* 进入当前状态的时间 */
    uint32_t   run_start_ms;    /* 比赛开始时间(用于 110s 判定) */

    /* 启动按键 */
    uint8_t    key_stable;      /* 消抖后的电平(1=按下) */
    uint8_t    key_last_raw;
    uint32_t   key_change_ms;
    uint8_t    start_consumed;  /* 已经启动过, 二次按下无效 */

    /* 状态内的一次性动作标记(放在上下文里, App_Init 的 memset 能一并清掉) */
    uint8_t    finish_done;
    uint8_t    estop_done;
    uint8_t    homing_wait;     /* 启动后正在等升降回零完成 */
    uint8_t    finish_phase;    /* AppFinishPhase_e */
    uint32_t   phase_ms;        /* 收尾分阶段计时 */
    uint8_t    geofence_warned; /* P1-2: 越界已记录(只记录模式下防刷屏) */

    /* 脚本执行器 */
    uint8_t    prg[AppScriptProgramSlots];
    uint8_t    prg_len;
    uint8_t    prg_idx;
    const ScriptSegDef_t *seg;
    uint16_t   step_idx;
    uint8_t    step_started;
    uint32_t   step_ms;

    /* 飞行记录: 每步耗时(最多记录 24 步, 超出只统计总数) */
    uint16_t   step_log_cnt;
    uint16_t   step_log_ms[24];
} AppContext_t;

/* Private variables ---------------------------------------------------------*/

static AppContext_t s_app;

/* 底层对象(必须静态分配) */
static AT8236_Object      s_board;
static Chassis_Object     s_chassis;
static uint8_t            s_chassis_ok;

/* 初始化期硬件缺失标志: CubeMX 未配置启动键/LED 引脚时置 1 */
static uint8_t            s_hw_missing;

/*==============================================================================
 * 小工具
 *============================================================================*/

static void App_LedWrite(uint8_t on)
{
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN,
                      on ? STATUS_LED_ON_STATE
                         : ((STATUS_LED_ON_STATE == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

static void App_Log(const char *fmt, ...)
{
#if (DEBUG_VISION_ENABLE == 0)
    /* 用极简打印: 64K Flash 上不能把 vsnprintf 拉进来(见 app_log.h)。
       这里把 va_list 原样转交给 AppLog_Printf, 只做一次格式化。 */
    va_list ap;

    va_start(ap, fmt);
    AppLog_VPrintf(fmt, ap);
    va_end(ap);
#else
    (void)fmt;
#endif
}

static void App_SetState(AppState_e st)
{
    s_app.state    = st;
    s_app.state_ms = HAL_GetTick();

    /* P1-1: 进入 FAULT 时把收尾阶段机复位到"等停稳",
       避免沿用 APP_FINISH 遗留的阶段(比如已经 FIN_DONE 就跳过禁用) */
    if (st == APP_FAULT)
    {
        s_app.finish_phase = FIN_WAIT_STOP;
        s_app.phase_ms     = s_app.state_ms;
    }
}

/*==============================================================================
 * 启动按键(单键、单次、消抖)
 *============================================================================*/

static bool App_StartKeyPressed(void)
{
#if (APP_START_KEY_FALLBACK == 1)
    /* 调试兜底: USART1 收到过任意字节即视为"按键"
       —— 只能用于 RUN_DEBUG, 比赛模式已在 app.h 里 #error */
    static uint32_t last_rx;

    if (uart1_port.stats.rx_count != last_rx)
    {
        last_rx = uart1_port.stats.rx_count;
        return true;
    }
    return false;
#else
    uint8_t raw;
    uint32_t now = HAL_GetTick();

    raw = (HAL_GPIO_ReadPin(START_KEY_PORT, START_KEY_PIN) == START_KEY_ACTIVE_STATE) ? 1U : 0U;

    if (raw != s_app.key_last_raw)
    {
        s_app.key_last_raw  = raw;
        s_app.key_change_ms = now;
        return false;
    }

    if ((now - s_app.key_change_ms) < START_KEY_DEBOUNCE_MS)
    {
        return false;
    }

    if (raw && !s_app.key_stable)
    {
        s_app.key_stable = 1U;
        return true;      /* 按下沿, 只报一次 */
    }
    if (!raw)
    {
        s_app.key_stable = 0U;
    }
    return false;
#endif
}

/*==============================================================================
 * 脚本执行器
 *============================================================================*/

/**
  * @brief  下发一步动作(首次进入该步时调用)
  * @retval 0 已下发; 1 下发失败(按超时处理)
  */
static uint8_t App_StartStep(const ActStep_t *st)
{
    float speed;

    switch (st->type)
    {
        case ACT_MOVE:
            speed = (st->p2 == 0) ? CHASSIS_V_SLOW_MMS : (float)st->p2;
            return (ChassisMotion_MoveStraight((float)st->p1, speed, st->timeout_ms) == 0U) ? 0U : 1U;

        case ACT_TURN:
            speed = (st->p2 == 0) ? CHASSIS_W_MAX_DPS : (float)st->p2;
            return (ChassisMotion_Turn((float)st->p1, speed, st->timeout_ms) == 0U) ? 0U : 1U;

        case ACT_WAIT:
            /* 等待步没有下发动作, 只靠 step_ms 计时 */
            return 0U;

        case ACT_LIFT:
            return Lift_SetHeight((float)st->p1);

        case ACT_BUCKET:
            if (st->p2 != 0)
            {
                return Bucket_SetPoseSlow((BucketPose_e)st->p1, (uint16_t)st->p2);
            }
            return Bucket_SetPose((BucketPose_e)st->p1);

        case ACT_CHECK_LIFT:
            return Lift_HasReached((float)st->p1, (float)st->p2) ? 0U : 1U;

        case ACT_END:
        default:
            return 0U;
    }
}

/**
  * @brief  判断一步是否完成
  */
static bool App_StepDone(const ActStep_t *st)
{
    switch (st->type)
    {
        case ACT_MOVE:
        case ACT_TURN:
            /* P0-3: 超时/故障也算"已停止", 但**不算成功** ——
               成功必须用 Succeeded(), 否则降级分支永远不会走 */
            return ChassisMotion_Succeeded();

        case ACT_WAIT:
            return ((HAL_GetTick() - s_app.step_ms) >= (uint32_t)st->p1);

        case ACT_LIFT:
            return Lift_HasReached((float)st->p1, LIFT_ARRIVE_TOL_MM);

        case ACT_BUCKET:
            return Bucket_IsDone();

        case ACT_CHECK_LIFT:
            return true;    /* 瞬时判断, 下发时已给出结果 */

        default:
            return true;
    }
}

/**
  * @brief  进入下一段(段内步号归零)
  */
static bool App_EnterSegment(uint8_t seg_idx)
{
    const ScriptSegDef_t *seg = AppScript_GetSegment(seg_idx);

    if (seg == NULL)
    {
        return false;
    }
    s_app.seg       = seg;
    s_app.step_idx  = 0U;
    s_app.step_started = 0U;
    return true;
}

/**
  * @brief  当前脚本执行一步
  * @retval true 脚本全部走完
  */
static bool App_RunScript(void)
{
    const ActStep_t *st;
    bool  timeout;
    uint32_t elapsed;

    if (s_app.seg == NULL)
    {
        return true;
    }

    /* 段走完 -> 进入程序里的下一段 */
    if (s_app.step_idx >= s_app.seg->count)
    {
        s_app.prg_idx++;
        if (s_app.prg_idx >= s_app.prg_len)
        {
            return true;
        }
        if (!App_EnterSegment(s_app.prg[s_app.prg_idx]))
        {
            return true;
        }
    }

    st = &s_app.seg->steps[s_app.step_idx];

    if (st->type == ACT_END)
    {
        s_app.step_idx = s_app.seg->count;   /* 段结束 */
        return false;
    }

    if (s_app.step_started == 0U)
    {
        s_app.step_started = 1U;
        s_app.step_ms      = HAL_GetTick();

        /* ACT_CHECK_LIFT 是"询问式"步骤: 返回非 0 不是命令下发失败,
           而是"没升到位", 必须走 on_timeout 的降级分支。
           统一按 timeout=true 处理, 下面的 switch 会把它跳到 FALLBACK。 */
        if (App_StartStep(st) != 0U)
        {
            App_Log("[app] step %u %s start FAILED/not-met\r\n",
                    (unsigned)s_app.step_idx, st->tag);
            timeout = true;
        }
        else
        {
            timeout = false;
        }
    }
    else
    {
        timeout = false;
    }

    elapsed = HAL_GetTick() - s_app.step_ms;

    if (!timeout && st->timeout_ms != 0U && elapsed >= st->timeout_ms)
    {
        timeout = true;
    }

    /* P0-3: 底盘动作在底层自己结束(超时/无反馈)时, 也按超时处理。
       脚本里多数 ACT_MOVE 的 timeout_ms=0, 只有底层计时器,
       若不在这里判失败, 超时会被当成功, ON_TO_FALLBACK/ON_TO_FINISH 永不执行。 */
    if (!timeout && ((st->type == ACT_MOVE) || (st->type == ACT_TURN)) &&
        ChassisMotion_IsDone() && !ChassisMotion_Succeeded())
    {
        timeout = true;
    }

    if (!timeout && App_StepDone(st))
    {
        /* 记录耗时(供赛后复盘) */
        if (s_app.step_log_cnt < 24U)
        {
            s_app.step_log_ms[s_app.step_log_cnt] = (uint16_t)elapsed;
        }
        s_app.step_log_cnt++;

        s_app.step_idx++;
        s_app.step_started = 0U;
        return false;
    }

    if (timeout)
    {
        App_Log("[app] step %u %s TIMEOUT (%lu ms)\r\n",
                (unsigned)s_app.step_idx, st->tag, (unsigned long)elapsed);

        /* 先保证这个动作停下来 */
        if ((st->type == ACT_MOVE) || (st->type == ACT_TURN))
        {
            ChassisMotion_Stop();
        }
        else if (st->type == ACT_LIFT)
        {
            Lift_Stop();
        }
        else if (st->type == ACT_BUCKET)
        {
            /* 舵机按时间到位, 超时说明没在动: 保持当前角度, 不额外动作 */
        }

        switch (st->on_timeout)
        {
            case ON_TO_FALLBACK:
            {
                uint8_t fb = AppScript_FallbackSegment();

                /* 在当前程序里找到降级段的位置; 找不到就直接收尾 */
                for (uint8_t i = s_app.prg_idx + 1U; i < s_app.prg_len; i++)
                {
                    if (s_app.prg[i] == fb)
                    {
                        s_app.prg_idx = i;
                        (void)App_EnterSegment(fb);
                        return false;
                    }
                }
                return true;
            }

            case ON_TO_NEXT:
                s_app.step_idx++;
                s_app.step_started = 0U;
                return false;

            case ON_TO_FINISH:
            default:
                return true;
        }
    }

    return false;
}

/*==============================================================================
 * 安全监控(每次 App_Run 都先检查)
 *============================================================================*/

static void App_SafetyCheck(void)
{
    float x, y, th;

    if (s_app.state != APP_RUN)
    {
        return;
    }

    /* 1. 时间硬上限(APP_RUN 入口处也会先判一次, 这里覆盖脚本执行期) */
    if ((HAL_GetTick() - s_app.run_start_ms) >= MATCH_SAFE_DEADLINE_MS)
    {
        s_app.fault = APP_FAULT_NONE;
        App_SetState(APP_FINISH);
        return;
    }

    /* 2. 堵转 */
    if (DC_Motor_AnyStall())
    {
        s_app.fault = APP_FAULT_STALL;
        App_SetState(APP_FAULT);
        return;
    }

    /* 3. 升降故障 */
    if (Lift_GetFault() != 0U)
    {
        s_app.fault = APP_FAULT_LIFT_RANGE;
        App_SetState(APP_FAULT);
        return;
    }

    /* 4. Geofence(P1-2): 处理方式由 GEOFENCE_ACTION 决定
          (比赛=只记录, 调试=急停), 见 app_config.h 的说明 */
    ChassisMotion_GetPose(&x, &y, &th);
    if (!ChassisMotion_PoseInField(x, y))
    {
#if (GEOFENCE_ACTION == 1)
        s_app.fault = APP_FAULT_GEOFENCE;
        App_SetState(APP_FAULT);
        return;
#else
        /* 只记录一次, 不打断比赛 */
        if (s_app.geofence_warned == 0U)
        {
            s_app.geofence_warned = 1U;
            App_Log("[app] geofence warn x=%ld y=%ld (仅记录)\r\n",
                    (long)x, (long)y);
        }
#endif
    }
}

/*==============================================================================
 * 收尾序列(APP_FINISH)
 *============================================================================*/

/**
  * @brief  收尾: 分阶段执行, 保证"先停稳 -> 再禁用 -> 最后点灯"(R25/R26)
  * @note   由 App_Run 的 APP_FINISH 分支每圈推进一个阶段。
  *         LED 只在所有电机确认停止并禁用之后才点亮。
  */
static void App_DoFinishSequence(void)
{
    switch ((AppFinishPhase_e)s_app.finish_phase)
    {
        case FIN_PARK_CHASSIS:
            ChassisMotion_Stop();
            (void)Chassis_Stop(&s_chassis);
            s_app.finish_phase = FIN_PARK_LIFT;
            break;

        case FIN_PARK_LIFT:
#if (FINISH_PARK_LIFT == 1)
            (void)Lift_SetHeight(LIFT_H_HOME);
#endif
            s_app.finish_phase = FIN_WAIT_STOP;
            s_app.phase_ms     = HAL_GetTick();
            break;

        case FIN_WAIT_STOP:
            /* 等车与升降都停稳; 升降回 HOME 需要时间, 最多等 3s */
            if (DC_Motor_AllStopped() ||
                ((HAL_GetTick() - s_app.phase_ms) >= 3000U))
            {
                s_app.finish_phase = FIN_DISABLE;
            }
            break;

        case FIN_DISABLE:
            /* 电机全部禁用、PWM=0 */
            DC_Motor_Enable(DC_MOTOR_FL, false);     /* 4 轮共用一个使能位 */
            DC_Motor_Enable(DC_MOTOR_LIFT, false);

            /* 舵机: 保持最后脉宽 or 断开 */
#if (FINISH_HOLD_SERVO == 0)
            Bucket_Disarm();
#endif

            App_Log("[app] finish | steps=%u\r\n", (unsigned)s_app.step_log_cnt);
            for (uint16_t i = 0; i < s_app.step_log_cnt && i < 24U; i++)
            {
                App_Log("  step %02u : %u ms\r\n", (unsigned)i, (unsigned)s_app.step_log_ms[i]);
            }

            s_app.finish_phase = FIN_SETTLE;
            s_app.phase_ms     = HAL_GetTick();
            break;

        case FIN_SETTLE:
            if ((HAL_GetTick() - s_app.phase_ms) >= FINISH_SETTLE_MS)
            {
                s_app.finish_phase = FIN_DONE;
            }
            break;

        case FIN_DONE:
        default:
            break;
    }
}

/**
  * @brief  故障收尾(P1-1, 仅比赛模式)
  * @note   与正常收尾共用同一个阶段机, 但跳过 FIN_PARK_LIFT:
  *         故障时升降可能已经不可信, 再命令它运动是二次风险。
  *         阶段: WAIT_STOP -> DISABLE -> SETTLE -> DONE(LED 常亮)
  */
#if (RUN_MODE == RUN_MATCH)
static void App_DoFaultFinish(void)
{
    switch ((AppFinishPhase_e)s_app.finish_phase)
    {
        case FIN_WAIT_STOP:
            if (DC_Motor_AllStopped() ||
                ((HAL_GetTick() - s_app.phase_ms) >= 3000U))
            {
                s_app.finish_phase = FIN_DISABLE;
            }
            break;

        case FIN_DISABLE:
            DC_Motor_Enable(DC_MOTOR_FL, false);
            DC_Motor_Enable(DC_MOTOR_LIFT, false);
#if (FINISH_HOLD_SERVO == 0)
            Bucket_Disarm();
#endif
            s_app.finish_phase = FIN_SETTLE;
            s_app.phase_ms     = HAL_GetTick();
            break;

        case FIN_SETTLE:
            if ((HAL_GetTick() - s_app.phase_ms) >= FINISH_SETTLE_MS)
            {
                s_app.finish_phase = FIN_DONE;
            }
            break;

        case FIN_DONE:
        default:
            break;
    }
}
#endif /* RUN_MODE == RUN_MATCH */

/*==============================================================================
 * 对外接口
 *============================================================================*/

/**
  * @brief  只做"驱动层 + 执行器"初始化, 不进入比赛状态机
  * @note   台架调试(debug_init)也调用它, 保证联调与比赛共用同一套硬件初始化
  */
void App_InitDrivers(void)
{
    AT8236_Config_t board_cfg;
    Chassis_Config_t ch_cfg;

    memset(&s_app, 0, sizeof(s_app));
    s_chassis_ok = 0U;

    /* ---- 1. 底层节拍与串口 ---- */
    Task_IT_Init();          /* TIM4 1ms 节拍 + UART/驱动滴答 */

    /* ---- 2. 驱动板 + 底盘 ---- */
    AT8236_DefaultConfig(&board_cfg, AT8236_UART_HANDLE, "AT8236");
    board_cfg.motor_type        = AT8236_MOTOR_TYPE;
    board_cfg.encoder_lines     = AT8236_ENCODER_LINES;
    board_cfg.gear_ratio        = AT8236_GEAR_RATIO;
    board_cfg.deadzone          = AT8236_DEADZONE;
    board_cfg.wheel_diameter_mm = WHEEL_DIAMETER_MM;

    if (AT8236_Init(&s_board, &board_cfg) == 0U)
    {
        Chassis_DefaultConfig(&ch_cfg);
        ch_cfg.track_width_mm     = TRACK_WIDTH_MM;
        ch_cfg.wheel_diameter_mm  = WHEEL_DIAMETER_MM;
        ch_cfg.counts_per_rev     = ENCODER_CPR;
        ch_cfg.max_linear_mmps    = CHASSIS_V_MAX_MMS;
        ch_cfg.max_angular_dps    = CHASSIS_W_MAX_DPS;
        ch_cfg.linear_accel_mmps2 = CHASSIS_ACC_MMS2;
        ch_cfg.angular_accel_dps2 = CHASSIS_ACC_DPS2;

        if (Chassis_Init(&s_chassis, &ch_cfg, Chassis_GetAT8236Ops(), &s_board) == 0U)
        {
            s_chassis_ok = 1U;
            ChassisMotion_Bind(&s_chassis);

            /* 打开驱动板上报: 累计编码器 + 速度 mm/s。
               必须显式下发 —— 驱动板重启后上报开关会被清掉(见 at8236.c 的
               AT8236_SetPid 注释), 不开则 Chassis 拿不到轮速/计数:
                 里程计恒为 0 -> 直行动作永远走不完;
                 堵转检测看到"有指令无反馈" -> 误判 FAULT。
               不开 delta($MTEP): 115200 上三类全开占用过高, 累计值已够用。 */
            (void)AT8236_SetUpload(&s_board, 1U, 0U, 1U);
        }
    }

    /* ---- 3. 升降 + 铲斗 + 执行器抽象 ---- */
    (void)Lift_Init();
    Bucket_Init();

    DC_Motor_Bind(&s_chassis, Lift_GetObject());
    DC_Motor_Init();

    /* ---- 4. 执行器一律禁用, 等启动键 ---- */
    DC_Motor_Enable(DC_MOTOR_FL, false);
    DC_Motor_Enable(DC_MOTOR_LIFT, false);

    /* ---- 5. 导航位姿 = 起始区 ---- */
    if (s_chassis_ok != 0U)
    {
        ChassisMotion_SetPose(START_POSE_X_MM, START_POSE_Y_MM, START_POSE_THETA_DEG);
    }

    /* ---- 6. 启动键/LED 引脚硬件检测 ----
       PB12 / PC13 需要在 CubeMX 中配置(见 README_APP.md §CubeMX 待补清单)。
       这里只读寄存器, 不改变任何引脚状态:
         a) PC13: 所属 GPIOA/A/B/D 时钟里, 本工程 MX_GPIO_Init 只开了 A/B/D,
            没开 C 就一定没配 PC13;
         b) PB12: 不能只看 IOPBEN(已因 PB0/PB1 打开), 要直接看 GPIOB->CRH 里
            PB12 的模式位。复位值 0b0100 = 浮空输入, 与"配成上拉输入"的 0b1000
            可区分。留成浮空是危险的: 悬空引脚读数随机, 可能凭空触发启动,
            因此判为缺失。
       探测结果只置标志, 不改变任何引脚状态。 */
#if (APP_START_KEY_FALLBACK == 0)
    {
        /* PB12 在 GPIOB->CRH 的 [19:16]: 期望 0b1000(上拉输入) 或
           0b1010(带中断的上拉输入); 其余(浮空/模拟/输出)都算没配置好 */
        uint32_t crh = (GPIOB->CRH >> 16) & 0xFU;

        if (((RCC->APB2ENR & RCC_APB2ENR_IOPCEN) == 0U) ||
            ((RCC->APB2ENR & RCC_APB2ENR_IOPBEN) == 0U) ||
            ((crh != 0x8U) && (crh != 0xAU)))
        {
            s_hw_missing = 1U;
        }
    }
#endif
}

void App_Init(void)
{
    App_InitDrivers();

    App_SetState(APP_READY);

#if (RUN_MODE == RUN_MATCH)
    if (s_hw_missing != 0U)
    {
        /* CubeMX 还没配 PB12/PC13: 按键与结束 LED 都不可用, 不满足 R25/R19,
           直接进故障态, 不上场。配置方法见 README_APP.md §CubeMX 待补清单。 */
        s_app.fault = APP_FAULT_SELF;
        App_Log("[app] HARDWARE MISSING: 请在 CubeMX 配置 PB12(启动键)/PC13(状态 LED)\r\n");
        App_SetState(APP_FAULT);
        return;
    }
    App_Log("[app] READY | wait start key\r\n");
#else
    App_Log("[app] READY(debug)\r\n");
#endif
}

/* 比赛开始时: 先回零再跑脚本。
   升降的高度全部是"相对回零零点"的, 不回零就往里塞高度值没有意义,
   所以启动后第一件事是启回零; 回零期间不跑脚本(见 APP_START_HOMING)。
   @note P1-3: LIFT_ZERO_AT_POWERUP==1 时上电即零点, 本函数不参与编译。 */
#if (LIFT_ZERO_AT_POWERUP == 0)
static uint8_t App_BeginHoming(void)
{
    if (Lift_StartHoming() != 0U)
    {
        return 1U;
    }
    s_app.phase_ms = HAL_GetTick();
    return 0U;
}
#endif

void App_Run(void)
{
    switch (s_app.state)
    {
        case APP_READY:
        {
            uint32_t now = HAL_GetTick();

            /* 就绪: LED 慢闪, 所有执行器不输出 */
            App_LedWrite((uint8_t)(((now - s_app.state_ms) / READY_LED_BLINK_MS) & 1U));

            if (!s_app.start_consumed && App_StartKeyPressed())
            {
                s_app.start_consumed = 1U;
                s_app.run_start_ms   = HAL_GetTick();

                /* 比赛开始: 打开执行器 */
                Bucket_Arm();
                DC_Motor_Enable(DC_MOTOR_FL, true);
                DC_Motor_Enable(DC_MOTOR_LIFT, true);

                s_app.prg_len = AppScript_GetProgram(s_app.prg, AppScriptProgramSlots);
                s_app.prg_idx = 0U;

                if ((s_app.prg_len == 0U) || !App_EnterSegment(s_app.prg[0]))
                {
                    s_app.fault = APP_FAULT_SELF;
                    App_SetState(APP_FAULT);
                }
                else
                {
                    /* P1-3: LIFT_ZERO_AT_POWERUP==1 时上电位置就是零点
                       (设置阶段由队员把平台放到最底), 启动后不再回零,
                       省掉最多 10s 的比赛时间。仅当配置要求回零时才回零。 */
#if (LIFT_ZERO_AT_POWERUP == 1)
                    App_Log("[app] START | zero at powerup, skip homing\r\n");
                    s_app.homing_wait = 0U;
                    App_SetState(APP_RUN);
#else
                    /* 升降必须先建立零点, 否则所有高度指令都不可信 */
                    if (App_BeginHoming() != 0U)
                    {
                        App_Log("[app] homing FAILED\r\n");
                        s_app.fault = APP_FAULT_LIFT_RANGE;
                        App_SetState(APP_FAULT);
                    }
                    else
                    {
                        App_Log("[app] START | homing...\r\n");
                        s_app.homing_wait = 1U;
                        App_SetState(APP_RUN);
                    }
#endif
                }
            }
            break;
        }

        case APP_RUN:
        {
            /* P2-2: 110s 硬截止在**所有** APP_RUN 分支之前判,
               回零阶段也要受同一约束(虽然 10s 回零超时会先到,
               但两条路径不该有例外)。 */
            if ((HAL_GetTick() - s_app.run_start_ms) >= MATCH_SAFE_DEADLINE_MS)
            {
                App_Log("[app] SAFE DEADLINE -> finish\r\n");
                s_app.fault = APP_FAULT_NONE;
                App_SetState(APP_FINISH);
                break;
            }

            /* 回零阶段: 期间不跑脚本(升降在被占用), 只等回零结果 */
            if (s_app.homing_wait != 0U)
            {
                LiftHomingState_e hs = Lift_GetHomingState();

                if (hs == LIFT_HOMING_DONE)
                {
                    s_app.homing_wait = 0U;
                    App_Log("[app] homing done\r\n");
                }
                else if ((hs == LIFT_HOMING_FAILED) ||
                         ((HAL_GetTick() - s_app.run_start_ms) >= MATCH_HOMING_TIMEOUT_MS))
                {
                    App_Log("[app] homing TIMEOUT/FAILED\r\n");
                    s_app.fault = APP_FAULT_LIFT_RANGE;
                    App_SetState(APP_FAULT);
                    break;
                }
                else
                {
                    break;      /* 还没回零好, 本圈不推进脚本 */
                }
            }

            App_SafetyCheck();

            if (s_app.state == APP_RUN)
            {
                if (App_RunScript())
                {
                    App_SetState(APP_FINISH);
                }
            }
            break;
        }

        case APP_FINISH:
        {
            /* 分阶段收尾: 每圈推进一步, 非阻塞 */
            App_DoFinishSequence();

            /* LED 只在所有电机禁用、且收尾静置时间到了才点亮 */
            if ((AppFinishPhase_e)s_app.finish_phase == FIN_DONE)
            {
                App_LedWrite(1U);
            }
            break;
        }

        case APP_FAULT:
        {
            if (s_app.estop_done == 0U)
            {
                s_app.estop_done = 1U;
                DC_Motor_EmergencyStopAll();
                Lift_Stop();
                Bucket_Disarm();
                /* P1-1: 故障收尾从"等停稳"开始(急停已经下发) */
                s_app.finish_phase = FIN_WAIT_STOP;
                s_app.phase_ms     = HAL_GetTick();
                App_Log("[app] FAULT code=%u\r\n", (unsigned)s_app.fault);
            }

#if (RUN_MODE == RUN_MATCH)
            /* P1-1: 比赛模式下 FAULT 也必须走完"结束"流程 ——
               R25 要求结束时用 LED 表示已结束。急停已在上面做完,
               这里只等执行器确实停下 -> 禁用 -> 静置 -> 常亮。
               故障码留在 RAM 里, 赛后用 Debug 版读以区分原因。 */
            if ((AppFinishPhase_e)s_app.finish_phase == FIN_DONE)
            {
                App_LedWrite(1U);         /* 常亮 = 已结束 */
            }
            else
            {
                App_DoFaultFinish();
            }
#else
            /* 调试模式: 快闪, 方便现场区分"故障"与"正常结束" */
            App_LedWrite((uint8_t)(((HAL_GetTick() - s_app.state_ms) / FAULT_LED_BLINK_MS) & 1U));
#endif
            break;
        }

        case APP_BOOT:
        default:
            App_SetState(APP_READY);
            break;
    }
}

/**
  * @brief  5ms 控制节拍(TIM4 中断上下文)
  * @note   顺序: 底盘闭环 -> 运动规划 -> 升降(分频 10ms) -> 舵机 -> 堵转检测
  * @note   升降模块的 PID 按 control_period_s = 0.01(10ms) 配置,
  *         必须按 10ms 调用; 用 5ms 调会让积分/微分项的时间基准翻倍。
  */
void App_ControlTick(void)
{
    static uint8_t div2;
    uint32_t now = HAL_GetTick();

    if (s_chassis_ok != 0U)
    {
        Chassis_Task(&s_chassis, now);
        ChassisMotion_Tick();
    }

    /* 升降: 10ms 一次(与控制环 PID 的时间基准一致) */
    div2 ^= 1U;
    if (div2 != 0U)
    {
        Lift_Tick();
    }

    Bucket_Tick();
    DC_Motor_Tick();
}

AppState_e App_GetState(void)      { return s_app.state; }
uint8_t    App_GetFault(void)      { return s_app.fault; }
uint32_t   App_GetElapsedMs(void)
{
    return (s_app.state == APP_RUN) ? (HAL_GetTick() - s_app.run_start_ms) : 0U;
}

const Chassis_Object *App_GetChassis(void)
{
    return s_chassis_ok ? &s_chassis : NULL;
}

const Elevating_Object *App_GetLift(void)
{
    return Lift_GetObject();
}

struct AppBucket *App_GetBucket(void)
{
    return NULL;   /* 本工程铲斗是无状态舵机模块, 不对外暴露对象 */
}