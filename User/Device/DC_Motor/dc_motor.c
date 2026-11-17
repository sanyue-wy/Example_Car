/**
 * @file    dc_motor.c
 * @brief   执行器统一抽象的实现层
 * @note    本文件不直接操作寄存器, 只做两类真实执行器的"使能门控 + 状态汇总":
 *            - 底盘: 通过 Chassis 对象(内部经 AT8236 适配器下发);
 *            - 升降: 通过 Elevating 对象(内部经 EncoderMotor)。
 *          两个对象由 App 层注入, 用注册函数解耦, 避免 dc_motor 反向依赖 APP。
 */

/* Includes ------------------------------------------------------------------*/
#include "dc_motor.h"
#include "chassis.h"
#include "elevating.h"
#include "app_config.h"

/* Private variables ---------------------------------------------------------*/

/* 外部注入的真实对象(App_Init 里登记) */
static Chassis_Object   *s_chassis;
static Elevating_Object *s_lift;

/* 使能位: 底盘 4 轮共用一个; 升降单独一个 */
static bool s_chassis_enabled;
static bool s_lift_enabled;

/* 堵转标志: 底盘按轮记录, 升降单独一个 */
static volatile bool s_stall[DC_MOTOR_NUM];

/* 急停锁存: 置位后所有 Enable 都无效, 只能 App 层清 */
static volatile bool s_estop_latched;

/* Private macros ------------------------------------------------------------*/

/* 判定"停稳"的速度阈值: 底盘轮速 mm/s, 升降转速 rps */
#define DC_MOTOR_STOP_MMPS   8.0f
#define DC_MOTOR_STOP_RPS    0.05f

/* 堵转判定: |目标| 大于此值且反馈长期不跟随 */
#define DC_MOTOR_STALL_CMD_MMPS  60.0f
#define DC_MOTOR_STALL_CMD_RPS   0.15f
#define DC_MOTOR_STALL_FEEDBACK_MMPS 15.0f
#define DC_MOTOR_STALL_FEEDBACK_RPS  0.03f
#define DC_MOTOR_STALL_MS        STALL_TIMEOUT_MS

/* 堵转计时器(每 5ms 由 App 调用一次 DC_Motor_Tick) */
static uint16_t s_stall_cnt[DC_MOTOR_NUM];

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  登记真实执行器对象(由 App_Init 调用)
  */
void DC_Motor_Bind(Chassis_Object *chassis, Elevating_Object *lift)
{
    s_chassis = chassis;
    s_lift    = lift;
}

/**
  * @brief  初始化: 全部禁用、清所有标志。不向硬件输出任何东西。
  */
void DC_Motor_Init(void)
{
    s_chassis_enabled = false;
    s_lift_enabled    = false;
    s_estop_latched   = false;

    for (uint8_t i = 0; i < (uint8_t)DC_MOTOR_NUM; i++)
    {
        s_stall[i]     = false;
        s_stall_cnt[i] = 0U;
    }

    /* 只把目标清零; 具体"如何停"由各执行器自己的 Init 保证 */
    if (s_chassis != NULL)
    {
        (void)Chassis_Hold(s_chassis);
    }
    if (s_lift != NULL)
    {
        Elevating_Stop(s_lift);
    }
}

/**
  * @brief  允许/禁止执行器动作
  */
void DC_Motor_Enable(DC_MotorId_e id, bool enable)
{
    if (s_estop_latched && enable)
    {
        return; /* 急停锁存: 必须先 DC_Motor_ClearFaults 才能重新使能 */
    }

    if (id <= DC_MOTOR_RR)
    {
        s_chassis_enabled = enable;
        /* P2-1: 使能门控下沉到 Chassis —— 未使能时 Chassis_Task 会强制 HOLD
           并拒绝一切速度指令, 而不只是"发一次 hold" */
        if (s_chassis != NULL)
        {
            Chassis_SetEnabled(s_chassis, enable ? 1U : 0U);
        }
    }
    else if (id == DC_MOTOR_LIFT)
    {
        s_lift_enabled = enable;
        if (!enable && (s_lift != NULL))
        {
            Elevating_Stop(s_lift);
        }
    }
}

/**
  * @brief  紧急停止: 底盘 hold(驱动板 $spd:0), 升降输出 0
  * @note   P0-1: 底盘停车帧由 Chassis_Task 在中断里落地(hold_pending),
  *         本函数只清使能与目标, 不会因 USART2 忙而丢掉停车指令。
  */
void DC_Motor_EmergencyStopAll(void)
{
    s_estop_latched   = true;
    s_chassis_enabled = false;
    s_lift_enabled    = false;

    if (s_chassis != NULL)
    {
        (void)Chassis_SetEnabled(s_chassis, 0U);
        (void)Chassis_Hold(s_chassis);
    }
    if (s_lift != NULL)
    {
        Elevating_Stop(s_lift);
    }
}

/**
  * @brief  堵转检测: 每 5ms 调一次
  * @note   只看"有指令但反馈不跟随"; 目标很小时不计, 避免静止误判
  */
void DC_Motor_Tick(void)
{
    Chassis_Feedback_t fb;
    uint8_t  need = (uint8_t)(DC_MOTOR_STALL_MS / 5U);
    uint8_t  i;

    if (need == 0U)
    {
        need = 1U;
    }

    /* ---- 底盘 4 轮 ---- */
    if ((s_chassis != NULL) && s_chassis_enabled)
    {
        float target[CHASSIS_MOTOR_NUM];

        Chassis_GetFeedback(s_chassis, &fb);

        for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            target[i] = Chassis_GetWheelCommand(s_chassis, i);
        }

        for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            bool commanded = (FLOAT_ABS(target[i]) > DC_MOTOR_STALL_CMD_MMPS);
            bool following = (FLOAT_ABS(fb.wheel_speed_mmps[i]) >
                              DC_MOTOR_STALL_FEEDBACK_MMPS);

            if (commanded && !following)
            {
                if (s_stall_cnt[i] < need) { s_stall_cnt[i]++; }
                if (s_stall_cnt[i] >= need) { s_stall[i] = true; }
            }
            else
            {
                s_stall_cnt[i] = 0U;
            }
        }
    }
    else
    {
        for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            s_stall_cnt[i] = 0U;
        }
    }

    /* ---- 升降 ---- */
    if ((s_lift != NULL) && s_lift_enabled)
    {
        float target = Elevating_GetTargetMm(s_lift);
        float actual = Elevating_GetSpeedMmS(s_lift);
        /* 升降目标是位置, 用"是否在移动状态且位置不变"判堵转更稳妥:
           这里用速度量与位置误差的组合: 目标未到且速度≈0 -> 计数 */
        bool commanded = (FLOAT_ABS(target - Elevating_GetPositionMm(s_lift)) > 2.0f) &&
                         (Elevating_GetState(s_lift) == ELEVATING_STATE_MOVING);
        bool following = (FLOAT_ABS(actual) > 0.5f);

        if (commanded && !following)
        {
            if (s_stall_cnt[DC_MOTOR_LIFT] < need) { s_stall_cnt[DC_MOTOR_LIFT]++; }
            if (s_stall_cnt[DC_MOTOR_LIFT] >= need) { s_stall[DC_MOTOR_LIFT] = true; }
        }
        else
        {
            s_stall_cnt[DC_MOTOR_LIFT] = 0U;
        }
    }
    else
    {
        s_stall_cnt[DC_MOTOR_LIFT] = 0U;
    }
}

/**
  * @brief  所有执行器都停稳?
  */
bool DC_Motor_AllStopped(void)
{
    Chassis_Feedback_t fb;

    if ((s_chassis != NULL) && (Chassis_GetMode(s_chassis) == CHASSIS_MODE_VELOCITY))
    {
        return false; /* 还在按速度跑 */
    }

    if (s_chassis != NULL)
    {
        Chassis_GetFeedback(s_chassis, &fb);
        for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
        {
            if (FLOAT_ABS(fb.wheel_speed_mmps[i]) > DC_MOTOR_STOP_MMPS)
            {
                return false;
            }
        }
    }

    if (s_lift != NULL)
    {
        if (Elevating_GetState(s_lift) == ELEVATING_STATE_MOVING)
        {
            return false;
        }
        if (FLOAT_ABS(Elevating_GetSpeedMmS(s_lift)) > 0.5f)
        {
            return false;
        }
    }

    return true;
}

/**
  * @brief  读取状态快照
  */
void DC_Motor_GetState(DC_MotorId_e id, DC_MotorState_t *out)
{
    Chassis_Feedback_t fb;

    if (out == NULL)
    {
        return;
    }

    out->target  = 0.0f;
    out->actual  = 0.0f;
    out->output  = 0.0f;
    out->enabled = 0U;
    out->stall   = (id < DC_MOTOR_NUM) ? (s_stall[id] ? 1U : 0U) : 0U;

    if ((id <= DC_MOTOR_RR) && (s_chassis != NULL))
    {
        Chassis_GetFeedback(s_chassis, &fb);
        out->target  = Chassis_GetWheelCommand(s_chassis, (uint8_t)id);
        out->actual  = fb.wheel_speed_mmps[id];
        out->output  = out->target;              /* 驱动板闭环, 指令即输出目标 */
        out->enabled = s_chassis_enabled ? 1U : 0U;
    }
    else if ((id == DC_MOTOR_LIFT) && (s_lift != NULL))
    {
        out->target  = Elevating_GetTargetMm(s_lift);
        out->actual  = Elevating_GetPositionMm(s_lift);
        out->output  = (float)EncoderMotor_GetDuty(&((Elevating_Object *)s_lift)->motor);
        out->enabled = s_lift_enabled ? 1U : 0U;
    }
}

/**
  * @brief  清故障标志与急停锁存
  */
void DC_Motor_ClearFaults(void)
{
    for (uint8_t i = 0; i < (uint8_t)DC_MOTOR_NUM; i++)
    {
        s_stall[i]     = false;
        s_stall_cnt[i] = 0U;
    }
    s_estop_latched = false;
}

/**
  * @brief  任一执行器堵转?
  */
bool DC_Motor_AnyStall(void)
{
    for (uint8_t i = 0; i < (uint8_t)DC_MOTOR_NUM; i++)
    {
        if (s_stall[i])
        {
            return true;
        }
    }
    return false;
}