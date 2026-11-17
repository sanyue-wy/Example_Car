/**
 * @file    lift.c
 * @brief   升降台实现: 回零流程 + 高度指令 + 故障汇总
 */

/* Includes ------------------------------------------------------------------*/
#include "lift.h"
#include "main.h"
#include <stdlib.h>     /* abs */

/* Private macros ------------------------------------------------------------*/

/* 回零: 低速向下, 用很小的占空比顶到机械下限, 避免撞坏丝杆 */
#define LIFT_HOMING_DUTY        120        /* 占空比 ‰, 粗略约 12% */
#define LIFT_HOMING_STALL_MS    500U       /* 连续这么久编码器不动 -> 认为到底 */
#define LIFT_HOMING_MOVE_MIN_CNT 4         /* 至少动了这么多计数才算"在动" */
#define LIFT_HOMING_TIMEOUT_MS  8000U      /* 下行总超时, 防止一直顶 */
#define LIFT_HOMING_SETTLE_MS   200U

/* Private variables ---------------------------------------------------------*/

static Elevating_Object  s_lift;
static uint8_t           s_ready;

static LiftHomingState_e s_homing_state;
static uint32_t          s_homing_start_ms;
static uint32_t          s_homing_last_move_ms;
static int32_t           s_homing_last_cnt;

static volatile uint8_t  s_fault;

/* Private functions ---------------------------------------------------------*/

static void Lift_UpdateFault(void)
{
    Elevating_State_e st = Elevating_GetState(&s_lift);

    if (st == ELEVATING_STATE_ERROR)
    {
        s_fault |= LIFT_FAULT_TIMEOUT;
    }
    if (s_lift.motor.stats.fault_count != 0U)
    {
        s_fault |= LIFT_FAULT_MOTOR;
    }
    if (Elevating_GetPositionMm(&s_lift) < (Elevating_GetMinMm(&s_lift) - 0.5f) ||
        Elevating_GetPositionMm(&s_lift) > (Elevating_GetMaxMm(&s_lift) + 0.5f))
    {
        s_fault |= LIFT_FAULT_RANGE;
    }
}

/* Exported functions ---------------------------------------------------------*/

uint8_t Lift_Init(void)
{
    uint8_t ret;

    s_fault       = LIFT_FAULT_NONE;
    s_homing_state = LIFT_HOMING_IDLE;
    s_ready       = 0U;

    ret = Elevating_Init(&s_lift);
    if (ret != 0U)
    {
        return ret;
    }

    /* 上电当前位置暂定为零点; 正式零点由 Lift_StartHoming 建立 */
    Elevating_SetZero(&s_lift);
    Elevating_Stop(&s_lift);

    s_ready = 1U;
    return 0U;
}

void Lift_Tick(void)
{
    int32_t cnt;

    if (s_ready == 0U)
    {
        return;
    }

    Elevating_Task(&s_lift);
    Lift_UpdateFault();

    if (s_homing_state != LIFT_HOMING_DOWN && s_homing_state != LIFT_HOMING_SETTLE)
    {
        return;
    }

    /* ---- 回零: 下行 ---- */
    if (s_homing_state == LIFT_HOMING_DOWN)
    {
        cnt = ENCODER_GetCount(&s_lift.motor.encoder);

        if (abs((int)(cnt - s_homing_last_cnt)) >= LIFT_HOMING_MOVE_MIN_CNT)
        {
            s_homing_last_cnt      = cnt;
            s_homing_last_move_ms  = HAL_GetTick();
        }

        /* 到达机械下限: 长时间不动 */
        if ((HAL_GetTick() - s_homing_last_move_ms) >= LIFT_HOMING_STALL_MS)
        {
            Elevating_Stop(&s_lift);
            s_homing_state = LIFT_HOMING_SETTLE;
            s_homing_start_ms = HAL_GetTick();
            return;
        }

        /* 总超时 */
        if ((HAL_GetTick() - s_homing_start_ms) >= LIFT_HOMING_TIMEOUT_MS)
        {
            Elevating_Stop(&s_lift);
            s_fault |= LIFT_FAULT_HOMING;
            s_homing_state = LIFT_HOMING_FAILED;
            return;
        }

        /* 低速下行: 直接开环给占空比, 不走位置环 */
        EncoderMotor_SetDuty(&s_lift.motor, -LIFT_HOMING_DUTY);
        return;
    }

    /* ---- 回零: 停稳后清零 ---- */
    if ((HAL_GetTick() - s_homing_start_ms) >= LIFT_HOMING_SETTLE_MS)
    {
        Elevating_Stop(&s_lift);
        Elevating_SetZero(&s_lift);
        s_homing_state = LIFT_HOMING_DONE;
    }
}

uint8_t Lift_StartHoming(void)
{
    if ((s_ready == 0U) ||
        (s_homing_state == LIFT_HOMING_DOWN) ||
        (s_homing_state == LIFT_HOMING_SETTLE))
    {
        return 1U;
    }

#if LIFT_HAS_HOME_SWITCH
    /* 下限位开关分支: 本工程未配置该引脚, 直接判失败,
       防止把"有开关但没接"当成"无限位"去堵转。 */
    s_fault |= LIFT_FAULT_HOMING;
    s_homing_state = LIFT_HOMING_FAILED;
    return 1U;
#else
    s_homing_state        = LIFT_HOMING_DOWN;
    s_homing_start_ms     = HAL_GetTick();
    s_homing_last_move_ms = HAL_GetTick();
    s_homing_last_cnt     = ENCODER_GetCount(&s_lift.motor.encoder);
    return 0U;
#endif
}

LiftHomingState_e Lift_GetHomingState(void)
{
    return s_homing_state;
}

uint8_t Lift_SetHeight(float mm)
{
    if (!(mm >= 0.0f && mm <= LIFT_TRAVEL_MAX_MM))
    {
        s_fault |= LIFT_FAULT_RANGE;
        return 1U;
    }
    return (Elevating_MoveTo(&s_lift, mm) == 0U) ? 0U : 1U;
}

float Lift_GetHeight(void)
{
    return Elevating_GetPositionMm(&s_lift);
}

float Lift_GetTarget(void)
{
    return Elevating_GetTargetMm(&s_lift);
}

bool Lift_HasReached(float mm, float tol)
{
    float err;
    Elevating_State_e st;

    if (s_ready == 0U)
    {
        return false;
    }

    err = Elevating_GetPositionMm(&s_lift) - mm;
    if (err < 0.0f) { err = -err; }

    st = Elevating_GetState(&s_lift);
    return (err <= tol) &&
           ((st == ELEVATING_STATE_ARRIVED) || (st == ELEVATING_STATE_IDLE));
}

void Lift_Stop(void)
{
    if (s_ready != 0U)
    {
        Elevating_Stop(&s_lift);
    }
    if (s_homing_state == LIFT_HOMING_DOWN)
    {
        s_homing_state = LIFT_HOMING_FAILED;
    }
}

uint8_t Lift_GetFault(void)
{
    return s_fault;
}

Elevating_Object *Lift_GetObject(void)
{
    return &s_lift;
}