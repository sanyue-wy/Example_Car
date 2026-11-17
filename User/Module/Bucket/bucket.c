/**
 * @file    bucket.c
 * @brief   铲斗舵机实现: 角度 -> 脉宽 -> TIM3_CH1
 */

/* Includes ------------------------------------------------------------------*/
#include "bucket.h"
#include "main.h"       /* HAL_GetTick */
#include "drv_pwm.h"

/* Private macros ------------------------------------------------------------*/

#define BUCKET_FREQ_HZ     50U      /* 舵机标准 50Hz */
#define BUCKET_MIN_US      500U     /* 0 度 */
#define BUCKET_MAX_US      2500U    /* 270 度 */

/* Private variables ---------------------------------------------------------*/

static PWM_ChannelObject s_pwm;
static uint8_t           s_inited;
static uint8_t           s_armed;

static BucketPose_e s_pose;
static uint16_t     s_angle_now;      /* 当前输出角度(度) */
static uint16_t     s_angle_from;     /* 慢速移动起点 */
static uint16_t     s_angle_to;       /* 慢速移动终点 */
static uint32_t     s_move_start_ms;
static uint32_t     s_move_ms;
static bool         s_moving;

/* Private functions ---------------------------------------------------------*/

static uint16_t Bucket_PoseToAngle(BucketPose_e pose)
{
    switch (pose)
    {
        case BUCKET_STOW:  return (uint16_t)BUCKET_ANG_STOW;
        case BUCKET_SCOOP: return (uint16_t)BUCKET_ANG_SCOOP;
        case BUCKET_CARRY: return (uint16_t)BUCKET_ANG_CARRY;
        case BUCKET_DUMP:  return (uint16_t)BUCKET_ANG_DUMP;
        default:           return (uint16_t)BUCKET_ANG_STOW;
    }
}

/**
  * @brief  角度(0~270) -> 脉宽(500~2500us) 并写到通道
  */
static void Bucket_WriteAngle(uint16_t deg)
{
    uint32_t pulse;

    if (deg > 270U) { deg = 270U; }

    pulse = BUCKET_MIN_US +
            ((BUCKET_MAX_US - BUCKET_MIN_US) * (uint32_t)deg) / 270U;

    (void)PWM_Channel_SetPulse(&s_pwm, pulse);
    s_angle_now = deg;
}

/* Exported functions ---------------------------------------------------------*/

void Bucket_Init(void)
{
    uint8_t ret;

    s_armed  = 0U;
    s_moving = false;
    s_pose   = BUCKET_STOW;

    ret = PWM_Channel_Init(&s_pwm, BUCKET_SERVO_TIM, BUCKET_SERVO_CHANNEL,
                           "BUCKET", BUCKET_FREQ_HZ, 0U);
    if (ret != 0U)
    {
        /* 通道冲突属于配置错误, 不能继续: 让上层通过 Bucket_IsArmed 感知 */
        s_inited = 0U;
        return;
    }

    /* 先写安全角度, 保持舵机不抖 */
    Bucket_WriteAngle((uint16_t)BUCKET_ANG_SAFE);
    s_inited = 1U;

#if (BUCKET_SAFE_OUTPUT_AT_INIT == 0)
    /* 严格模式: 上电切断比较输出 */
    (void)HAL_TIM_PWM_Stop(BUCKET_SERVO_TIM, BUCKET_SERVO_CHANNEL);
#endif
}

void Bucket_Arm(void)
{
    if (s_inited == 0U)
    {
        return;
    }
#if (BUCKET_SAFE_OUTPUT_AT_INIT == 0)
    (void)HAL_TIM_PWM_Start(BUCKET_SERVO_TIM, BUCKET_SERVO_CHANNEL);
#endif
    s_armed = 1U;
    (void)Bucket_SetPose(BUCKET_STOW);
}

void Bucket_Disarm(void)
{
    s_armed  = 0U;
    s_moving = false;

    if (s_inited != 0U)
    {
        (void)HAL_TIM_PWM_Stop(BUCKET_SERVO_TIM, BUCKET_SERVO_CHANNEL);
    }
}

uint8_t Bucket_SetPose(BucketPose_e pose)
{
    if (s_inited == 0U)  { return 2U; }
    if (s_armed == 0U)   { return 1U; }
    if (pose >= BUCKET_POSE_NUM) { return 2U; }

    s_pose        = pose;
    s_angle_from  = s_angle_now;
    s_angle_to    = Bucket_PoseToAngle(pose);
    s_move_start_ms = HAL_GetTick();
    s_move_ms     = BUCKET_MOVE_MS;
    s_moving      = true;

    /* BUCKET_MOVE_MS 为 0 时立即到位 */
    if (s_move_ms == 0U)
    {
        Bucket_WriteAngle(s_angle_to);
        s_moving = false;
    }
    return 0U;
}

uint8_t Bucket_SetPoseSlow(BucketPose_e pose, uint16_t ms)
{
    uint8_t ret = Bucket_SetPose(pose);

    if (ret == 0U)
    {
        s_move_ms = (ms == 0U) ? BUCKET_MOVE_MS : ms;
        if (s_move_ms == 0U)
        {
            Bucket_WriteAngle(s_angle_to);
            s_moving = false;
        }
    }
    return ret;
}

bool Bucket_IsDone(void)
{
    return (s_moving == false);
}

void Bucket_Tick(void)
{
    uint32_t elapsed;
    int32_t  from, to, now;
    float    t;

    if ((s_inited == 0U) || (s_armed == 0U) || (s_moving == false))
    {
        return;
    }

    elapsed = HAL_GetTick() - s_move_start_ms;

    if ((s_move_ms == 0U) || (elapsed >= s_move_ms))
    {
        Bucket_WriteAngle(s_angle_to);
        s_moving = false;
        return;
    }

    /* 线性插值 */
    t    = (float)elapsed / (float)s_move_ms;
    from = (int32_t)s_angle_from;
    to   = (int32_t)s_angle_to;

    now = from + (int32_t)((float)(to - from) * t);
    if (now < 0)   { now = 0; }
    if (now > 270) { now = 270; }

    Bucket_WriteAngle((uint16_t)now);
}

BucketPose_e Bucket_GetPose(void)
{
    return s_pose;
}

uint16_t Bucket_GetAngle(void)
{
    return s_angle_now;
}

bool Bucket_IsArmed(void)
{
    return (s_armed != 0U);
}