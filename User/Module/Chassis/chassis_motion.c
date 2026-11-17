/**
 * @file    chassis_motion.c
 * @brief   底盘运动原语实现: 非阻塞直线/转角 + 无陀螺仪航向保持
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis_motion.h"
#include "main.h"        /* HAL_GetTick */
#include <math.h>

/* Private macros ------------------------------------------------------------*/

#define CMOTION_PI            3.14159265358979323846f
#define CMOTION_DEG2RAD       (CMOTION_PI / 180.0f)
#define CMOTION_RAD2DEG       (180.0f / CMOTION_PI)

#define CMOTION_MIN_V         30.0f    /* 低于此速度不启动, 防死区里打转 */
#define CMOTION_MIN_W         5.0f
#define CMOTION_DEFAULT_TMO   15000U   /* 未给超时时的兜底 */

/* Private variables ---------------------------------------------------------*/

static Chassis_Object *s_chassis;

static ChassisMotion_e      s_action = CMOVE_IDLE;
static ChassisMotionState_e s_state  = CMOTION_IDLE;

static float    s_target;        /* 距离 mm 或角度 deg */
static float    s_speed;         /* mm/s 或 deg/s */
static float    s_start_x, s_start_y, s_start_yaw;   /* 本动作起点(里程计原始值) */
static float    s_progress;      /* 已完成的距离/角度 */

/* SetPose 设定的起点偏置: GetPose = 里程计增量 + 偏置。
   与 s_start_* 分开存放 —— 后者每个动作都会被覆盖。 */
static float    s_pose_ofs_x;
static float    s_pose_ofs_y;
static float    s_pose_ofs_yaw;

static uint32_t s_start_ms;
static uint32_t s_timeout_ms = CMOTION_DEFAULT_TMO;

static ChassisMotionReport_t s_report;

/* 航向保持(直线) */
static float s_heading_ref_deg;

/* Private function declarations ---------------------------------------------*/

static float CMotion_Abs(float v) { return (v < 0.0f) ? -v : v; }
static float CMotion_Sign(float v) { return (v < 0.0f) ? -1.0f : 1.0f; }

static float CMotion_WrapDeg(float deg)
{
    while (deg > 180.0f)  { deg -= 360.0f; }
    while (deg < -180.0f) { deg += 360.0f; }
    return deg;
}

static uint32_t CMotion_EstimateTimeout(float magnitude, float speed)
{
    float sec;

    if (speed <= 0.0f)
    {
        return CMOTION_DEFAULT_TMO;
    }
    /* 理论时间 x 2 + 2s 余量 */
    sec = (magnitude / speed) * 2.0f + 2.0f;
    if (sec > 60.0f)
    {
        sec = 60.0f;
    }
    return (uint32_t)(sec * 1000.0f);
}

static void CMotion_Begin(ChassisMotion_e action, float target, float speed,
                          uint32_t timeout_ms)
{
    float x, y, yaw;

    ChassisMotion_GetPose(&x, &y, &yaw);

    s_action      = action;
    s_state       = CMOTION_RUN;
    s_target      = target;
    s_speed       = speed;
    s_start_x     = x;
    s_start_y     = y;
    s_start_yaw   = yaw;
    s_progress    = 0.0f;
    s_start_ms    = HAL_GetTick();

    s_timeout_ms  = (timeout_ms == 0U) ?
                    CMotion_EstimateTimeout(CMotion_Abs(target), CMotion_Abs(speed)) :
                    timeout_ms;

    s_heading_ref_deg = yaw;

    s_report.action     = action;
    s_report.state      = CMOTION_RUN;
    s_report.target     = target;
    s_report.achieved   = 0.0f;
    s_report.elapsed_ms = 0U;
}

/**
  * @brief  结束当前动作并记录报告
  */
static void CMotion_Finish(ChassisMotionState_e state)
{
    s_state        = state;
    s_report.state = state;
    s_report.achieved   = s_progress;
    s_report.elapsed_ms = HAL_GetTick() - s_start_ms;
    s_action       = CMOVE_IDLE;

    if (s_chassis != NULL)
    {
        (void)Chassis_Stop(s_chassis);
    }
}

/* Exported functions ---------------------------------------------------------*/

void ChassisMotion_Bind(Chassis_Object *chassis)
{
    s_chassis = chassis;
}

void ChassisMotion_SetPose(float x_mm, float y_mm, float theta_deg)
{
    if (s_chassis == NULL)
    {
        return;
    }

    /* 里程计清零后再设偏置: 之后 GetPose() = 清零后的增量 + 本偏置,
       等价于"现在车处在 (x_mm, y_mm, theta_deg)"。 */
    Chassis_ResetOdometry(s_chassis);

    s_pose_ofs_x   = x_mm;
    s_pose_ofs_y   = y_mm;
    s_pose_ofs_yaw = theta_deg;

    s_heading_ref_deg = theta_deg;
}

void ChassisMotion_GetPose(float *x_mm, float *y_mm, float *theta_deg)
{
    Chassis_Odometry_t odom;
    float ox = 0.0f, oy = 0.0f, oth = 0.0f;

    if (s_chassis != NULL)
    {
        Chassis_GetOdometry(s_chassis, &odom);
        ox  = odom.x_mm;
        oy  = odom.y_mm;
        oth = odom.yaw_deg;
    }

    if (x_mm != NULL)     { *x_mm = ox + s_pose_ofs_x; }
    if (y_mm != NULL)     { *y_mm = oy + s_pose_ofs_y; }
    if (theta_deg != NULL)
    {
        *theta_deg = CMotion_WrapDeg(oth + s_pose_ofs_yaw);
    }
}

bool ChassisMotion_PoseInField(float x_mm, float y_mm)
{
    return (x_mm >= GEOFENCE_MARGIN_MM) &&
           (x_mm <= (TRACK_X_MAX - GEOFENCE_MARGIN_MM)) &&
           (y_mm >= GEOFENCE_MARGIN_MM) &&
           (y_mm <= (TRACK_Y_MAX - GEOFENCE_MARGIN_MM));
}

uint8_t ChassisMotion_MoveStraight(float dist_mm, float v_mms, uint32_t timeout_ms)
{
    float v;

    if ((s_chassis == NULL) || (s_state == CMOTION_RUN))
    {
        return (s_chassis == NULL) ? 1U : 2U;
    }
    if ((dist_mm == 0.0f) || !(v_mms == v_mms))
    {
        return 1U;
    }

    v = CMotion_Abs(v_mms);
    if (v < CMOTION_MIN_V) { v = CMOTION_MIN_V; }
    if (v > CHASSIS_V_MAX_MMS) { v = CHASSIS_V_MAX_MMS; }

    CMotion_Begin(CMOVE_STRAIGHT, dist_mm, v, timeout_ms);
    return 0U;
}

uint8_t ChassisMotion_Turn(float angle_deg, float w_dps, uint32_t timeout_ms)
{
    float w;

    if ((s_chassis == NULL) || (s_state == CMOTION_RUN))
    {
        return (s_chassis == NULL) ? 1U : 2U;
    }
    if ((angle_deg == 0.0f) || !(w_dps == w_dps))
    {
        return 1U;
    }

    w = CMotion_Abs(w_dps);
    if (w < CMOTION_MIN_W) { w = CMOTION_MIN_W; }
    if (w > CHASSIS_W_MAX_DPS) { w = CHASSIS_W_MAX_DPS; }

    CMotion_Begin(CMOVE_TURN, angle_deg, w, timeout_ms);
    return 0U;
}

uint8_t ChassisMotion_SetVelocity(float v_mms, float w_dps)
{
    if ((s_chassis == NULL) || (s_state == CMOTION_RUN))
    {
        return (s_chassis == NULL) ? 1U : 2U;
    }
    CMotion_Begin(CMOVE_VELOCITY, 0.0f, 0.0f, 0U);
    s_timeout_ms = 0xFFFFFFFFU; /* 不自动超时, 由调用方 Stop */
    return Chassis_SetVelocity(s_chassis, v_mms, w_dps);
}

void ChassisMotion_Stop(void)
{
    if (s_chassis == NULL)
    {
        return;
    }
    if (s_state == CMOTION_RUN)
    {
        CMotion_Finish(CMOTION_DONE);
    }
    (void)Chassis_Stop(s_chassis);
}

void ChassisMotion_Hold(void)
{
    if (s_chassis == NULL)
    {
        return;
    }
    s_state = CMOTION_IDLE;
    s_action = CMOVE_IDLE;
    (void)Chassis_Hold(s_chassis);
}

void ChassisMotion_Tick(void)
{
    float x, y, yaw;
    float dx, dy;
    float err, w_cmd;

    if ((s_chassis == NULL) || (s_state != CMOTION_RUN))
    {
        return;
    }

    /* 超时检查(优先级最高) */
    if ((s_timeout_ms != 0xFFFFFFFFU) &&
        ((HAL_GetTick() - s_start_ms) >= s_timeout_ms))
    {
        CMotion_Finish(CMOTION_TIMEOUT);
        return;
    }

    ChassisMotion_GetPose(&x, &y, &yaw);

    dx = x - s_start_x;
    dy = y - s_start_y;

    switch (s_action)
    {
        case CMOVE_STRAIGHT:
        {
            /* 沿起点朝向的投影 = 有符号前进距离 */
            float rad = s_start_yaw * CMOTION_DEG2RAD;
            float along = dx * cosf(rad) + dy * sinf(rad);

            s_progress = along;

            if (CMotion_Sign(s_target) * (s_target - along) <= CHASSIS_POS_TOL_MM)
            {
                CMotion_Finish(CMOTION_DONE);
                return;
            }

            /* 航向保持: 编码器航向误差 -> 角速度修正 */
            err   = CMotion_WrapDeg(s_heading_ref_deg - yaw);
            w_cmd = HEADING_HOLD_KP * err;
            if (w_cmd > HEADING_HOLD_MAX_DPS)  { w_cmd = HEADING_HOLD_MAX_DPS; }
            if (w_cmd < -HEADING_HOLD_MAX_DPS) { w_cmd = -HEADING_HOLD_MAX_DPS; }

            (void)Chassis_SetVelocity(s_chassis, CMotion_Sign(s_target) * s_speed, w_cmd);
            break;
        }

        case CMOVE_TURN:
        {
            float dth = CMotion_WrapDeg(yaw - s_start_yaw);

            s_progress = dth;

            if (CMotion_Sign(s_target) * (s_target - dth) <= CHASSIS_ANG_TOL_DEG)
            {
                CMotion_Finish(CMOTION_DONE);
                return;
            }

            (void)Chassis_SetVelocity(s_chassis, 0.0f, CMotion_Sign(s_target) * s_speed);
            break;
        }

        case CMOVE_VELOCITY:
        default:
            /* 速度模式: 由调用方自行停止, 这里只累计走过的距离 */
            s_progress = sqrtf(dx * dx + dy * dy);
            break;
    }
}

bool ChassisMotion_IsDone(void)
{
    return (s_state == CMOTION_DONE) ||
           (s_state == CMOTION_TIMEOUT) ||
           (s_state == CMOTION_FAULT) ||
           (s_state == CMOTION_IDLE);
}

ChassisMotionState_e ChassisMotion_GetState(void)
{
    return s_state;
}

void ChassisMotion_GetReport(ChassisMotionReport_t *out)
{
    if (out != NULL)
    {
        *out = s_report;
    }
}