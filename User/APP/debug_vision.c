/**
 * @file    debug_vision.c
 * @brief   蓝牙可视化调试服务实现(串口: USART1, 115200 8N1)
 *
 * @note    【串口共用策略】本工程蓝牙按用户要求接 USART1, 与文本打印同口。
 *          DEBUG_VISION_ENABLE==1 时全工程关闭文本打印(见 app.c / debug.c),
 *          该串口只跑 btdbg 的 COBS 二进制帧, 不会混帧。
 *          代价: 打开可视化时不能用串口助手看 [PASS]/[FAIL] 文本。
 *
 * @note    【数据流】
 *          USART1 空闲中断(drv_uart)
 *              -> 本文件 RxByte -> btdbg_rx_byte()   (中断上下文, 库的无锁环形缓冲)
 *          主循环 DebugVision_Loop
 *              -> btdbg_poll() -> 参数 SET 回调 -> DebugVision_OnParamChanged()
 *              -> btdbg_telemetry_due() -> btdbg_send_telemetry()
 *          5ms 控制环 DebugVision_Snapshot
 *              -> 只写 volatile 快照, 不调用库
 */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"

#if (DEBUG_VISION_ENABLE == 1)

#include "debug_vision.h"
#include "app.h"
#include "app_config.h"
#include "chassis.h"
#include "chassis_motion.h"
#include "lift.h"
#include "bucket.h"
#include "dc_motor.h"
#include "encoder_motor.h"
#include "drv_uart.h"
#include "drv_tim.h"
#include "usart.h"
#include "main.h"

#include <string.h>
#include <stddef.h>     /* offsetof */

/*==============================================================================
 * 场景会用到电机的统一开关
 *============================================================================*/
#if (VISION_SCENE == VISION_SCENE_WHEEL_SPD) || \
    (VISION_SCENE == VISION_SCENE_STRAIGHT)  || \
    (VISION_SCENE == VISION_SCENE_SCRIPT)
#define DV_USES_CHASSIS  1
#else
#define DV_USES_CHASSIS  0
#endif

#if (VISION_SCENE == VISION_SCENE_LIFT_POS)
#define DV_USES_LIFT     1
#else
#define DV_USES_LIFT     0
#endif

/*==============================================================================
 * 安全硬上限: 每个会动的场景自带超时, 到点急停
 *============================================================================*/
#define DV_SCENE_TIMEOUT_MS     15000U   /* 单个运动场景最长 15s */
#define DV_STATS_PERIOD_MS      2000U    /* 链路统计打印周期 */
#define DV_TX_TIMEOUT_MS        20U      /* 阻塞发送超时 */

/*==============================================================================
 * 控制环 -> 遥测 的快照(volatile)
 *============================================================================*/
typedef struct
{
    volatile uint32_t seq;                /* 序号: 写前后各 +1, 读时校验一致 */

    volatile float wheel_actual[4];       /* 四轮实际速度 mm/s */
    volatile float wheel_target[4];       /* 四轮目标速度 mm/s */
    volatile float lift_pos;              /* 升降实际位置 mm */
    volatile float lift_target;           /* 升降目标位置 mm */
    volatile float lift_duty;             /* 升降输出千分比 */

    volatile float car_speed;             /* 车体线速度 mm/s */
    volatile float car_dist;              /* 累计里程 mm */
    volatile float car_yaw;               /* 航向 deg */

    volatile float snapshot_valid;
} DvSnapshot_t;

static DvSnapshot_t s_snap;

/*==============================================================================
 * btdbg 接入
 *============================================================================*/
static btdbg_config_t s_cfg;

static DvSnapshot_t   s_read;         /* Loop 里拷贝出来的稳定副本 */
static uint32_t       s_scene_start_ms;
static uint8_t        s_scene_armed;
static uint8_t        s_fault_reason;

/* 升降位置阶跃场景的"上/下"方向记忆 */
#if DV_USES_LIFT
static uint8_t        s_lift_dir_up;
#endif

/* 参数同步标志(主循环读, 回调里写) */
static volatile uint8_t s_param_dirty_pos;
static volatile uint8_t s_param_dirty_spd;
static volatile uint8_t s_param_dirty_yaw;
static volatile uint8_t s_param_dirty_geom;

/*==============================================================================
 * 发送
 *============================================================================*/
static void DebugVision_Write(const uint8_t *data, uint16_t len)
{
    /* 库在主循环上下文调用本函数; 阻塞发送保证返回时数据已拷进移位寄存器,
       库可复用其内部编码缓冲。USART1 未配 DMA, 用 IT 发送 + 忙等待完成。 */
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, len, DV_TX_TIMEOUT_MS);
}

static uint32_t DebugVision_Millis(void)
{
    return HAL_GetTick();
}

/*==============================================================================
 * 参数 -> 控制对象 同步
 *============================================================================*/
static void DebugVision_OnParamChanged(const btdbg_param_t *p)
{
    if (p == NULL)
    {
        return;
    }

    /* 只打标记, 真正的同步放在主循环里做(回调本身就在主循环 btdbg_poll 内,
       但关中断赋值必须集中, 便于审计) */
    if ((p->ptr == (void *)&g_pos_kp)  || (p->ptr == (void *)&g_pos_ki)  ||
        (p->ptr == (void *)&g_pos_kd)  || (p->ptr == (void *)&g_pos_i_lim) ||
        (p->ptr == (void *)&g_pos_out_lim) || (p->ptr == (void *)&g_pos_deadband))
    {
        s_param_dirty_pos = 1U;
    }
    else if ((p->ptr == (void *)&g_spd_kp) || (p->ptr == (void *)&g_spd_ki) ||
             (p->ptr == (void *)&g_spd_kd) || (p->ptr == (void *)&g_spd_i_lim) ||
             (p->ptr == (void *)&g_spd_out_lim))
    {
        s_param_dirty_spd = 1U;
    }
    else if ((p->ptr == (void *)&g_yaw_kp) || (p->ptr == (void *)&g_yaw_kd))
    {
        s_param_dirty_yaw = 1U;
    }
    else if ((p->ptr == (void *)&g_wheel_dia) || (p->ptr == (void *)&g_enc_cpr))
    {
        s_param_dirty_geom = 1U;
    }
    /* 陀螺仪 / CG_OFFSET / TGT_SPEED / ACC_LIMIT: 本车不使用或无需同步 */
}

/**
  * @brief  把上/上位机写入的参数同步到控制对象
  * @note   临界区只做赋值; 改增益时同时清积分, 防止积分值在新增益下突跳。
  */
static void DebugVision_SyncParams(void)
{
    __disable_irq();

    if (s_param_dirty_pos)
    {
        s_param_dirty_pos = 0U;
        __enable_irq();

        /* 位置环在 Elevating 内部, 通过 Lift 暴露的对象直接写增益 */
        {
            Elevating_Object *lift = Lift_GetObject();

            if (lift != NULL)
            {
                lift->pos_pid.Kp           = g_pos_kp;
                lift->pos_pid.Ki           = g_pos_ki;
                lift->pos_pid.Kd           = g_pos_kd;
                lift->pos_pid.IntegralLimit = g_pos_i_lim;
                lift->pos_pid.MaxOut       = g_pos_out_lim;
                lift->pos_pid.DeadBand     = g_pos_deadband;
                PID_Reset_Integral(&lift->pos_pid);
            }
        }
        __disable_irq();
    }

    if (s_param_dirty_spd)
    {
        s_param_dirty_spd = 0U;
        __enable_irq();

        /* 本工程底盘速度环在 AT8236 驱动板内部, 主控没有轮速 PID。
           这里把上位机调的 SPD_* 写进升降电机速度环(唯一的板载速度环),
           并在 README 中说明: 底盘速度环请在驱动板上位机用 $mpid 调。 */
        {
            Elevating_Object *lift = Lift_GetObject();

            if (lift != NULL)
            {
                lift->motor.pid.Kp           = g_spd_kp;
                lift->motor.pid.Ki           = g_spd_ki;
                lift->motor.pid.Kd           = g_spd_kd;
                lift->motor.pid.IntegralLimit = g_spd_i_lim;
                lift->motor.pid.MaxOut       = g_spd_out_lim;
                PID_Reset_Integral(&lift->motor.pid);
            }
        }
        __disable_irq();
    }

    if (s_param_dirty_yaw)
    {
        s_param_dirty_yaw = 0U;
        /* 航向保持参数在 chassis_motion.c 内是编译期常量, 本版不热改;
           保留标志位以免误以为已生效。需要热改时在 chassis_motion 里加 setter。 */
    }

    if (s_param_dirty_geom)
    {
        s_param_dirty_geom = 0U;
        /* 轮径/CPR 改动会影响驱动板侧换算与里程计, 仅允许 IDLE 写入;
           生效需要重新 Init 底盘, 这里只记录, 由下次上电生效。 */
    }

    __enable_irq();
}

/*==============================================================================
 * 场景推进
 *============================================================================*/
static void DebugVision_SceneReset(void)
{
    s_scene_armed    = 0U;
    s_scene_start_ms = HAL_GetTick();
}

/**
  * @brief  推进当前场景(主循环)
  * @note   任何一个会动的场景都必须由"车上的启动按键"触发, 不接受上位机指令
  */
static void DebugVision_SceneTick(void)
{
#if DV_USES_CHASSIS || DV_USES_LIFT
    uint32_t elapsed = HAL_GetTick() - s_scene_start_ms;
#endif
#if DV_USES_CHASSIS
    const Chassis_Object *ch = App_GetChassis();
#endif

    switch ((VisionScene_t)(VISION_SCENE))
    {
        case DV_SCENE_LINK:
            /* V1: 只发遥测, 不动任何执行器 */
            break;

        case DV_SCENE_OBSERVE:
            /* V6: 只读, 不动任何执行器 */
            break;

        case DV_SCENE_WHEEL_SPD:
#if DV_USES_CHASSIS
            if (ch == NULL) { break; }
            if (s_scene_armed == 0U)
            {
                /* 必须按键触发 */
                if (HAL_GPIO_ReadPin(START_KEY_PORT, START_KEY_PIN) == START_KEY_ACTIVE_STATE)
                {
                    s_scene_armed    = 1U;
                    s_scene_start_ms = HAL_GetTick();
                    DC_Motor_Enable(DC_MOTOR_FL, true);
                    Bucket_Arm();
                }
                break;
            }
            if (elapsed >= DV_SCENE_TIMEOUT_MS)
            {
                DC_Motor_EmergencyStopAll();
                btdbg_set_state(BTDBG_ST_IDLE, 0U);
                s_scene_armed = 0U;
                break;
            }
            /* 目标速度来自 TGT_SPEED(m/s) */
            (void)Chassis_SetWheelSpeeds((Chassis_Object *)ch,
                                         g_tgt_speed * 1000.0f,
                                         g_tgt_speed * 1000.0f);
            btdbg_set_state(BTDBG_ST_RUNNING, 0U);
#endif
            break;

        case DV_SCENE_LIFT_POS:
#if DV_USES_LIFT
            if (s_scene_armed == 0U)
            {
                if (HAL_GPIO_ReadPin(START_KEY_PORT, START_KEY_PIN) == START_KEY_ACTIVE_STATE)
                {
                    s_scene_armed    = 1U;
                    s_scene_start_ms = HAL_GetTick();
                    DC_Motor_Enable(DC_MOTOR_LIFT, true);
                    (void)Lift_StartHoming();
                }
                break;
            }
            if (elapsed >= DV_SCENE_TIMEOUT_MS)
            {
                Lift_Stop();
                btdbg_set_state(BTDBG_ST_IDLE, 0U);
                s_scene_armed = 0U;
                break;
            }
            if (Lift_GetHomingState() == LIFT_HOMING_DONE)
            {
                /* 回零后在 0 与 LEVEL1 之间阶跃(状态放静态量, 不放 static 局部) */
                float h = Lift_GetHeight();

                if ((s_lift_dir_up == 0U) && (h < 1.0f))
                {
                    (void)Lift_SetHeight(LIFT_H_LEVEL1);
                    s_lift_dir_up = 1U;
                }
                else if ((s_lift_dir_up == 1U) && (h > (LIFT_H_LEVEL1 - 1.0f)))
                {
                    (void)Lift_SetHeight(0.0f);
                    s_lift_dir_up = 0U;
                }
            }
            btdbg_set_state(BTDBG_ST_RUNNING, 0U);
#endif
            break;

        case DV_SCENE_STRAIGHT:
        case DV_SCENE_SCRIPT:
#if DV_USES_CHASSIS
            if (ch == NULL) { break; }
            if (s_scene_armed == 0U)
            {
                if (HAL_GPIO_ReadPin(START_KEY_PORT, START_KEY_PIN) == START_KEY_ACTIVE_STATE)
                {
                    s_scene_armed    = 1U;
                    s_scene_start_ms = HAL_GetTick();
                    DC_Motor_Enable(DC_MOTOR_FL, true);
                    Bucket_Arm();
                    (void)ChassisMotion_MoveStraight(1000.0f,
                                                     g_tgt_speed * 1000.0f, 0U);
                }
                break;
            }
            if ((elapsed >= DV_SCENE_TIMEOUT_MS) || ChassisMotion_IsDone())
            {
                ChassisMotion_Stop();
                DC_Motor_EmergencyStopAll();
                btdbg_set_state(BTDBG_ST_IDLE, 0U);
                s_scene_armed = 0U;
                break;
            }
            btdbg_set_state(BTDBG_ST_RUNNING, 0U);
#endif
            break;

        default:
            break;
    }
}

/*==============================================================================
 * 遥测
 *============================================================================*/
static void DebugVision_CopySnapshot(void)
{
    uint32_t s1 = 0U;
    uint32_t s2 = 0U;

    do
    {
        s1 = s_snap.seq;
        if ((s1 & 1U) != 0U) { continue; }        /* 正在写, 重试 */

        memcpy(&s_read, (const void *)&s_snap, sizeof(s_read));

        s2 = s_snap.seq;
    }
    while ((s1 != s2) || ((s2 & 1U) != 0U));
}

static void DebugVision_SendTelemetry(void)
{
    float car[3];
    float motor[5];

    DebugVision_CopySnapshot();

    /* CH_CAR: 速度 / 里程 / 航向 */
    car[0] = s_read.car_speed;
    car[1] = s_read.car_dist;
    car[2] = s_read.car_yaw;
    btdbg_send_telemetry(BTDBG_CH_CAR, car, 3U);

    /* CH_MOTOR: FL/FR/RL/RR 实际速度 + 升降位置 */
    motor[0] = s_read.wheel_actual[0];
    motor[1] = s_read.wheel_actual[1];
    motor[2] = s_read.wheel_actual[2];
    motor[3] = s_read.wheel_actual[3];
    motor[4] = s_read.lift_pos;
    btdbg_send_telemetry(BTDBG_CH_MOTOR, motor, 5U);

    /* 本车无陀螺仪, 不发 CH_GYRO(见任务书 v2 §4) */
}

/*==============================================================================
 * 对外接口
 *============================================================================*/

void DebugVision_Init(void)
{
    memset(&s_snap, 0, sizeof(s_snap));
    memset(&s_read, 0, sizeof(s_read));

    /* btdbg_init 保存的是指针, s_cfg 必须是静态生命周期 */
    memset(&s_cfg, 0, sizeof(s_cfg));

    s_cfg.write             = DebugVision_Write;
    s_cfg.millis            = DebugVision_Millis;
    s_cfg.params            = g_car_params;
    s_cfg.param_count       = g_car_param_count;
    s_cfg.fw_ver            = 2U;

    memcpy(s_cfg.uid, (const void *)UID_BASE, 12U);

    s_cfg.on_param_changed  = DebugVision_OnParamChanged;
    s_cfg.save_param        = NULL;      /* 本版不做 Flash 持久化 */
    s_cfg.start_gyro_calib  = NULL;      /* 无陀螺仪 */
    s_cfg.save_pset_ver     = NULL;
    s_cfg.pset_ver          = 0U;
    s_cfg.report_hz_var     = &g_report_hz;

    btdbg_init(&s_cfg);

    /* 把 USART1 的接收挂到 btdbg: drv_uart 已把空闲中断收到的整段数据
       交给 port->on_rx, 这里逐字节喂进库的环形缓冲(中断上下文允许)。 */
    uart1_port.on_rx = DebugVision_RxBytes;

    DebugVision_SceneReset();
}

/**
  * @brief  USART1 接收回调(中断上下文)
  * @note   只能在中断里调 btdbg_rx_byte, 其它 btdbg_* 一律在主循环
  */
void DebugVision_RxBytes(UART_PortObject *port, uint8_t *data, uint16_t size)
{
    (void)port;

    for (uint16_t i = 0; i < size; i++)
    {
        btdbg_rx_byte(data[i]);
    }
}

void DebugVision_Snapshot(void)
{
    Chassis_Feedback_t fb;
    const Chassis_Object *ch = App_GetChassis();
    uint8_t i;

    s_snap.seq++;      /* 偶数 -> 奇数: 开始写 */

    if (ch != NULL)
    {
        Chassis_GetFeedback(ch, &fb);
        for (i = 0; i < 4U; i++)
        {
            s_snap.wheel_actual[i] = fb.wheel_speed_mmps[i];
            s_snap.wheel_target[i] = Chassis_GetWheelCommand(ch, i);
        }
    }

    s_snap.lift_pos    = Lift_GetHeight();
    s_snap.lift_target = Lift_GetTarget();
    s_snap.lift_duty   = (float)EncoderMotor_GetDuty(&Lift_GetObject()->motor);

    if (ch != NULL)
    {
        Chassis_Odometry_t odom;

        Chassis_GetOdometry(ch, &odom);
        s_snap.car_speed = odom.linear_mmps;
        s_snap.car_dist  = odom.x_mm;      /* 前进方向累计里程 */
        s_snap.car_yaw   = odom.yaw_deg;
    }

    s_snap.seq++;      /* 奇数 -> 偶数: 写完 */
}

void DebugVision_Loop(void)
{
    static uint32_t s_stats_ms;

    /* 1. 收帧 + 处理参数 SET(回调) */
    btdbg_poll();

    /* 2. 同步参数到控制对象 */
    DebugVision_SyncParams();

    /* 3. 推进场景(含按键触发与超时急停) */
    DebugVision_SceneTick();

    /* 4. 发遥测 */
    if (btdbg_telemetry_due())
    {
        DebugVision_SendTelemetry();
    }

    /* 5. 每 2s 打印链路质量(注意: 与蓝牙同口, 这里用 USART1 文本会混帧,
          所以改为通过 btdbg 的统计字段与上位机侧判断, 不在本口打印文本) */
    (void)s_stats_ms;
}

void DebugVision_SetFault(uint8_t reason)
{
    s_fault_reason = reason;
    btdbg_set_state(BTDBG_ST_FAULT, reason);
}

#endif /* DEBUG_VISION_ENABLE == 1 */