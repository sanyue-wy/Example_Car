/**
 * @file    chassis.h
 * @brief   四轮差速底盘(滑移转向)运动控制模块(接口层)
 *          (v, ω) -> 左右轮线速度 -> 抽象电机接口; 编码器反馈 -> 里程计
 * @version 1.0
 * @date    2026-11-17
 */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define CHASSIS_MOTOR_NUM        4U   /* M1~M4: 左前/左后/右前/右后 */

/**
  * 是否编译内置的 AT8236 适配器。
  * 置 0 时 chassis.c 完全不依赖 at8236.h, 只保留纯运动学/里程计,
  * 可移植到其它电机总线(PWM、CAN、其它串口板) —— 只需实现 Chassis_MotorOps_t。
  */
#ifndef CHASSIS_ENABLE_AT8236
#define CHASSIS_ENABLE_AT8236    1
#endif

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 解耦结构:
  *
  *   application (app.c / debug)          只调 Chassis_SetVelocity/Stop/Task
  *          |
  *   Chassis (本模块, 纯运动学)            不包含任何 at8236.h/UART/HAL 调用
  *          |  Chassis_MotorOps_t (函数指针)
  *          +--> AT8236 适配器 (chassis.c 末尾, 可裁剪)
  *          +--> 任意其它电机总线 (PWM/CAN/...) 由使用方实现 4 个函数
  *
  * 底盘对外只有"前进速度 v(mm/s) + 角速度 ω(°/s)"与四个轮速, 不出现
  * M1~M4 指令、$spd、串口帧; 换驱动板只替换 ops 实现。
  *
  * 坐标/符号约定(全模块统一, 适配器负责把物理接线差异吸收到 invert_mask):
  *   - x 向前, y 向左, yaw 逆时针为正;
  *   - v > 0 前进, ω > 0 左转(逆时针, 右轮快/左轮慢);
  *   - 轮序: 0=M1 左前, 1=M2 左后, 2=M3 右前, 3=M4 右后;
  *     左侧轮 = (M1+M2)/2, 右侧轮 = (M3+M4)/2。
  *
  * 硬件前提:
  *   - 本工程电机 JCA25-370-103K: 67mm 轮, 11 线编码器, 减速比 103,
  *     输出轴一圈 4532 计数; 12V 空载约 0.30m/s, 所以默认 max_linear=300mm/s;
  *   - 驱动板 AT8236 的 4 路接口按板子丝印 M1~M4 接电机, 左右轮距默认 150mm,
  *     装车后必须实测修改 track_width_mm / wheel_diameter_mm;
  *   - 若某一侧电机机械镜像安装, 正指令反而倒转, 用 invert_mask 对应位取反,
  *     不要靠改接线"凑"方向。
  ******************************************************************************
  */

/**
  * @brief  函数返回码
  */
typedef enum
{
  CHASSIS_OK        = 0, /* 成功 */
  CHASSIS_ERR_PARAM = 1, /* 参数错误 */
  CHASSIS_ERR_STATE = 2, /* 未初始化 */
  CHASSIS_ERR_BUSY  = 3  /* 上一帧指令尚未发完 */
} Chassis_Result_e;

/**
  * @brief  底盘运动模式
  */
typedef enum
{
  CHASSIS_MODE_IDLE     = 0, /* 未初始化/空闲 */
  CHASSIS_MODE_VELOCITY = 1, /* 按目标 v/ω 闭环运行 */
  CHASSIS_MODE_STOPPING = 2, /* 按减速度斜坡回零, 到零后转 HOLD */
  CHASSIS_MODE_HOLD     = 3, /* 主动保持(速度指令 0, 轮子被锁住) */
  CHASSIS_MODE_COAST    = 4  /* 滑行(输出断开) */
} Chassis_Mode_e;

/**
  * @brief  电机反馈(原始量, 轮序 0~3)
  */
typedef struct
{
  float   wheel_speed_mmps[CHASSIS_MOTOR_NUM]; /* 驱动板测得轮速 mm/s */
  int32_t wheel_count[CHASSIS_MOTOR_NUM];      /* 累计编码器计数 */
} Chassis_Feedback_t;

/**
  * @brief  电机总线抽象接口(由使用方实现)
  * @note   set_wheel_speeds: wheel[i] 已按底盘约定换向, 正值=底盘前进方向;
  *         返回 0 成功, 非 0 失败/忙
  *         hold: 主动刹车/锁轮(对应 $spd:0)
  *         coast: 输出断开(对应 $pwm:0)
  *         get_feedback: 填原始轮速/计数, 返回 1 表示反馈有效; 可为 NULL
  */
typedef struct
{
  uint8_t (*set_wheel_speeds)(void *ctx, const float wheel_mmps[CHASSIS_MOTOR_NUM]);
  uint8_t (*hold)(void *ctx);
  uint8_t (*coast)(void *ctx);
  uint8_t (*get_feedback)(void *ctx, Chassis_Feedback_t *feedback);
} Chassis_MotorOps_t;

/**
  * @brief  底盘配置(Init 时整份拷贝进对象)
  */
typedef struct
{
  const char *name;                /* 调试名, 如 "CHASSIS" */

  float    track_width_mm;         /* 左右轮中心距(轮距) */
  float    wheel_diameter_mm;      /* 轮径 */
  uint32_t counts_per_rev;         /* 输出轴一圈编码器计数(11×4×103=4532) */

  float    max_linear_mmps;        /* 线速度上限 */
  float    max_angular_dps;        /* 角速度上限 */
  float    max_wheel_mmps;         /* 单轮线速度上限(驱动板速度指令 ±1000) */
  float    linear_accel_mmps2;     /* 线加速度斜坡 */
  float    angular_accel_dps2;     /* 角加速度斜坡 */
  float    linear_deadband_mmps;   /* 线速度输入死区 */
  float    angular_deadband_dps;   /* 角速度输入死区 */

  uint8_t  invert_mask;            /* 逐轮换向, bit0=M1 ... bit3=M4 */

  uint32_t command_period_ms;      /* 控制指令下发周期(驱动板保持型指令) */
  uint32_t command_timeout_ms;     /* 目标刷新租约, 超时自动停车; 0=关闭 */
  uint32_t feedback_timeout_ms;    /* 反馈超时判定 */
} Chassis_Config_t;

/**
  * @brief  里程计(以 Init 位置为原点)
  */
typedef struct
{
  float    x_mm;          /* 前向位移 */
  float    y_mm;          /* 左向位移 */
  float    yaw_rad;       /* 航向角, 逆时针为正 */
  float    yaw_deg;
  float    linear_mmps;   /* 最近一次反馈的线速度 */
  float    angular_dps;   /* 最近一次反馈的角速度 */
  uint32_t update_ms;     /* 最近一次反馈更新时间(HAL_GetTick 口径) */
  uint8_t  valid;         /* 1=至少收到过一次有效反馈 */
} Chassis_Odometry_t;

/**
  * @brief  运行统计
  */
typedef struct
{
  volatile uint32_t task_count;    /* Task 调用次数 */
  volatile uint32_t cmd_sent;      /* 成功下发到电机总线的次数 */
  volatile uint32_t cmd_busy;      /* 下发失败/忙的次数 */
  volatile uint32_t timeout_count; /* 目标租约超时自动停车次数 */
  volatile uint32_t limit_count;   /* 目标限幅次数 */
  volatile uint32_t feedback_lost; /* 反馈无效次数 */
  volatile uint32_t hold_count;    /* 进入主动保持次数 */
} Chassis_Stats_t;

typedef struct Chassis_Object Chassis_Object;

/**
  * @brief  底盘对象(必须静态分配)
  */
struct Chassis_Object
{
  Chassis_Config_t cfg;                    /* 配置拷贝 */
  const Chassis_MotorOps_t *ops;           /* 电机总线接口 */
  void                    *motor_ctx;      /* 接口上下文(如 AT8236_Object*) */

  Chassis_Mode_e mode;                     /* 当前模式 */
  float target_v_mmps;                     /* 目标线速度 */
  float target_w_dps;                      /* 目标角速度 */
  float cmd_v_mmps;                        /* 斜坡后的线速度 */
  float cmd_w_dps;                         /* 斜坡后的角速度 */
  float wheel_cmd[CHASSIS_MOTOR_NUM];      /* 最近一次下发的四轮线速度 */
  float measured_left_mmps;                /* 左侧轮实测速度 */
  float measured_right_mmps;               /* 右侧轮实测速度 */
  float measured_v_mmps;                   /* 实测线速度 */
  float measured_w_dps;                    /* 实测角速度 */

  Chassis_Feedback_t feedback;             /* 最近一次原始反馈 */
  int32_t last_count[CHASSIS_MOTOR_NUM];   /* 里程计上一拍计数 */
  uint8_t feedback_valid;                  /* 最近一次反馈有效 */
  uint8_t feedback_seen;                   /* 已建立里程计基准 */

  uint32_t last_task_ms;                   /* 上一拍 Task 时间 */
  uint32_t last_target_ms;                 /* 最近一次目标刷新时间 */
  uint32_t last_tx_ms;                     /* 最近一次下发时间 */
  uint8_t  target_dirty;                   /* 目标已更新, Task 下一拍登记租约 */

  /* P0-1: 停车/急停请求只置标志, 真正发送统一在 Chassis_Task 里做。
     主循环不发 AT8236 帧 —— 否则会与中断里的 Task 抢 USART2,
     一旦 BUSY 丢掉 $spd:0, 驱动板按最后速度一直跑。 */
  volatile uint8_t hold_pending;           /* 请求 $spd:0, 发成功才清 */
  volatile uint8_t coast_pending;          /* 请求 $pwm:0, 发成功才清 */
  uint8_t  enabled;                        /* 0 = 强制 HOLD, 忽略一切速度指令(P2-1) */

  Chassis_Odometry_t odom;                 /* 里程计 */
  Chassis_Stats_t    stats;
  uint8_t initialized;
};

/* Exported functions ---------------------------------------------------------*/

/**
  * 初始化/配置
  */
void    Chassis_DefaultConfig(Chassis_Config_t *cfg);
uint8_t Chassis_Init(Chassis_Object *chassis, const Chassis_Config_t *cfg,
                     const Chassis_MotorOps_t *ops, void *motor_ctx);

/**
  * 运行控制
  * @note  SetVelocity/Stop 只更新目标; 实际斜坡与下发由 Chassis_Task 完成。
  */
uint8_t Chassis_SetVelocity(Chassis_Object *chassis, float v_mmps, float omega_dps);
uint8_t Chassis_SetWheelSpeeds(Chassis_Object *chassis, float left_mmps, float right_mmps);
uint8_t Chassis_Stop(Chassis_Object *chassis);
uint8_t Chassis_Hold(Chassis_Object *chassis);
uint8_t Chassis_Coast(Chassis_Object *chassis);
uint8_t Chassis_EmergencyStop(Chassis_Object *chassis);

/**
  * 使能门控(P2-1)
  * @note  禁止后 Chassis_Task 强制 HOLD 并拒绝一切速度指令;
  *        允许后不会自动开始运动, 需再下发目标。
  */
void    Chassis_SetEnabled(Chassis_Object *chassis, uint8_t enabled);
uint8_t Chassis_IsEnabled(const Chassis_Object *chassis);

/**
  * 周期任务(建议 10ms 调用): 斜坡、限幅、下发、反馈、里程计、看门狗
  * @param now_ms 当前毫秒, 必须与 HAL_GetTick 同源
  */
void    Chassis_Task(Chassis_Object *chassis, uint32_t now_ms);

/**
  * 状态查询
  */
Chassis_Mode_e Chassis_GetMode(const Chassis_Object *chassis);
float   Chassis_GetTargetLinearMmps(const Chassis_Object *chassis);
float   Chassis_GetTargetAngularDps(const Chassis_Object *chassis);
float   Chassis_GetLinearMmps(const Chassis_Object *chassis);
float   Chassis_GetAngularDps(const Chassis_Object *chassis);
float   Chassis_GetWheelCommand(const Chassis_Object *chassis, uint8_t index);
void    Chassis_GetFeedback(const Chassis_Object *chassis, Chassis_Feedback_t *feedback);
void    Chassis_GetOdometry(const Chassis_Object *chassis, Chassis_Odometry_t *odom);
void    Chassis_ResetOdometry(Chassis_Object *chassis);
uint8_t Chassis_IsOnline(const Chassis_Object *chassis, uint32_t now_ms);

/**
  * 内置 AT8236 适配器(CHASSIS_ENABLE_AT8236==1 时可用)
  * @note  用法: Chassis_Init(&chassis, &cfg, Chassis_GetAT8236Ops(), &board);
  *        board 为已 AT8236_Init 的对象; 本头文件不暴露任何 AT8236 类型。
  */
#if CHASSIS_ENABLE_AT8236
const Chassis_MotorOps_t *Chassis_GetAT8236Ops(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H__ */
