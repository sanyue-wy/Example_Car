/**
 * @file    dc_motor.h
 * @brief   统一的"可运动执行器"抽象(接口层)
 *
 * @note    【为什么需要这一层】任务书把 5 个执行器写成同一种 PWM 直驱编码电机,
 *          但本车实际是两种完全不同的执行器:
 *            - 4 个底盘轮: 由 AT8236 驱动板内部闭环, 主控只下发 mm/s 速度;
 *            - 1 个升降电机: PWM+方向直驱, 主控自己做速度环/位置环。
 *          若强行套一层"5 路 PWM 速度环", 就会和驱动板的内环打架。
 *          所以这里定义的是**行为**抽象(使能/急停/状态), 具体执行器各自实现。
 *
 * @note    【安全约束】任何执行器在 Enable 之前必须完全不输出:
 *            - 底盘: Chassis_Init 会下发一次 hold, 速度指令为 0;
 *            - 升降: EncoderMotor_Init 后占空比为 0, 方向脚全低(滑行)。
 *          App_Init 阶段不会 Enable 任何执行器, 满足 R11/R14/G27。
 */

#ifndef __DC_MOTOR_H__
#define __DC_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 前置声明: 具体对象由 APP 层持有并在 App_Init 里登记 */
struct Chassis_Object;
struct Elevating_Object;

/*==============================================================================
 * 执行器编号
 *============================================================================*/
typedef enum
{
    /* 编号必须与 chassis 逻辑轮序一致: 0=左前 1=左后 2=右前 3=右后;
       DC_Motor_GetState 直接用 id 索引底盘轮速, 顺序错会读错轮。
       实车通道映射见 chassis_map.h (左前=M4 左后=M3 右前=M1 右后=M2)。 */
    DC_MOTOR_FL   = 0,   /* 左前轮(经 AT8236 M4) */
    DC_MOTOR_RL   = 1,   /* 左后轮(经 AT8236 M3) */
    DC_MOTOR_FR   = 2,   /* 右前轮(经 AT8236 M1) */
    DC_MOTOR_RR   = 3,   /* 右后轮(经 AT8236 M2) */
    DC_MOTOR_LIFT = 4,   /* 升降电机(PWM 直驱) */
    DC_MOTOR_NUM  = 5
} DC_MotorId_e;

/**
 * @brief  执行器运行状态快照(供安全监控/蓝牙遥测读取)
 */
typedef struct
{
    float    target;      /* 目标量: 轮 -> mm/s; 升降 -> 目标转速 rps */
    float    actual;      /* 实际量: 轮 -> mm/s; 升降 -> rps */
    float    output;      /* 输出量: 轮 -> 驱动板指令; 升降 -> 占空比千分比 */
    uint8_t  enabled;     /* 1 = 允许动作 */
    uint8_t  stall;       /* 1 = 检测到堵转(需上层处理) */
} DC_MotorState_t;

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief  初始化执行器抽象层(不输出任何东西)
 * @note   必须在 Chassis_Init / Elevating_Init 之前或之后调用均可,
 *         但 App_Init 中所有执行器初始化完成后调用一次
 */
void DC_Motor_Init(void);

/**
 * @brief  登记真实执行器对象(由 App_Init 调用)
 * @note   传入的对象必须静态分配且生命周期覆盖整个程序
 */
void DC_Motor_Bind(struct Chassis_Object *chassis, struct Elevating_Object *lift);

/**
 * @brief  堵转检测周期任务, 每 5ms 调用一次
 */
void DC_Motor_Tick(void);

/**
 * @brief  允许/禁止某个执行器动作
 * @note   禁止时同时把目标清零; 底盘 4 轮共享一个使能位(驱动板指令是整帧的)
 */
void DC_Motor_Enable(DC_MotorId_e id, bool enable);

/**
 * @brief  紧急停止全部执行器: 底盘 hold, 升降输出 0 并清积分
 * @note   可从中断外任意上下文调用; 内部只做寄存器写与状态清零
 */
void DC_Motor_EmergencyStopAll(void);

/**
 * @brief  判定"所有执行器都真的停了"
 * @note   底盘看驱动板反馈轮速, 升降看编码器转速; 反馈消失也视为已停
 */
bool DC_Motor_AllStopped(void);

/**
 * @brief  读取某个执行器的状态快照
 */
void DC_Motor_GetState(DC_MotorId_e id, DC_MotorState_t *out);

/**
 * @brief  清零所有堵转/超速标志(故障复位用)
 */
void DC_Motor_ClearFaults(void);

/**
 * @brief  查询是否任一执行器报了堵转
 */
bool DC_Motor_AnyStall(void);

#ifdef __cplusplus
}
#endif

#endif /* __DC_MOTOR_H__ */