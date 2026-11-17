/**
 * @file    lift.h
 * @brief   升降台(Module): 在既有 Elevating 之上补三件事
 *            1. 回零(Homing): 无下限位时低速下行到堵转/超时, 然后清零;
 *            2. 绝对高度指令 Lift_SetHeight(mm) + 到位判定 Lift_HasReached();
 *            3. 升降专属故障汇总(超程/超时), 供 APP 安全监控读。
 *
 * @note    复用关系: Lift -> Elevating(已有的位置环/软限位) -> EncoderMotor -> drv_tim/drv_pwm/drv_encoder/pid
 *          不重写 elevating.c, 只在其上加"回零流程"与"对外单位统一"。
 */

#ifndef __LIFT_H__
#define __LIFT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "elevating.h"
#include "app_config.h"

/**
 * @brief  回零状态
 */
typedef enum
{
    LIFT_HOMING_IDLE    = 0,
    LIFT_HOMING_DOWN    = 1,   /* 低速下行找零点 */
    LIFT_HOMING_SETTLE  = 2,   /* 停稳后清零 */
    LIFT_HOMING_DONE    = 3,
    LIFT_HOMING_FAILED  = 4
} LiftHomingState_e;

/**
 * @brief  故障位(按位置位)
 */
#define LIFT_FAULT_NONE       0x00U
#define LIFT_FAULT_TIMEOUT    0x01U   /* 位置环超时 */
#define LIFT_FAULT_RANGE      0x02U   /* 越过软限位 */
#define LIFT_FAULT_MOTOR      0x04U   /* 电机层飞车/故障 */
#define LIFT_FAULT_HOMING     0x08U   /* 回零失败 */

/**
 * @brief  绑定并初始化升降台(不输出; 调用后电机占空比为 0)
 */
uint8_t Lift_Init(void);

/**
 * @brief  周期任务(内部调用 Elevating_Task; 每 10ms 一次, 由 App_ControlTick 分频)
 */
void Lift_Tick(void);

/**
 * @brief  请求回零(非阻塞)
 * @note   无下限位时: 以 HOMING_DUTY 低速下行, 直到编码器在 HOMING_STALL_MS 内
 *         几乎不变化, 判为到达机械下限; 然后 Elevating_SetZero()。
 *         有下限位(LIFT_HAS_HOME_SWITCH==1)时走开关分支(引脚未配置则直接失败)。
 * @retval 0 已开始; 1 当前不可用(未初始化/已在回零)
 */
uint8_t Lift_StartHoming(void);

LiftHomingState_e Lift_GetHomingState(void);

/**
 * @brief  设置目标高度(mm, 相对零点, 向上为正)
 * @note   自动钳位到 [0, LIFT_TRAVEL_MAX_MM]; 越界请求返回 1 并不动作
 */
uint8_t Lift_SetHeight(float mm);

/**
 * @brief  当前高度/目标高度
 */
float Lift_GetHeight(void);
float Lift_GetTarget(void);

/**
 * @brief  是否已到指定高度(容差默认 LIFT_ARRIVE_TOL_MM)
 * @note   供 L2 判定"是否真的升到位", 不到位自动降级 L1
 */
bool Lift_HasReached(float mm, float tol);

/**
 * @brief  立即停止升降(输出 0)
 */
void Lift_Stop(void);

/**
 * @brief  升降故障位
 */
uint8_t Lift_GetFault(void);

/**
 * @brief  读取内部对象(供 dc_motor / 调试层)
 */
Elevating_Object *Lift_GetObject(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_H__ */