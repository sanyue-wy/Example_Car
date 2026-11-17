/**
 * @file    elevating.h
 * @brief   丝杆升降平台模块(接口层)
 *          编码器电机 -> 丝杆 -> 平台直线运动: 位置闭环 + 软限位
 * @version 1.0
 * @date    2026-11-17
 */

#ifndef __ELEVATING_H__
#define __ELEVATING_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "encoder_motor.h"

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 分层与解耦:
  *   业务层 -> Elevating(本模块, mm 位置接口)
  *          -> EncoderMotor(设备层, 有符号占空比/转速接口)
  *          -> drv_encoder + drv_pwm(驱动层)
  *
  * 本模块只做"直线位置"相关的事:
  * - 编码器计数 <-> 平台位置(mm)换算(导程/每圈计数);
  * - 位置环: mm 误差 -> 目标转速(rps), 交给 EncoderMotor 的速度环执行;
  * - 软限位、到位判定、移动超时、飞车/断线保护;
  * - 回零(软件零点), 不依赖限位开关。
  *
  * 硬件/机械相关硬编码(方向、导程、行程、CPR、PID 增益)全部集中在
  * elevating.c 文件顶部的"★ 方向 / 机械参数"区块, 换装配只改那一块。
  *
  * 使用约束:
  * 1. 每个控制周期调用一次 Elevating_Task(), 周期由模块内部固定为 10ms;
  * 2. 上电后平台停在机械零点(下限位)附近再调 Elevating_SetZero(),
  *    之后所有位置都以该点为 0、向上为正;
  * 3. MoveTo 的目标超出 [MIN, MAX] 会被拒绝(返回 2), 不会动;
  * 4. 对象必须静态分配, 内部组合的 EncoderMotor/PWM/编码器对象会被驱动层登记。
  ******************************************************************************
  */

/**
  * @brief  升降平台状态
  */
typedef enum
{
  ELEVATING_STATE_IDLE    = 0, /* 未运动/手动停止 */
  ELEVATING_STATE_MOVING  = 1, /* 正在执行 MoveTo/MoveRelative */
  ELEVATING_STATE_ARRIVED = 2, /* 已到达目标(误差在到位阈值内) */
  ELEVATING_STATE_ERROR   = 3, /* 超时/越过软限位/电机故障, 已停车 */
} Elevating_State_e;

typedef struct Elevating_Object Elevating_Object;

/**
  * @brief  升降平台对象
  */
struct Elevating_Object
{
  EncoderMotor_Object motor;    /* 编码器电机(设备层组合) */
  PID_Controller_t    pos_pid;  /* 位置环: mm 误差 -> 目标转速(rps) */

  float    target_mm;           /* 目标位置(mm, 以零点为 0, 向上为正) */
  float    position_mm;         /* 最近一次 Task 采样的位置 */
  float    speed_mm_s;          /* 最近一次 Task 采样的线速度(mm/s, 向上为正) */
  int32_t  zero_count;          /* SetZero 时的编码器计数 */
  uint32_t move_ticks;          /* 本次移动已运行的 Task 次数 */
  uint32_t timeout_ticks;       /* 本次移动的超时阈值(Task 次数) */
  uint32_t fault_ref;           /* 移动开始时的电机 fault_count 基准 */

  Elevating_State_e state;
  uint8_t  initialized;
  uint8_t  last_motor_ret;      /* Init 时 EncoderMotor_Init 的返回码(诊断) */
};

/* Exported functions ---------------------------------------------------------*/

uint8_t  Elevating_Init(Elevating_Object *lift);
void     Elevating_Task(Elevating_Object *lift);
uint8_t  Elevating_MoveTo(Elevating_Object *lift, float target_mm);
uint8_t  Elevating_MoveRelative(Elevating_Object *lift, float delta_mm);
void     Elevating_Stop(Elevating_Object *lift);
void     Elevating_SetZero(Elevating_Object *lift);

float    Elevating_GetPositionMm(const Elevating_Object *lift);
float    Elevating_GetTargetMm(const Elevating_Object *lift);
float    Elevating_GetSpeedMmS(const Elevating_Object *lift);
float    Elevating_GetMinMm(const Elevating_Object *lift);
float    Elevating_GetMaxMm(const Elevating_Object *lift);
Elevating_State_e Elevating_GetState(const Elevating_Object *lift);
uint8_t  Elevating_IsArrived(const Elevating_Object *lift);

#ifdef __cplusplus
}
#endif

#endif /* __ELEVATING_H__ */