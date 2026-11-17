/**
 * @file    chassis_motion.h
 * @brief   底盘运动原语(Module): 在既有 Chassis 速度闭环之上叠加
 *          非阻塞的"走直线 / 转身 / 停靠"动作, 并提供无陀螺仪航向保持。
 *
 * @note    为什么不直接改 chassis.c:
 *          chassis.c 是本工程已有的、能工作的底盘驱动(接口稳定、有自检代码),
 *          按任务书 §0.8 的规定只能复用或在其上扩展, 不得重写。
 *          所以本模块是 chassis 之上的"运动规划/状态机"薄层。
 *
 * @note    层次:
 *            APP (app.c / app_script.c)  ->  本模块(ChassisMotion)
 *                                       ->  Chassis(速度/里程计)
 *                                       ->  AT8236 驱动板
 */

#ifndef __CHASSIS_MOTION_H__
#define __CHASSIS_MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "chassis.h"
#include "app_config.h"

/*==============================================================================
 * 动作类型
 *============================================================================*/
typedef enum
{
    CMOVE_IDLE = 0,
    CMOVE_STRAIGHT,    /* 走直线 dist_mm: 正=前进, 负=后退 */
    CMOVE_TURN,        /* 原地转 angle_deg: 正=逆时针(左转) */
    CMOVE_VELOCITY,    /* 直接给 v/omega(供调试/靠墙校正) */
} ChassisMotion_e;

/**
 * @brief  运动状态
 */
typedef enum
{
    CMOTION_IDLE  = 0,  /* 无动作 */
    CMOTION_RUN   = 1,  /* 执行中 */
    CMOTION_DONE  = 2,  /* 正常完成 */
    CMOTION_TIMEOUT = 3,/* 超时(由调用方决定如何处理) */
    CMOTION_FAULT = 4   /* 越界/无反馈 */
} ChassisMotionState_e;

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief  绑定已初始化的 Chassis 对象(不重写 chassis.c)
 */
void ChassisMotion_Bind(Chassis_Object *chassis);

/**
 * @brief  当前位姿是否在赛道安全区内(含 GEOFENCE 边距)
 * @note   由里程计估计; 越界由 App 层安全监控判 FAULT
 */
bool ChassisMotion_PoseInField(float x_mm, float y_mm);

/**
 * @brief  把里程计重置到给定位姿(比赛开始时设为起始位姿)
 */
void ChassisMotion_SetPose(float x_mm, float y_mm, float theta_deg);

/**
 * @brief  读取估计位姿
 */
void ChassisMotion_GetPose(float *x_mm, float *y_mm, float *theta_deg);

/**
 * @brief  走直线(非阻塞, 立即返回)
 * @param  dist_mm  有符号距离, 正=前进
 * @param  v_mms    绝对速度(mm/s), 会被限幅到 CHASSIS_V_MAX_MMS
 * @param  timeout_ms 超时; 0 = 自动按距离/速度估算 + 2s
 * @retval 0 已接受; 1 参数错误; 2 上一动作未结束
 */
uint8_t ChassisMotion_MoveStraight(float dist_mm, float v_mms, uint32_t timeout_ms);

/**
 * @brief  原地转(非阻塞)
 * @param  angle_deg 有符号角度, 正=逆时针
 * @param  w_dps     绝对角速度(deg/s)
 */
uint8_t ChassisMotion_Turn(float angle_deg, float w_dps, uint32_t timeout_ms);

/**
 * @brief  直接给速度(靠墙校正/调试用), 也要通过 ChassisMotion_Stop 结束
 */
uint8_t ChassisMotion_SetVelocity(float v_mms, float w_dps);

/**
 * @brief  请求停车(走斜坡减速到 0)
 */
void ChassisMotion_Stop(void);

/**
 * @brief  硬停(立即 hold)
 */
void ChassisMotion_Hold(void);

/**
 * @brief  周期任务, 每 5ms 由 App_ControlTick 调用
 */
void ChassisMotion_Tick(void);

/**
 * @brief  动作是否结束(含 DONE/TIMEOUT/FAULT)
 */
bool ChassisMotion_IsDone(void);

ChassisMotionState_e ChassisMotion_GetState(void);

/**
 * @brief  最近一次动作的完成情况(调试打印用)
 */
typedef struct
{
    ChassisMotion_e      action;
    ChassisMotionState_e state;
    float    target;        /* 目标距离/角度 */
    float    achieved;      /* 实际走到的距离/角度(取结束时的估计量) */
    uint32_t elapsed_ms;
} ChassisMotionReport_t;

void ChassisMotion_GetReport(ChassisMotionReport_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_MOTION_H__ */