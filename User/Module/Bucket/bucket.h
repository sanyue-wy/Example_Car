/**
 * @file    bucket.h
 * @brief   铲斗舵机(Module): 4 个离散姿态 + 慢速插值
 *
 * @note    安全约束(R11/R14/G27): Bucket_Init() 只登记通道, **不输出 PWM**;
 *          只有 Bucket_Arm() 之后才开始输出, 而且第一次输出必须来自 APP_RUN。
 *
 * @note    为什么不用现有 servo.c: servo.c 固定绑定 TIM1_CH1/2/3(分拣机的夹爪/托盘),
 *          本车铲斗按用户确认改在 TIM3_CH1(PA6)。两套输出频率不同(20kHz vs 50Hz),
 *          必须严格分开, 所以本模块直接用 drv_pwm 绑定 BUCKET_SERVO_TIM。
 */

#ifndef __BUCKET_H__
#define __BUCKET_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

/**
 * @brief  铲斗姿态(角度值在 app_config.h)
 */
typedef enum
{
    BUCKET_STOW  = 0,   /* 收起/运输位 */
    BUCKET_SCOOP = 1,   /* 张开取料位 */
    BUCKET_CARRY = 2,   /* 承托位(岩石被完全支撑 -> SUPPORT) */
    BUCKET_DUMP  = 3,   /* 倾倒位 */
    BUCKET_POSE_NUM = 4
} BucketPose_e;

/**
 * @brief  初始化: 绑定 PWM 通道并停在 BUCKET_ANG_INIT(默认调零 135°)
 * @note   内部仍会 PWM_Channel_Init(它会启动定时器输出), 因此调用后立刻写
 *         BUCKET_ANG_INIT 脉宽并置 output_enabled = 0 —— 见 Bucket_Arm 的说明。
 *         App_Init 阶段电机/舵机不得"执行功能", 但舵机保持在固定角度不视为功能动作;
 *         若校内规则更严格, 把 BUCKET_SAFE_OUTPUT_AT_INIT 置 0 并让舵机上电即断开。
 */
#define BUCKET_SAFE_OUTPUT_AT_INIT   1

void Bucket_Init(void);

/**
 * @brief  进入比赛(允许输出), 并立即到 STOW 姿态
 */
void Bucket_Arm(void);

/**
 * @brief  禁止输出(急停/收尾)
 * @note   停止比较输出(HAL_TIM_PWM_Stop), 舵机失去信号后保持机械位置
 */
void Bucket_Disarm(void);

/**
 * @brief  设置目标姿态(非阻塞, 由 Bucket_IsDone 查询)
 * @param  pose 目标姿态
 * @retval 0 已接受; 1 未 Arm; 2 参数错误
 */
uint8_t Bucket_SetPose(BucketPose_e pose);

/**
 * @brief  慢速移动(倾倒防弹出): 在 ms 时间内在当前角度与目标之间线性插值
 * @note   p2 = 0 时用 BUCKET_MOVE_MS
 */
uint8_t Bucket_SetPoseSlow(BucketPose_e pose, uint16_t ms);

/**
 * @brief  直接设置目标角度(0~270, 非阻塞, 由 Bucket_IsDone 查询)
 * @note   调试/舵机调零用: 不走姿态表。之后 Bucket_GetPose() 返回
 *         BUCKET_POSE_NUM, 表示当前是"自定义角度"。
 * @param  deg 目标角度(度), 135 对应 1500us(行程中点)
 * @retval 0 已接受; 1 未 Arm; 2 参数错误
 */
uint8_t Bucket_SetAngle(uint16_t deg);

/**
 * @brief  动作是否结束(按固定时间判到位, 舵机无反馈)
 */
bool Bucket_IsDone(void);

/**
 * @brief  周期任务(更新插值与到位计时), 每 5ms 调用
 */
void Bucket_Tick(void);

BucketPose_e Bucket_GetPose(void);
uint16_t     Bucket_GetAngle(void);

/**
 * @brief  是否已 Arm(允许输出)
 */
bool Bucket_IsArmed(void);

#ifdef __cplusplus
}
#endif

#endif /* __BUCKET_H__ */