/**
 * @file    task_it.h
 * @brief   APP 层周期任务注册(全部挂在 drv_tim 的 1ms 节拍上)
 */

#ifndef __TASK_IT_H__
#define __TASK_IT_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  统一注册周期任务
 * @note   在 MX_TIM4_Init / 串口初始化之后调用(见 App_Init / debug_init)
 *         订阅表容量 TIM_TIMER_MAX_SERVICES = 8, 当前占用 3 项。
 */
void Task_IT_Init(void);

/**
 * @brief  1ms 节拍服务(系统级轻量任务)
 */
void MM_TIM_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_IT_H__ */