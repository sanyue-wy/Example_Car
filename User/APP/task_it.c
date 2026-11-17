/**
 * @file    task_it.c
 * @brief   周期任务注册: 所有周期任务挂在 drv_tim 的 1ms 节拍上, 按分频调用
 *
 * @note    订阅表容量 TIM_TIMER_MAX_SERVICES = 8, 当前 3 项:
 *            1. MM_TIM_Callback   1ms  系统节拍(本工程无心跳 LED, 留空)
 *            2. Task_Control_5ms  5ms  控制环(App / debug 二选一)
 *            3. UART_Port_Tick    1ms  串口周期发送
 */

/* Includes ------------------------------------------------------------------*/
#include "task_it.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "debug.h"
#include "main.h"

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  5ms 控制环: 分发到比赛层或调试层
  */
static void Task_Control_5ms(void)
{
#if (RUN_MODE == RUN_MATCH)
    App_ControlTick();
#else
    Debug_ControlTick();
#endif
}

/* Exported functions ---------------------------------------------------------*/

void Task_IT_Init(void)
{
    /* 系统 1ms 节拍: 由 drv_tim.h 的 DRV_TIM_TICK_* 绑定(本工程 TIM4) */
    Driver_TIM7_Init();

    TIM_Timer_Subscribe(&tim7_timer, MM_TIM_Callback,   1);   /* 1ms  */
    TIM_Timer_Subscribe(&tim7_timer, Task_Control_5ms,  5);   /* 5ms  */
    TIM_Timer_Subscribe(&tim7_timer, UART_Port_Tick,    1);   /* 1ms  */
}

/**
  * @brief  1ms 系统节拍
  * @note   本工程板子没有已配置的心跳 LED 引脚(PC13 已用作"结束指示"),
  *         所以这里只留空; 若以后加心跳灯, 在这里翻转即可。
  */
void MM_TIM_Callback(void)
{
    /* 预留给系统级毫秒任务 */
}