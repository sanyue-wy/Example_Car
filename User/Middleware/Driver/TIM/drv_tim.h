/**
 * @file    drv_tim.h
 * @brief   面向对象风格的定时中断驱动(接口层)
 * @version 1.0
 * @date    2026-09-26
 */

#ifndef __DRIVER_TIM_H__
#define __DRIVER_TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* Exported macros ------------------------------------------------------------*/

/* ---- 系统 1ms 节拍绑定(换芯片/换工程只改这里) ---------------------------
 * 不再写死 F4 才有的 TIM7: 默认绑定本工程 CubeMX 已配置的 TIM4(1kHz)。
 * tim7_timer / Driver_TIM7_Init 是历史命名, 保留仅为兼容 task_it.c 与调试层;
 * 实际定时器由下面两个宏决定, 句柄需已在 tim.h 中声明并完成 MX_xxx_Init。
 * 注意: 节拍定时器不能同时充当 HAL 时间基(本工程 HAL 时间基是 SysTick)。
 */
#ifndef DRV_TIM_TICK_HANDLE
#define DRV_TIM_TICK_HANDLE   (&htim4)
#endif
#ifndef DRV_TIM_TICK_NAME
#define DRV_TIM_TICK_NAME     "TIM4"
#endif

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 建模思路与 drv_can 一致: 一个定时器 = 一个 TIM_TimerObject 对象。
  * 数据(句柄/时间基/订阅表/统计) + 行为(节拍服务回调) 封装在一起,
  * HAL 中断回调只按句柄查对象再分发, 不写 if (Instance == TIMx) 分支链。
  *
  * 可迁移性体现在两处:
  * 1. 时间基不依赖 CubeMX 里写死的 PSC/ARR —— Init 按 period_us 与运行时
  *    读到的实际定时器时钟重算, 换主频/换板卡/换定时器都不用改驱动;
  * 2. 业务侧只需要 void(void) 的函数即可挂入节拍(CAN_Bus_Tick /
  *    UART_Port_Tick / 任务函数均可直接订阅), 与具体定时器解耦。
  ******************************************************************************
  */

#define TIM_TIMER_MAX_SERVICES 8U /* 每个定时器最多挂 8 个节拍服务 */

typedef struct TIM_TimerObject TIM_TimerObject;

/**
  * @brief  节拍服务回调: 无参无返回, 业务函数可直接挂入订阅表
  */
typedef void (*TIM_TimerService_t)(void);

/**
  * @brief  对象级回调(对象的"虚函数"): 每次更新中断触发
  * @param  timer 触发中断的定时器对象
  */
typedef void (*TIM_TimerCallback_t)(TIM_TimerObject *timer);

/**
  * @brief  节拍服务订阅项: 按 period 分频由更新中断调用(观察者模式)
  */
typedef struct
{
  TIM_TimerService_t fn;   /* 服务函数, 空槽为 NULL */
  uint16_t           period;  /* 分频系数: 每 period 个节拍调用一次, 0 视为 1 */
  uint16_t           counter; /* 内部节拍计数, 业务层不要修改 */
} TIM_TimerSubscription_t;

/**
  * @brief  定时器运行统计(中断上下文写入, 主循环读取)
  */
typedef struct
{
  volatile uint32_t irq_count; /* 更新中断次数(1ms 节拍下即毫秒计数) */
  volatile uint32_t err_count; /* HAL_TIM_ErrorCallback 触发次数 */
} TIM_TimerStats_t;

/**
  * @brief  定时器对象
  */
struct TIM_TimerObject
{
  const char              *name;      /* 调试用名字, 如 "TIM4" */
  TIM_HandleTypeDef       *htim;      /* 底层 HAL 句柄 */
  uint32_t                 period_us; /* 时间基周期(微秒), Init 时据此重算 PSC/ARR */
  TIM_TimerCallback_t      on_tick;   /* 对象级回调, 可为 NULL */
  TIM_TimerSubscription_t  subs[TIM_TIMER_MAX_SERVICES]; /* 节拍服务订阅表 */
  uint8_t                  sub_count; /* 订阅表已用长度 */
  TIM_TimerStats_t         stats;     /* 运行统计 */
};

/* Exported variables ---------------------------------------------------------*/

/* 系统 1ms 节拍对象(历史命名 tim7_timer, 实际定时器见 DRV_TIM_TICK_*) */
extern TIM_TimerObject tim7_timer;

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  系统节拍接入入口: 绑定 DRV_TIM_TICK_HANDLE 与 tim7_timer, 启动 1ms 节拍
  * @note   必须在对应 MX_TIMx_Init 之后调用(本工程为 TIM4);
  *         节拍服务用 TIM_Timer_Subscribe 挂到 tim7_timer 上(见 task_it.c)
  */
void Driver_TIM7_Init(void);

uint8_t  TIM_Timer_Init(TIM_TimerObject *timer, TIM_HandleTypeDef *htim, const char *name,
                        uint32_t period_us, TIM_TimerCallback_t on_tick);
uint8_t  TIM_Timer_Subscribe(TIM_TimerObject *timer, TIM_TimerService_t fn, uint16_t period_ticks);
uint32_t TIM_Timer_GetIrqCount(TIM_TimerObject *timer);

/* 时基计算工具(供 drv_pwm 等共用, 与具体主频解耦) */
uint32_t TIM_Timer_GetClockHz(TIM_TypeDef *instance);
uint8_t  TIM_Timer_CalcTimebase(uint32_t clock_hz, uint32_t period_us, uint16_t *psc, uint16_t *arr);
uint8_t  TIM_Timer_CalcFreq(uint32_t clock_hz, uint32_t freq_hz, uint16_t *psc, uint16_t *arr);

/**
 * @brief  获取系统运行毫秒数 (硬件/系统时间基准抽象)
 */
uint32_t SysTime_GetMs(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_TIM_H__ */
