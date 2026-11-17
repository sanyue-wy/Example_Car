/**
 * @file    drv_pwm.h
 * @brief   面向对象风格的 PWM 输出驱动(接口层)
 * @version 1.0
 * @date    2026-09-26
 */

#ifndef __DRIVER_PWM_H__
#define __DRIVER_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 建模思路与 drv_can 一致: 一个 PWM 输出通道 = 一个 PWM_ChannelObject 对象。
  * 数据(句柄/通道/时间基/占空比) + 行为(脉宽/占空比设置) 封装在一起,
  * 对外只有 Init / SetPulse / SetDuty 三个动作:
  * - 舵机按脉宽工作, 用 SetPulse;
  * - 编码器电机按占空比调压调速, 用 SetDuty(千分比, 直接换算成周期计数,
  *   20kHz 下仍有 1/7200 的分辨率; 若按 µs 传脉宽只剩 50 级)。
  * Start/Stop 不封装: 通道 Init 后即常开, 停止由业务侧送 0 占空比完成。
  *
  * 硬件约束(无法封装掉, 迁移时注意):
  * 1. 同一定时器所有通道共用一个 PWM 频率 —— 频率在第一个通道 Init 时
  *    按实际定时器时钟重算 PSC/ARR(与 drv_tim 同一套计算, 可迁移),
  *    后续通道 Init 的频率必须与之一致, 否则报错;
  * 2. 输出引脚需在 CubeMX 中配置为对应 TIMx_CHy 复用推挽,
  *    否则波形只在定时器内部产生、不会出现在引脚上;
  * 3. 同一个定时器不要同时用作 drv_tim 的节拍定时器与 PWM 输出。
  ******************************************************************************
  */

/**
  * @brief  通道运行统计
  */
typedef struct
{
  volatile uint32_t set_count; /* 占空比/脉宽设置次数 */
} PWM_ChannelStats_t;

typedef struct PWM_ChannelObject PWM_ChannelObject;

/**
  * @brief  PWM 通道对象
  */
struct PWM_ChannelObject
{
  const char          *name;          /* 调试用名字, 如 "TIM2_CH1" */
  TIM_HandleTypeDef   *htim;          /* 底层 HAL 句柄 */
  uint32_t             channel;       /* 通道号: TIM_CHANNEL_1 ~ TIM_CHANNEL_4 */
  uint32_t             freq_hz;       /* PWM 频率(Hz), 同一定时器各通道一致 */
  uint32_t             timer_clock_hz;/* 定时器输入时钟(Init 时缓存) */
  uint32_t             period_ticks;  /* 一个周期的计数总数 = ARR + 1 */
  uint16_t             duty;          /* 当前占空比(千分比 0~1000) */
  PWM_ChannelStats_t   stats;         /* 运行统计 */
};

/* Exported functions ---------------------------------------------------------*/

uint8_t PWM_Channel_Init(PWM_ChannelObject *ch, TIM_HandleTypeDef *htim, uint32_t channel,
                         const char *name, uint32_t freq_hz, uint16_t duty_permille);
uint8_t PWM_Channel_SetPulse(PWM_ChannelObject *ch, uint32_t pulse_us);
uint8_t PWM_Channel_SetDuty(PWM_ChannelObject *ch, uint16_t duty_permille);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_PWM_H__ */
