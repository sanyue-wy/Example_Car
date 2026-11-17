/**
 * @file    drv_encoder.h
 * @brief   面向对象风格的增量式编码器输入驱动(接口层)
 * @version 1.0
 * @date    2026-11-17
 */

#ifndef __DRIVER_ENCODER_H__
#define __DRIVER_ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 建模思路与 drv_pwm 一致: 一个编码器 = 一个 ENCODER_Object 对象。
  * 数据(句柄/计数/方向/统计) + 行为(采样、清零、换算) 封装在一起,
  * 业务层不再直接读 TIMx->CNT, 也不关心计数器回绕与方向处理。
  *
  * 硬件约束(无法封装掉, 迁移时注意):
  * 1. 定时器需在 CubeMX 中配置为 Encoder_Interface(本工程为 TIM2,
  *    PA0=TIM2_CH1, PA1=TIM2_CH2, TI12 四倍频); 驱动只校验 SMS 模式,
  *    不改写编码器配置 —— 换定时器只需换句柄;
  * 2. F103 上 TIM2~TIM4 为 16 位计数器, CNT 在 ARR+1 处回绕; 驱动按
  *    "跨越半周期视为反向"做符号化差分, 因此两次 ENCODER_Update 之间的
  *    计数变化必须小于半个量程, 否则方向会判反(采样周期别太长);
  * 3. counts_per_rev 是"输出轴转一圈的计数值", 已含倍频(TI12=x4)与减速比;
  *    填 0 表示未知 —— 此时只做纯计数, 换算接口返回 0;
  * 4. 编码器是纯输入外设, 本驱动不占用中断; ENCODER_Update 应挂在
  *    周期任务(drv_tim 节拍服务)或控制循环中调用;
  * 5. 同一对象不要在中断与主循环中并发调用 ENCODER_Update
  *    (读 CNT + 累加不是原子操作), 推荐固定在一个上下文里采样。
  ******************************************************************************
  */

/**
  * @brief  编码器运行统计
  */
typedef struct
{
  volatile uint32_t update_count; /* 采样(Update)次数 */
  volatile uint32_t reset_count;  /* 清零(Reset)次数 */
} ENCODER_Stats_t;

typedef struct ENCODER_Object ENCODER_Object;

/**
  * @brief  编码器对象
  */
struct ENCODER_Object
{
  const char        *name;           /* 调试用名字, 如 "TIM2_ENC" */
  TIM_HandleTypeDef *htim;           /* 底层 HAL 句柄 */
  uint32_t           period_ticks;   /* 一个回绕周期 = ARR + 1; 0 表示 32 位满量程 */
  uint32_t           counts_per_rev; /* 输出轴一圈计数值(含倍频/减速比), 0 表示未知 */
  uint32_t           raw_count;      /* 最近一次采样到的原始 CNT 值 */
  int32_t            total_count;    /* 累计有符号计数, 已跨回绕展开 */
  int32_t            delta;          /* 最近一次采样的有符号增量 */
  uint8_t            invert;         /* 方向取反: 0 正常, 1 取反 */
  ENCODER_Stats_t    stats;          /* 运行统计 */
};

/* Exported functions ---------------------------------------------------------*/

uint8_t ENCODER_Init(ENCODER_Object *enc, TIM_HandleTypeDef *htim, const char *name,
                     uint32_t counts_per_rev, uint8_t invert);
void    ENCODER_Update(ENCODER_Object *enc);
int32_t ENCODER_GetCount(const ENCODER_Object *enc);
int32_t ENCODER_GetDelta(const ENCODER_Object *enc);
uint32_t ENCODER_GetRawCount(const ENCODER_Object *enc);
void    ENCODER_Reset(ENCODER_Object *enc);
void    ENCODER_SetInvert(ENCODER_Object *enc, uint8_t invert);
float   ENCODER_GetRevolutions(const ENCODER_Object *enc);
float   ENCODER_GetSpeedRps(const ENCODER_Object *enc, uint32_t dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_ENCODER_H__ */