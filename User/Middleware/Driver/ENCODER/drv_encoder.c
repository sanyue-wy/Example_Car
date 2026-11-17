/**
 * @file    drv_encoder.c
 * @brief   面向对象风格的增量式编码器输入驱动(实现层)
 * @version 1.0
 * @date    2026-11-17
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_encoder.h"

/* Private macros ------------------------------------------------------------*/

#define ENCODER_MAX_INSTANCES 8U /* 按实际用到的编码器数增删 */

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 与 drv_pwm 同构, 并做了如下取舍:
  * 1. 一个编码器封装为一个 ENCODER_Object 对象; 业务层只调 Update / GetCount,
  *    不接触 CNT 寄存器, 也不需要自己处理 16 位回绕;
  * 2. 编码器硬件配置沿用 CubeMX(HAL_TIM_Encoder_Init 已配好 TI1/TI2/TI12),
  *    驱动 Init 只校验 SMS 是否处于编码器模式并启动接口, 不重算滤波器/极性,
  *    换板卡时只要 CubeMX 里仍选 Encoder_Interface 即可;
  * 3. 计数采用"符号化差分 + 累加": 每次 Update 把 CNT 与上次值做模
  *    period_ticks 的差, 超过半量程按反向处理, 再累加到 32 位总计数,
  *    彻底避开 16 位回绕导致的跳变;
  * 4. 静态对象 + 固定容量注册表, 一个定时器只允许绑定一个对象,
  *    避免同一计数器被两个业务模块各自解读。
  ******************************************************************************
  */

/* 对象注册表(静态分配, 无堆操作) */
static ENCODER_Object *s_encoder_registry[ENCODER_MAX_INSTANCES];
static uint8_t s_encoder_count = 0U;

/* Private function declarations ---------------------------------------------*/

static ENCODER_Object *ENCODER_FindBySelf(ENCODER_Object *enc);
static ENCODER_Object *ENCODER_FindByHandle(TIM_HandleTypeDef *htim);
static int32_t ENCODER_Sample(ENCODER_Object *enc, uint32_t now);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  在注册表中查找对象本身
  * @param  enc 对象指针
  * @retval 已登记返回该对象, 未登记返回 NULL
  */
static ENCODER_Object *ENCODER_FindBySelf(ENCODER_Object *enc)
{
  for (uint8_t i = 0U; i < s_encoder_count; i++)
  {
    if (s_encoder_registry[i] == enc)
    {
      return s_encoder_registry[i];
    }
  }
  return NULL;
}

/**
  * @brief  按 HAL 句柄查找编码器对象(占用检查用)
  * @param  htim HAL 句柄
  * @retval 对象指针, 未登记返回 NULL
  */
static ENCODER_Object *ENCODER_FindByHandle(TIM_HandleTypeDef *htim)
{
  for (uint8_t i = 0U; i < s_encoder_count; i++)
  {
    if (s_encoder_registry[i]->htim == htim)
    {
      return s_encoder_registry[i];
    }
  }
  return NULL;
}

/**
  * @brief  对一次 CNT 采样做回绕展开并累加
  * @param  enc 编码器对象
  * @param  now 本次读到的原始计数值
  * @retval 本次有符号增量(已按 invert 取反)
  * @note   period_ticks == 0 视为 32 位计数器满量程, 直接按 uint32 自然回绕
  *         求差; 其余按模运算求差, 差值超过半量程即判为反向。
  */
static int32_t ENCODER_Sample(ENCODER_Object *enc, uint32_t now)
{
  uint32_t span = enc->period_ticks;
  int32_t delta;

  if (span == 0U)
  {
    /* 32 位满量程: uint32 减法本身已含回绕 */
    delta = (int32_t)(now - enc->raw_count);
  }
  else
  {
    uint32_t diff = (now >= enc->raw_count) ? (now - enc->raw_count)
                                            : (span - enc->raw_count + now);

    if (diff > (span >> 1U))
    {
      delta = (int32_t)diff - (int32_t)span; /* 反向转过的大半个周期 */
    }
    else
    {
      delta = (int32_t)diff;
    }
  }

  enc->raw_count = now;
  if (enc->invert != 0U)
  {
    delta = -delta;
  }
  enc->delta = delta;
  enc->total_count += delta;

  return delta;
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  初始化并接入一个编码器
  * @param  enc            编码器对象实例, 由调用方静态分配
  * @param  htim           底层 HAL 句柄, 需已完成 MX_xxx_Init(编码器模式)
  * @param  name           调试用名字
  * @param  counts_per_rev 输出轴一圈计数值(含倍频/减速比), 0 表示未知
  * @param  invert         方向取反: 0 正常, 1 取反
  * @retval 0 成功; 1 参数错误(含 ARR=0); 2 定时器被其它对象占用/注册表已满/
  *         对象已绑定其它定时器; 3 定时器不是编码器模式;
  *         4 启动编码器接口失败
  * @note   重复 Init 同一对象视为原位重新绑定: 计数与统计全部清零;
  *         已在运行的编码器接口不会重复启动。
  */
uint8_t ENCODER_Init(ENCODER_Object *enc, TIM_HandleTypeDef *htim, const char *name,
                     uint32_t counts_per_rev, uint8_t invert)
{
  ENCODER_Object *self;
  uint32_t sms;

  if (enc == NULL || htim == NULL || name == NULL || invert > 1U || htim->Init.Period == 0U)
  {
    return 1;
  }

  /* SMS[2:0]: 001 = TI1(x2), 010 = TI2(x2), 011 = TI12(x4), 其余不是编码器模式 */
  sms = htim->Instance->SMCR & TIM_SMCR_SMS;
  if (sms != TIM_ENCODERMODE_TI1 && sms != TIM_ENCODERMODE_TI2 && sms != TIM_ENCODERMODE_TI12)
  {
    return 3;
  }

  /* 注册表检查: 对象换定时器不允许; 定时器换对象也不允许 */
  self = ENCODER_FindBySelf(enc);
  if (self != NULL)
  {
    if (self->htim != htim)
    {
      return 2;
    }
  }
  else
  {
    if (ENCODER_FindByHandle(htim) != NULL)
    {
      return 2;
    }
    if (s_encoder_count >= ENCODER_MAX_INSTANCES)
    {
      return 2;
    }
  }

  /* 填充对象 */
  enc->name = name;
  enc->htim = htim;
  enc->period_ticks = htim->Init.Period + 1U; /* ARR = 0xFFFFFFFF 时自然溢出为 0 */
  enc->counts_per_rev = counts_per_rev;
  enc->invert = invert;
  enc->raw_count = 0U;
  enc->total_count = 0;
  enc->delta = 0;
  enc->stats.update_count = 0U;
  enc->stats.reset_count = 0U;

  /* 启动编码器接口: CC1/CC2 使能 + 计数器使能。
     若接口已由外部启动(HAL 会因通道 BUSY 返回 HAL_ERROR), 以 CEN 位为准放行 */
  if (HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL) != HAL_OK)
  {
    if ((htim->Instance->CR1 & TIM_CR1_CEN) == 0U)
    {
      return 4;
    }
  }

  /* 登记: 重复 Init 原位更新, 否则追加 */
  if (self == NULL)
  {
    s_encoder_registry[s_encoder_count] = enc;
    s_encoder_count++;
  }

  /* 计数基准归零, 保证 total_count 与 CNT 同一起点 */
  __HAL_TIM_SET_COUNTER(htim, 0U);
  enc->raw_count = 0U;

  return 0;
}

/**
  * @brief  采样一次编码器并更新累计计数/增量
  * @param  enc 编码器对象
  * @note   建议挂在 1~10ms 的周期任务里; 两次调用的计数变化必须小于半个
  *         量程(16 位定时器即 <32768), 否则方向会判反
  */
void ENCODER_Update(ENCODER_Object *enc)
{
  if (enc == NULL || enc->htim == NULL)
  {
    return;
  }

  (void)ENCODER_Sample(enc, __HAL_TIM_GET_COUNTER(enc->htim));
  enc->stats.update_count++;
}

/**
  * @brief  读取累计计数
  * @param  enc 编码器对象
  * @retval 自 Init/Reset 以来的有符号累计计数
  * @note   返回的是最近一次 Update 采样的结果, 不是硬件实时值
  */
int32_t ENCODER_GetCount(const ENCODER_Object *enc)
{
  if (enc == NULL)
  {
    return 0;
  }
  return enc->total_count;
}

/**
  * @brief  读取最近一次采样的增量
  * @param  enc 编码器对象
  * @retval 上一次 Update 与再上一次 Update 之间的有符号计数差
  * @note   适合速度环按固定周期读取: rpm = delta × 60 / (counts_per_rev × dt)
  */
int32_t ENCODER_GetDelta(const ENCODER_Object *enc)
{
  if (enc == NULL)
  {
    return 0;
  }
  return enc->delta;
}

/**
  * @brief  读取最近一次采样到的原始 CNT 值
  * @param  enc 编码器对象
  * @retval 原始计数值(0 ~ period_ticks-1), 调试用
  */
uint32_t ENCODER_GetRawCount(const ENCODER_Object *enc)
{
  if (enc == NULL)
  {
    return 0U;
  }
  return enc->raw_count;
}

/**
  * @brief  清零累计计数与硬件计数器
  * @param  enc 编码器对象
  * @note   清零瞬间正在进入的边沿可能丢失, 属正常现象
  */
void ENCODER_Reset(ENCODER_Object *enc)
{
  if (enc == NULL || enc->htim == NULL)
  {
    return;
  }

  __HAL_TIM_SET_COUNTER(enc->htim, 0U);
  enc->raw_count = 0U;
  enc->total_count = 0;
  enc->delta = 0;
  enc->stats.reset_count++;
}

/**
  * @brief  设置方向取反
  * @param  enc    编码器对象
  * @param  invert 0 正常, 1 取反
  * @note   只影响后续采样; 若要求历史累计计数也统一方向, 请随后调用
  *         ENCODER_Reset
  */
void ENCODER_SetInvert(ENCODER_Object *enc, uint8_t invert)
{
  if (enc == NULL)
  {
    return;
  }
  enc->invert = (invert != 0U) ? 1U : 0U;
}

/**
  * @brief  累计计数换算为输出轴圈数
  * @param  enc 编码器对象
  * @retval 圈数(带符号); counts_per_rev 为 0 时返回 0
  */
float ENCODER_GetRevolutions(const ENCODER_Object *enc)
{
  if (enc == NULL || enc->counts_per_rev == 0U)
  {
    return 0.0f;
  }
  return (float)enc->total_count / (float)enc->counts_per_rev;
}

/**
  * @brief  用最近一次增量换算输出轴转速
  * @param  enc   编码器对象
  * @param  dt_ms 两次 Update 的间隔(毫秒)
  * @retval 转速(转/秒, 带符号); counts_per_rev 或 dt_ms 为 0 时返回 0
  */
float ENCODER_GetSpeedRps(const ENCODER_Object *enc, uint32_t dt_ms)
{
  if (enc == NULL || enc->counts_per_rev == 0U || dt_ms == 0U)
  {
    return 0.0f;
  }
  return (float)enc->delta * 1000.0f / ((float)enc->counts_per_rev * (float)dt_ms);
}