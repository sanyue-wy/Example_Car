/**
 * @file    drv_tim.c
 * @brief   面向对象风格的定时中断驱动(实现层)
 * @version 1.0
 * @date    2026-09-26
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_tim.h"

/* Private macros ------------------------------------------------------------*/

#define TIM_TIMER_MAX_INSTANCES 8U /* 按实际用到的定时器增删 */

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 与 drv_can 同构, 并做了如下取舍:
  * 1. 一个定时器封装为一个 TIM_TimerObject 对象; 中断分发按"句柄 -> 对象 ->
  *    回调/服务"完成, 不写 if (Instance == TIMx) 分支链;
  * 2. 时间基不沿用 CubeMX 写死的 PSC/ARR: 旧工程的 168 分频抄自 168MHz 时钟树,
  *    在当前 72MHz 主频下节拍根本不是 1ms。Init 按 period_us 与运行时实测的
  *    定时器时钟重算, 换板卡/换时钟/换定时器都不需要动驱动 —— 可迁移性核心;
  * 3. 节拍服务订阅表 + 分频(period_ticks/counter 方案),
  *    业务侧只需要 void(void) 函数即可挂入, 与具体定时器解耦;
  * 4. 静态对象 + 固定容量注册表, 且"先登记、后开中断",
  *    保证中断触发时对象必然已就绪。
  ******************************************************************************
  */

/* 对象注册表(静态分配, 无堆操作) */
static TIM_TimerObject *s_timer_registry[TIM_TIMER_MAX_INSTANCES];
static uint8_t s_timer_count = 0;

/* 系统 1ms 节拍对象: 历史命名 tim7_timer, 实际定时器见 DRV_TIM_TICK_HANDLE */
TIM_TimerObject tim7_timer;

/* Private function declarations ---------------------------------------------*/

static TIM_TimerObject *TIM_Timer_FindByHandle(TIM_HandleTypeDef *htim);
static uint8_t TIM_Timer_CalcPscArr(uint64_t ticks, uint16_t *psc, uint16_t *arr);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  按 HAL 句柄查找定时器对象(中断分发用)
  * @param  htim HAL 句柄
  * @retval 对象指针, 未登记返回 NULL
  */
static TIM_TimerObject *TIM_Timer_FindByHandle(TIM_HandleTypeDef *htim)
{
  for (uint8_t i = 0; i < s_timer_count; i++)
  {
    if (s_timer_registry[i]->htim == htim)
    {
      return s_timer_registry[i];
    }
  }
  return NULL;
}

/**
  * @brief  把一个周期的计数总数拆成 (psc+1)*(arr+1), 两个因子都装进 16 位
  * @param  ticks 周期对应的定时器时钟计数总数
  * @param  psc   输出预分频寄存器值(装载值 = psc+1)
  * @param  arr   输出自动重装载值(装载值 = arr+1)
  * @retval 0 成功; 1 参数错误或无法拆分
  * @note   优先取最小 psc, 拆不尽时 arr 向下取整, 误差不超过 psc+1 个计数
  */
static uint8_t TIM_Timer_CalcPscArr(uint64_t ticks, uint16_t *psc, uint16_t *arr)
{
  uint32_t p;
  uint32_t a;

  if (psc == NULL || arr == NULL || ticks < 2U)
  {
    return 1;
  }

  if (ticks <= 65536U)
  {
    p = 0U;
    a = (uint32_t)ticks - 1U;
  }
  else
  {
    /* p+1 = ceil(ticks/65536), 保证 arr+1 = ticks/(p+1) <= 65536 */
    p = (uint32_t)((ticks + 65535U) / 65536U) - 1U;
    if (p > 65535U)
    {
      return 1;
    }
    a = (uint32_t)(ticks / (p + 1U)) - 1U;
  }

  *psc = (uint16_t)p;
  *arr = (uint16_t)a;
  return 0;
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  系统节拍接入入口: 绑定 DRV_TIM_TICK_HANDLE 与 tim7_timer, 启动 1ms 节拍
  * @note   必须在对应 MX_TIMx_Init 之后调用(本工程默认 TIM4);
  *         节拍服务(task_it.c 中的各任务 / CAN_Bus_Tick / UART_Port_Tick)
  *         用 TIM_Timer_Subscribe 挂到 tim7_timer 上
  * @retval 无(失败进入 Error_Handler)
  */
void Driver_TIM7_Init(void)
{
  /* 一次完成 登记 -> 重算 1ms 时间基 -> 开更新中断。
     定时器句柄/名字由 drv_tim.h 的宏指定, 不再写死 F4 才有的 TIM7 */
  if (TIM_Timer_Init(&tim7_timer, DRV_TIM_TICK_HANDLE, DRV_TIM_TICK_NAME, 1000U, NULL) != 0)
  {
    Error_Handler();
  }
}

/**
  * @brief  初始化并接入一个定时器
  * @param  timer     定时器对象实例, 由调用方静态分配
  * @param  htim      底层 HAL 句柄, 需已完成 MX_xxx_Init
  * @param  name      调试用名字
  * @param  period_us 更新中断周期(微秒); 驱动按实际定时器时钟重算 PSC/ARR,
  *                   不依赖 CubeMX 里配置的分频值
  * @param  on_tick   对象级回调, 可为 NULL
  * @retval 0 成功; 1 参数错误; 2 注册表已满; 3 时间基计算失败(时钟未就绪/
  *         周期超出定时器范围); 4 应用时间基失败; 5 启动中断失败
  * @note   先登记后开中断, 中断一旦触发回调就能拿到完整对象
  */
uint8_t TIM_Timer_Init(TIM_TimerObject *timer, TIM_HandleTypeDef *htim, const char *name,
                       uint32_t period_us, TIM_TimerCallback_t on_tick)
{
  uint8_t i;
  uint32_t clock_hz;
  uint16_t psc;
  uint16_t arr;

  if (timer == NULL || htim == NULL || period_us == 0U)
  {
    return 1;
  }

  /* 按实际定时器时钟重算时间基(可迁移性的关键: 与主频/板卡解耦) */
  clock_hz = TIM_Timer_GetClockHz(htim->Instance);
  if (clock_hz == 0U || TIM_Timer_CalcTimebase(clock_hz, period_us, &psc, &arr) != 0)
  {
    return 3;
  }

  /* 填充对象 */
  timer->name = name;
  timer->htim = htim;
  timer->period_us = period_us;
  timer->on_tick = on_tick;
  timer->sub_count = 0;
  timer->stats.irq_count = 0;
  timer->stats.err_count = 0;

  /* 登记: 重复 Init 则原位更新, 否则追加 */
  for (i = 0; i < s_timer_count; i++)
  {
    if (s_timer_registry[i] == timer)
    {
      break;
    }
  }
  if (i == s_timer_count)
  {
    if (s_timer_count >= TIM_TIMER_MAX_INSTANCES)
    {
      return 2;
    }
    s_timer_registry[s_timer_count] = timer;
    s_timer_count++;
  }

  /* 应用时间基: HAL_TIM_Base_Init 内部走 TIM_Base_SetConfig 落寄存器,
     重复调用幂等(CubeMX 已 Init 过时只刷时间基, 不重复走 MspInit) */
  htim->Init.Prescaler = psc;
  htim->Init.Period = arr;
  if (HAL_TIM_Base_Init(htim) != HAL_OK)
  {
    return 4;
  }

  /* 启动外设并打开更新中断 —— 必须放在登记之后 */
  __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start_IT(htim) != HAL_OK)
  {
    return 5;
  }

  return 0;
}

/**
  * @brief  订阅一个节拍服务(观察者注册)
  * @param  timer       定时器对象
  * @param  fn          节拍服务函数(void(void)), 如 CAN_Bus_Tick; 不可为 NULL
  * @param  period_ticks 分频系数: 每 period_ticks 个节拍调用一次, 0 视为 1
  * @retval 0 成功; 1 参数错误; 2 订阅表已满
  * @note   同一函数重复订阅视为更新分频; 中断上下文执行, 回调应尽快返回
  */
uint8_t TIM_Timer_Subscribe(TIM_TimerObject *timer, TIM_TimerService_t fn, uint16_t period_ticks)
{
  uint8_t i;
  uint8_t empty_slot = 0xFFU;

  if (timer == NULL || fn == NULL)
  {
    return 1;
  }

  for (i = 0; i < timer->sub_count; i++)
  {
    if (timer->subs[i].fn == fn)
    {
      timer->subs[i].period = period_ticks; /* 更新分频 */
      timer->subs[i].counter = 0;
      return 0;
    }
    if (timer->subs[i].fn == NULL && empty_slot == 0xFFU)
    {
      empty_slot = i; /* 复用注销留下的空槽 */
    }
  }

  if (empty_slot == 0xFFU)
  {
    if (timer->sub_count >= TIM_TIMER_MAX_SERVICES)
    {
      return 2;
    }
    empty_slot = timer->sub_count;
    timer->sub_count++;
  }

  timer->subs[empty_slot].fn = fn;
  timer->subs[empty_slot].period = period_ticks;
  timer->subs[empty_slot].counter = 0;
  return 0;
}

/**
  * @brief  读取更新中断累计次数
  * @retval 中断次数; 1ms 节拍下即毫秒计数, 可替代 HAL_GetTick 做轻量计时
  */
uint32_t TIM_Timer_GetIrqCount(TIM_TimerObject *timer)
{
  return (timer == NULL) ? 0U : timer->stats.irq_count;
}

/**
  * @brief  获取某定时器的实际输入时钟
  * @param  instance 定时器外设, 如 TIM2
  * @retval 时钟频率(Hz); 0 表示系统时钟未就绪
  * @note   APB 预分频 != 1 时定时器时钟 = 2 x PCLK(STM32 通用规则)
  */
uint32_t TIM_Timer_GetClockHz(TIM_TypeDef *instance)
{
  uint32_t pclk;
  uint8_t on_apb2 = 0U;

  /* APB2 定时器: 各系列不全是同一组, 用编译期宏过滤掉本芯片不存在的实例,
     避免在 F103 上引用 F4 才有的 TIM8~TIM11 直接编译报错 */
  if (instance == TIM1)
  {
    on_apb2 = 1U;
  }
#if defined(TIM8)
  else if (instance == TIM8)
  {
    on_apb2 = 1U;
  }
#endif
#if defined(TIM9)
  else if (instance == TIM9)
  {
    on_apb2 = 1U;
  }
#endif
#if defined(TIM10)
  else if (instance == TIM10)
  {
    on_apb2 = 1U;
  }
#endif
#if defined(TIM11)
  else if (instance == TIM11)
  {
    on_apb2 = 1U;
  }
#endif
#if defined(TIM15)
  else if (instance == TIM15)
  {
    on_apb2 = 1U;
  }
#endif
#if defined(TIM16)
  else if (instance == TIM16)
  {
    on_apb2 = 1U;
  }
#endif
#if defined(TIM17)
  else if (instance == TIM17)
  {
    on_apb2 = 1U;
  }
#endif

  pclk = (on_apb2 != 0U) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();

  if (pclk == 0U)
  {
    return 0U;
  }
  return (HAL_RCC_GetHCLKFreq() == pclk) ? pclk : (pclk * 2U);
}

/**
  * @brief  按中断周期计算时间基 PSC/ARR
  * @param  clock_hz  定时器输入时钟(Hz), 用 TIM_Timer_GetClockHz 获取
  * @param  period_us 中断周期(微秒)
  * @param  psc       输出预分频寄存器值
  * @param  arr       输出自动重装载值
  * @retval 0 成功; 1 参数错误或周期超出定时器范围
  */
uint8_t TIM_Timer_CalcTimebase(uint32_t clock_hz, uint32_t period_us, uint16_t *psc, uint16_t *arr)
{
  uint64_t ticks;

  if (clock_hz == 0U || period_us == 0U)
  {
    return 1;
  }

  /* ticks = clock * period_us / 1e6, 四舍五入 */
  ticks = ((uint64_t)clock_hz * period_us + 500000U) / 1000000U;
  return TIM_Timer_CalcPscArr(ticks, psc, arr);
}

/**
  * @brief  按 PWM 频率计算时间基 PSC/ARR(供 drv_pwm 共用)
  * @param  clock_hz 定时器输入时钟(Hz)
  * @param  freq_hz  目标频率(Hz)
  * @param  psc      输出预分频寄存器值
  * @param  arr      输出自动重装载值
  * @retval 0 成功; 1 参数错误或频率超出定时器范围
  */
uint8_t TIM_Timer_CalcFreq(uint32_t clock_hz, uint32_t freq_hz, uint16_t *psc, uint16_t *arr)
{
  uint64_t ticks;

  if (clock_hz == 0U || freq_hz == 0U || freq_hz > clock_hz)
  {
    return 1;
  }

  /* ticks = clock / freq, 四舍五入 */
  ticks = ((uint64_t)clock_hz + freq_hz / 2U) / freq_hz;
  return TIM_Timer_CalcPscArr(ticks, psc, arr);
}

/**
  * @brief  HAL 库定时器更新中断回调(覆盖 HAL 弱定义)
  * @note   按句柄查对象分发: ① 订阅表按分频逐个调用 ② 对象级回调
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* 不再把 TIM4 固定当作 HAL 时间基(F4 旧工程遗留):
     本工程 HAL 时间基是 SysTick, 节拍定时器(默认 TIM4)必须正常走下面的分发 */
  TIM_TimerObject *timer = TIM_Timer_FindByHandle(htim);

  if (timer == NULL)
  {
    return;
  }

  timer->stats.irq_count++;

  /* ① 节拍服务按分频分发 */
  for (uint8_t i = 0; i < timer->sub_count; i++)
  {
    TIM_TimerSubscription_t *sub = &timer->subs[i];
    uint16_t period = (sub->period == 0U) ? 1U : sub->period;

    if (sub->fn == NULL || ++sub->counter < period)
    {
      continue;
    }
    sub->counter = 0;
    sub->fn();
  }

  /* ② 对象级回调兜底 */
  if (timer->on_tick != NULL)
  {
    timer->on_tick(timer);
  }
}

/**
  * @brief  HAL 库定时器错误回调(覆盖 HAL 弱定义)
  * @note   触发前提是使能了相应错误/触发中断, 目前仅做计数
  */
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
  TIM_TimerObject *timer = TIM_Timer_FindByHandle(htim);

  if (timer != NULL)
  {
    timer->stats.err_count++;
  }
}

uint32_t SysTime_GetMs(void)
{
  return HAL_GetTick();
}
