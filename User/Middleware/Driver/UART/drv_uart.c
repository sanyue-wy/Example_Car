/**
 * @file    drv_uart.c
 * @brief   面向对象风格的 UART 驱动(实现层)
 *          参考 SCUT-Robotlab 与 USTC-RoboWalker 两份开源 UART 驱动改写
 * @version 1.0
 * @date    2026-09-25
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"
#include <stdio.h>
#include <stdarg.h>

/* Private macros ------------------------------------------------------------*/

#define UART_PORT_MAX 4U /* 按实际用到的串口数增删(本工程 F103 只有 USART1/2/3) */

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 综合两份参考实现并做了如下取舍:
  * 1. 一个串口封装为一个 UART_PortObject 对象; 中断分发按"句柄 -> 对象 -> 回调",
  *    不再像参考实现那样写 8 段 if (Instance == USARTx) 分支链;
  * 2. 参考实现一用 realloc 动态扩展回调数组(堆 + 中断路径竞态),
  *    这里改为静态对象 + 固定容量注册表, 且"先登记、后启动接收";
  * 3. 接收回调修正了参考实现二的缺陷 —— 传给业务层的是本次实际收到的
  *    Size, 而不是缓冲区总长度; 参考实现一的 vsprintf 换成有界 vsnprintf;
  * 4. 增加错误回调: 串口噪声/溢出导致接收中断后自动清理并重新武装,
  *    两份参考在一次帧错误后接收就永久停摆;
  * 5. 参考实现二的 TIM 周期发送泛化为 SetPeriodicTx + UART_Port_Tick,
  *    与 drv_can 的 CAN_Bus_Tick 共用同一个定时器节拍即可。
  ******************************************************************************
  */

/* 对象注册表(静态分配, 无堆操作) */
static UART_PortObject *s_port_registry[UART_PORT_MAX];
static uint8_t s_port_count = 0;

/* 打印用共享缓冲: UART_Port_Printf 为阻塞发送, 不可在中断里调用。
   用 PRIMASK 保护的忙标志替代 RTOS 互斥量, 裸机/RTOS 工程都能编译 */
static char s_print_buf[128];
static volatile uint8_t s_print_busy = 0U;

/* USART1 对象实例 */
static uint8_t UART1_RxBuffer[128]; /* 按协议帧长调整 */

UART_PortObject uart1_port;

/* Private function declarations ---------------------------------------------*/

static UART_PortObject *UART_Port_FindByHandle(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_Port_StartReceive(UART_PortObject *port);
static uint8_t UART_PrintLock(void);
static void UART_PrintUnlock(void);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  打印忙标志加锁(中断安全, 不依赖 RTOS)
  * @retval 1 获取成功; 0 已有其它上下文在打印
  */
static uint8_t UART_PrintLock(void)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t locked = 0U;

  __disable_irq();
  if (s_print_busy == 0U)
  {
    s_print_busy = 1U;
    locked = 1U;
  }
  __set_PRIMASK(primask);
  return locked;
}

/**
  * @brief  释放打印忙标志
  */
static void UART_PrintUnlock(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_print_busy = 0U;
  __set_PRIMASK(primask);
}

/**
  * @brief  按 HAL 句柄查找端口对象(中断分发用)
  * @retval 对象指针, 未登记返回 NULL
  */
static UART_PortObject *UART_Port_FindByHandle(UART_HandleTypeDef *huart)
{
  for (uint8_t i = 0; i < s_port_count; i++)
  {
    if (s_port_registry[i]->huart == huart)
    {
      return s_port_registry[i];
    }
  }
  return NULL;
}

/**
  * @brief  重新武装接收
  * @param  port 端口对象
  * @retval HAL 状态
  * @note   有 RX DMA 句柄就走 DMA, 否则退回中断接收(本工程 USART1 未配 DMA);
  *         两种模式都由 HAL_UARTEx_RxEventCallback 回调, 上层无感
  */
static HAL_StatusTypeDef UART_Port_StartReceive(UART_PortObject *port)
{
  if (port->huart->hdmarx != NULL)
  {
    return HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->rx_buffer, port->rx_buffer_size);
  }
  return HAL_UARTEx_ReceiveToIdle_IT(port->huart, port->rx_buffer, port->rx_buffer_size);
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  UART1 驱动接入入口: 绑定 huart1 与 uart1_port 并启动空闲中断接收
  * @note   必须在 MX_USART1_UART_Init 之后调用
  * @retval 无(失败进入 Error_Handler)
  */
void Driver_UART1_Init(void)
{
  if (UART_Port_Init(&uart1_port, &huart1, "USART1", NULL,
                     UART1_RxBuffer, (uint16_t)sizeof(UART1_RxBuffer)) != 0)
  {
    Error_Handler();
  }
}

/**
  * @brief  初始化并接入一个 UART 端口
  * @param  port           端口对象实例, 由调用方静态分配
  * @param  huart          底层 HAL 句柄, 需已完成 MX_xxx_Init
  * @param  name           调试用名字
  * @param  on_rx          接收回调, 可为 NULL(收到数据只计数)
  * @param  rx_buffer      接收缓冲区, 由调用方持有
  * @param  rx_buffer_size 接收缓冲区长度
  * @retval 0 成功; 1 参数错误; 2 注册表已满; 3 启动接收失败
  * @note   启动接收放在登记之后, 中断一旦触发回调就能拿到完整对象
  */
uint8_t UART_Port_Init(UART_PortObject *port, UART_HandleTypeDef *huart, const char *name,
                       UART_RxCallback_t on_rx, uint8_t *rx_buffer, uint16_t rx_buffer_size)
{
  uint8_t i;

  if (port == NULL || huart == NULL || rx_buffer == NULL || rx_buffer_size == 0U)
  {
    return 1;
  }

  /* 填充对象 */
  port->name = name;
  port->huart = huart;
  port->on_rx = on_rx;
  port->rx_buffer = rx_buffer;
  port->rx_buffer_size = rx_buffer_size;
  port->periodic_tx_data = NULL;
  port->periodic_tx_len = 0;
  port->stats.rx_count = 0;
  port->stats.rx_error = 0;
  port->stats.tx_count = 0;
  port->stats.tx_error = 0;
  port->stats.err_count = 0;

  /* 登记: 重复 Init 则原位更新, 否则追加 */
  for (i = 0; i < s_port_count; i++)
  {
    if (s_port_registry[i] == port)
    {
      break;
    }
  }
  if (i == s_port_count)
  {
    if (s_port_count >= UART_PORT_MAX)
    {
      return 2;
    }
    s_port_registry[s_port_count] = port;
    s_port_count++;
  }

  /* 启动接收: 配了 RX DMA 用 DMA, 否则用中断 + 空闲线检测 */
  if (UART_Port_StartReceive(port) != HAL_OK)
  {
    return 3;
  }

  return 0;
}

/**
  * @brief  异步发送一帧数据(DMA 优先, 无 DMA 时用中断)
  * @param  port   端口对象
  * @param  data   数据指针(发送期间须保持有效)
  * @param  length 长度
  * @retval 0 成功; 1 参数错误; 2 发送失败(上一帧未发完/外设异常)
  */
uint8_t UART_Port_Send(UART_PortObject *port, uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;

  if (port == NULL || port->huart == NULL || data == NULL || length == 0U)
  {
    return 1;
  }

  /* 本工程 USART1 未配 DMA, 退回中断发送; 两种模式都要求缓冲在发完前有效 */
  if (port->huart->hdmatx != NULL)
  {
    status = HAL_UART_Transmit_DMA(port->huart, data, length);
  }
  else
  {
    status = HAL_UART_Transmit_IT(port->huart, data, length);
  }

  if (status != HAL_OK)
  {
    port->stats.tx_error++;
    return 2;
  }

  port->stats.tx_count++;
  return 0;
}

/**
  * @brief  查询上一次异步发送是否尚未完成
  * @param  port 端口对象
  * @retval 1 上一帧仍在发送(DMA/中断/移位中); 0 空闲或参数错误
  * @note   发送完成由 HAL 在 TC 中断里把 gState 恢复 READY, 判断可靠;
  *         用于"发帧前先确认缓冲区没有在途读取"的场景(如 vofa 组帧缓冲)
  */
uint8_t UART_Port_TxBusy(UART_PortObject *port)
{
  if (port == NULL || port->huart == NULL)
  {
    return 0;
  }
  return (port->huart->gState == HAL_UART_STATE_BUSY_TX) ? 1U : 0U;
}

/**
  * @brief  阻塞格式化打印(仿照参考实现一的 UART_Print)
  * @param  port 端口对象(替代参考实现里的 PRINT_UART 全局宏)
  * @param  fmt  格式串, 换行需自行加 "\r\n"
  * @retval 已发送字符数; 参数错误、并发冲突或发送失败返回负值
  * @note   阻塞发送, 只能在主循环/线程上下文调用, 不要在中断里调用;
  *         静态缓冲不可重入, 忙标志基于 PRIMASK, 不依赖 RTOS;
  *         另一路正在打印时本路直接返回 -1(不排队、不阻塞)
  */
int UART_Port_Printf(UART_PortObject *port, const char *fmt, ...)
{
  va_list ap;
  int ret;

  if (port == NULL || port->huart == NULL || fmt == NULL)
  {
    return -1;
  }

  if (UART_PrintLock() == 0U)
  {
    return -1;
  }

  va_start(ap, fmt);
  ret = vsnprintf(s_print_buf, sizeof(s_print_buf), fmt, ap);
  va_end(ap);

  if (ret >= 0)
  {
    if ((size_t)ret >= sizeof(s_print_buf))
    {
      ret = (int)sizeof(s_print_buf) - 1; /* 超长截断 */
    }

    if (HAL_UART_Transmit(port->huart, (uint8_t *)s_print_buf, (uint16_t)ret, 100U) != HAL_OK)
    {
      port->stats.tx_error++;
      ret = -1;
    }
    else
    {
      port->stats.tx_count++;
    }
  }

  UART_PrintUnlock();

  return ret;
}

/**
  * @brief  配置端口的周期发送缓冲(仿照参考实现二的串口绘图帧)
  * @param  port   端口对象
  * @param  data   数据缓冲(调用方持有, 每个节拍都会被发送)
  * @param  length 长度; data 传 NULL 表示清除
  * @retval 无
  * @note   配合 UART_Port_Tick 使用; 上一帧 DMA 未发完时本拍跳过(计 tx_error)
  */
void UART_Port_SetPeriodicTx(UART_PortObject *port, uint8_t *data, uint16_t length)
{
  if (port == NULL)
  {
    return;
  }

  if (data == NULL || length == 0U)
  {
    port->periodic_tx_data = NULL;
    port->periodic_tx_len = 0;
    return;
  }

  port->periodic_tx_data = data;
  port->periodic_tx_len = length;
}

/**
  * @brief  UART 周期发送节拍, 与 CAN_Bus_Tick 共用同一个定时器中断调用
  * @note   遍历所有已登记端口, 配置了周期缓冲的端口每拍发送一次
  */
void UART_Port_Tick(void)
{
  for (uint8_t i = 0; i < s_port_count; i++)
  {
    UART_PortObject *port = s_port_registry[i];

    if (port->periodic_tx_data != NULL && port->periodic_tx_len != 0U)
    {
      UART_Port_Send(port, port->periodic_tx_data, port->periodic_tx_len);
    }
  }
}

/**
  * @brief  HAL 库 UART 空闲/DMA 接收事件回调(覆盖 HAL 弱定义)
  * @note   驱动在业务回调返回后重新武装接收, 保证缓冲区在回调期间不被覆写
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  UART_PortObject *port = UART_Port_FindByHandle(huart);

  if (port == NULL)
  {
    return;
  }

  port->stats.rx_count++;
  if (port->on_rx != NULL)
  {
    /*传实际收到的 Size, 修正参考实现二误传缓冲区总长度的缺陷 */
    port->on_rx(port, port->rx_buffer, Size);
  }

  if (UART_Port_StartReceive(port) != HAL_OK)
  {
    port->stats.rx_error++;
  }
}

/**
  * @brief  HAL 库 UART 错误回调(覆盖 HAL 弱定义)
  * @note   噪声/溢出等错误会中止进行中的接收, 这里清理现场后重新武装,
  *          否则一次帧错误后该串口就永久收不到数据
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  UART_PortObject *port = UART_Port_FindByHandle(huart);

  if (port == NULL)
  {
    return;
  }

  port->stats.err_count++;
  HAL_UART_AbortReceive(huart);
  if (UART_Port_StartReceive(port) != HAL_OK)
  {
    port->stats.rx_error++;
  }
}
