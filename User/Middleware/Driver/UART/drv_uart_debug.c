#include "debug_config.h"

#if uart_debug == 1

#include "drv_uart_debug.h"
#include "drv_uart.h"
#include "main.h"
#include "drv_tim.h"
#include "tim.h"
#include <string.h>

static void Debug_UartRxCallback(UART_PortObject *port, uint8_t *data, uint16_t size);

/**
  * @brief  USART1 接收回调(中断上下文): 原样回显, 方便串口助手手动发字节验证
  * @note   异步发送非阻塞(DMA/中断); 上一帧没发完则本段丢弃
  */
static void Debug_UartRxCallback(UART_PortObject *port, uint8_t *data, uint16_t size)
{
  (void)UART_Port_Send(port, data, size);
}

void UART_Debug_Init(void)
{
    /* 手动验证: 串口助手(115200) 发任意字节, 应原样回显 */
    uart1_port.on_rx = Debug_UartRxCallback;
    UART_Port_Printf(&uart1_port, "[uart] init ok\r\n");
}

void UART_Debug_Run(uint32_t now)
{
    {
        static uint32_t s_last = 0;
        if (now - s_last >= 1000U)
        {
            s_last = now;
            /* irq_count 每秒应 +1000 左右, 否则系统 1ms 节拍中断有问题 */
            UART_Port_Printf(&uart1_port,
                             "[uart] ok | tx=%lu rx=%lu err=%lu | tick_ms=%lu\r\n",
                             (unsigned long)uart1_port.stats.tx_count,
                             (unsigned long)uart1_port.stats.rx_count,
                             (unsigned long)(uart1_port.stats.rx_error + uart1_port.stats.err_count),
                             (unsigned long)TIM_Timer_GetIrqCount(&tim7_timer));
        }
    }
}

#endif /* uart_debug == 1 */
