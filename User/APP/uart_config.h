/**
 * @file    uart_config.h
 * @brief   串口波特率集中配置(切换调试通道只改这里)
 *
 * @note    本工程只有两个串口:
 *            - USART1 (PA9=TX / PA10=RX): 调试打印 / 蓝牙可视化 / DX-BT24
 *            - USART2 (PA2=TX / PA3=RX) : AT8236 四路驱动板(板载固件固定 115200)
 *
 *          USART1 常见两种接法, 只改 UART_DEBUG_BAUD 即可:
 *            1) USB-TTL 接电脑, 串口助手做文本调试 : 115200 (默认)
 *            2) DX-BT24 蓝牙模块(出厂默认)          : 9600
 *               —— 模块若已用 AT+BAUD 改成 115200, 这里也改回 115200。
 *
 * @note    改完宏必须重新编译烧录; 只改这里不重烧不会生效。
 *          串口助手/上位机的波特率必须与此处一致, 否则看到的是乱码。
 *
 * @note    定义用了 #ifndef, 也可以用编译参数 -DUART_DEBUG_BAUD=9600 临时覆盖,
 *          不必改文件。
 */

#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* USART1: 调试打印 / 蓝牙通道 */
#ifndef UART_DEBUG_BAUD
#define UART_DEBUG_BAUD     115200U
#endif

/* USART2: AT8236 驱动板通讯, 驱动板固件固定 115200, 一般不要改 */
#ifndef UART_BOARD_BAUD
#define UART_BOARD_BAUD     115200U
#endif

#ifdef __cplusplus
}
#endif

#endif /* __UART_CONFIG_H__ */
