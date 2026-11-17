/**
 * @file    app_log.h
 * @brief   极简整数打印(不依赖 newlib printf)
 *
 * @note    本机只有 64K Flash。一旦任何地方用到 UART_Port_Printf / printf,
 *          链接器就会把整个 vsnprintf 拉进来(约 3~4 KB), 比赛固件直接超容量。
 *          所以 APP 层统一走这个只支持整数的迷你格式化器。
 *
 *          支持的转换:
 *            %d %u %ld %lu   十进制(带符号/无符号)
 *            %x %X %02X      十六进制(可选 0 填充宽度)
 *            %c              单字符
 *            %s              字符串
 *            %%              百分号
 *          不支持: 浮点(%f)、宽度/精度组合(除 0 填充外)。
 *          浮点请先乘 100 转成整数, 打印成 "整数.两位"。
 */

#ifndef __APP_LOG_H__
#define __APP_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdarg.h>

/**
 * @brief  格式化并发送到 USART1
 * @note   主循环上下文调用; 中断里禁止调用(会阻塞发送)
 */
void AppLog_Printf(const char *fmt, ...);

/**
 * @brief  va_list 版本(供内部包装函数转发)
 */
void AppLog_VPrintf(const char *fmt, va_list ap);

/**
 * @brief  直接发一段字符串
 */
void AppLog_Str(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* __APP_LOG_H__ */