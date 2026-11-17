/**
 * @file    app_log.c
 * @brief   极简整数打印实现(不依赖 newlib vsnprintf)
 *
 * @note    用 UART1 的"阻塞发送"直接写字节: 一次调用只组一小段缓冲,
 *          占用栈 < 80 字节。DEBUG_VISION_ENABLE==1 时整个文件为空实现,
 *          保证蓝牙独占串口时不会混入文本。
 */

/* Includes ------------------------------------------------------------------*/
#include "app_log.h"
#include "debug_config.h"
#include "usart.h"
#include "drv_uart.h"
#include "main.h"
#include <stdarg.h>

#if (DEBUG_VISION_ENABLE == 0)

/* Private macros ------------------------------------------------------------*/

#define APPLOG_BUF     48U     /* 单次格式化缓冲 */
#define APPLOG_TX_TMO  10U     /* 阻塞发送超时(ms) */

/* Private functions ---------------------------------------------------------*/

static void AppLog_Flush(const char *buf, uint16_t len)
{
    if ((len == 0U) || (buf == NULL))
    {
        return;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, APPLOG_TX_TMO);
}

/* 把无符号数按进制写进缓冲, 返回写入长度 */
static uint16_t AppLog_UInt(char *out, uint32_t v, uint32_t base,
                            uint8_t upper, uint8_t pad, uint8_t width)
{
    char *buf = out;
    char  tmp[12];
    uint16_t n = 0U;
    uint16_t i = 0U;

    if (v == 0U)
    {
        tmp[n++] = '0';
    }
    else
    {
        while ((v != 0U) && (n < sizeof(tmp)))
        {
            uint32_t d = v % base;
            tmp[n++] = (char)((d < 10U) ? ('0' + d)
                                        : ((upper ? 'A' : 'a') + (d - 10U)));
            v /= base;
        }
    }

    /* 前导 0 填充 */
    if (pad && (width > n))
    {
        for (uint16_t k = 0U; k < (uint16_t)(width - n); k++)
        {
            buf[i++] = '0';
        }
    }

    while (n > 0U)
    {
        buf[i++] = tmp[--n];
    }

    return i;
}

/* Exported functions ---------------------------------------------------------*/

void AppLog_Str(const char *s)
{
    const char *p = s;
    uint16_t    n = 0U;

    if (s == NULL)
    {
        return;
    }

    /* 分段发送, 避免一次性求长度后再阻塞很久 */
    while (*p != '\0')
    {
        p++;
        n++;
        if (n >= APPLOG_BUF)
        {
            AppLog_Flush(s, n);
            s = p;
            n = 0U;
        }
    }
    if (n != 0U)
    {
        AppLog_Flush(s, n);
    }
}

void AppLog_Printf(const char *fmt, ...)
{
    va_list ap;

    if (fmt == NULL)
    {
        return;
    }
    va_start(ap, fmt);
    AppLog_VPrintf(fmt, ap);
    va_end(ap);
}

void AppLog_VPrintf(const char *fmt, va_list ap_in)
{
    char     buf[APPLOG_BUF];
    uint16_t i = 0U;
    va_list  ap;

    if (fmt == NULL)
    {
        return;
    }

    va_copy(ap, ap_in);

    while (*fmt != '\0')
    {
        if (*fmt != '%')
        {
            if (i >= (APPLOG_BUF - 1U))
            {
                AppLog_Flush(buf, i);
                i = 0U;
            }
            buf[i++] = *fmt++;
            continue;
        }

        fmt++;   /* 跳过 '%' */

        /* ---- 解析 flags/宽度 ---- */
        {
            uint8_t  pad   = 0U;
            uint8_t  width = 0U;
            uint8_t  upper = 0U;
            uint8_t  is_long = 0U;

            if (*fmt == '0')
            {
                pad = 1U;
                fmt++;
            }
            while ((*fmt >= '0') && (*fmt <= '9'))
            {
                width = (uint8_t)(width * 10U + (uint8_t)(*fmt - '0'));
                fmt++;
            }
            if (*fmt == 'l')
            {
                is_long = 1U;
                fmt++;
            }

            /* ---- 取参数 ---- */
            switch (*fmt)
            {
                case 'd':
                {
                    int32_t v = is_long ? (int32_t)va_arg(ap, long) : va_arg(ap, int);
                    uint32_t u;
                    if (v < 0)
                    {
                        if (i < (APPLOG_BUF - 1U)) { buf[i++] = '-'; }
                        u = (uint32_t)(-v);
                    }
                    else
                    {
                        u = (uint32_t)v;
                    }
                    i = (uint16_t)(i + AppLog_UInt(&buf[i], u, 10U, 0U, pad, width));
                    break;
                }
                case 'u':
                {
                    uint32_t v = is_long ? (uint32_t)va_arg(ap, unsigned long)
                                         : va_arg(ap, unsigned int);
                    i = (uint16_t)(i + AppLog_UInt(&buf[i], v, 10U, 0U, pad, width));
                    break;
                }
                case 'x':
                case 'X':
                {
                    uint32_t v;
                    upper = (uint8_t)((*fmt == 'X') ? 1U : 0U);
                    v = is_long ? (uint32_t)va_arg(ap, unsigned long)
                                : va_arg(ap, unsigned int);
                    i = (uint16_t)(i + AppLog_UInt(&buf[i], v, 16U, upper, pad, width));
                    break;
                }
                case 'c':
                    if (i < (APPLOG_BUF - 1U))
                    {
                        buf[i++] = (char)va_arg(ap, int);
                    }
                    break;

                case 's':
                {
                    const char *s = va_arg(ap, const char *);
                    if (s == NULL) { s = "(null)"; }
                    while (*s != '\0')
                    {
                        if (i >= (APPLOG_BUF - 1U))
                        {
                            AppLog_Flush(buf, i);
                            i = 0U;
                        }
                        buf[i++] = *s++;
                    }
                    break;
                }

                case '%':
                    if (i < (APPLOG_BUF - 1U)) { buf[i++] = '%'; }
                    break;

                default:
                    /* 不支持的转换: 原样输出 % 与字符 */
                    if (i < (APPLOG_BUF - 1U)) { buf[i++] = '%'; }
                    if ((*fmt != '\0') && (i < (APPLOG_BUF - 1U))) { buf[i++] = *fmt; }
                    break;
            }

            if (*fmt != '\0')
            {
                fmt++;
            }
        }

        if (i >= (APPLOG_BUF - 1U))
        {
            AppLog_Flush(buf, i);
            i = 0U;
        }
    }

    va_end(ap);

    if (i != 0U)
    {
        AppLog_Flush(buf, i);
    }
}

#else  /* DEBUG_VISION_ENABLE == 1: 串口被蓝牙占用, 文本全部丢弃 */

void AppLog_Str(const char *s)                { (void)s; }
void AppLog_Printf(const char *fmt, ...)      { (void)fmt; }
void AppLog_VPrintf(const char *fmt, va_list ap) { (void)fmt; (void)ap; }

#endif