#ifndef __AT8236_DEBUG_H__
#define __AT8236_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"

/**
 * AT8236 四路电机驱动板自检(电机会转! 先架空车轮)
 *
 * 启用方法(二选一):
 *   1. 在 User/APP/debug_config.h 中加入
 *        #define at8236_debug 1
 *      并在 User/APP/debug.c 的 debug_init/debug_run 中分别调用
 *        AT8236_Debug_Init();
 *        AT8236_Debug_Run(now);
 *   2. 编译选项里加 -Dat8236_debug=1, 同样需要自行调用上面两个函数。
 *
 * 与 bt_vision_debug 互斥: 两者都占用 USART2。
 * 本工程 USART2(PA2/PA3) 必须先在 CubeMX 中打开 RX/TX DMA, 否则 drv_uart
 * 无法启动接收, 自检会在第一步打印失败原因。
 */
#ifndef at8236_debug
#define at8236_debug 0
#endif

#if (at8236_debug == 1) && (bt_vision_debug == 1)
#error "at8236_debug and bt_vision_debug both bind USART2 (mutually exclusive)"
#endif

#if at8236_debug == 1
void AT8236_Debug_Init(void);
void AT8236_Debug_Run(uint32_t now);
#else
static inline void AT8236_Debug_Init(void) {}
static inline void AT8236_Debug_Run(uint32_t now) { (void)now; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __AT8236_DEBUG_H__ */
