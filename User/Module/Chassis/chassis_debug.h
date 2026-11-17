#ifndef __CHASSIS_DEBUG_H__
#define __CHASSIS_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"

/**
 * 四轮差速底盘 + AT8236 驱动板自检(电机会转! 必须架空车轮)
 *
 * 启用方法(二选一):
 *   1. 在 User/APP/debug_config.h 中加入
 *        #define chassis_debug 1
 *      并在 User/APP/debug.c 的 debug_init/debug_run 中分别调用
 *        Chassis_Debug_Init();
 *        Chassis_Debug_Run(now);
 *   2. 编译选项里加 -Dchassis_debug=1, 同样需要自行调用上面两个函数。
 *
 * 依赖: AT8236 驱动板接 USART2(PA2/PA3), 且 USART2 已配置 RX/TX DMA。
 * 互斥: 与 at8236_debug、bt_vision_debug 冲突(同占 USART2), 编译期报错。
 */
#ifndef chassis_debug
#define chassis_debug 0
#endif

#if (chassis_debug == 1) && (at8236_debug == 1)
#error "chassis_debug and at8236_debug both init the AT8236 board on USART2 (mutually exclusive)"
#endif

#if (chassis_debug == 1) && (bt_vision_debug == 1)
#error "chassis_debug and bt_vision_debug both bind USART2 (mutually exclusive)"
#endif

#if chassis_debug == 1
void Chassis_Debug_Init(void);
void Chassis_Debug_Run(uint32_t now);
#else
static inline void Chassis_Debug_Init(void) {}
static inline void Chassis_Debug_Run(uint32_t now) { (void)now; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_DEBUG_H__ */
