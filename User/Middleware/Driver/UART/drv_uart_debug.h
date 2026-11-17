#ifndef __DRV_UART_DEBUG__
#define __DRV_UART_DEBUG__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"

#if uart_debug == 1
void UART_Debug_Init(void);
void UART_Debug_Run(uint32_t now);
#else
static inline void UART_Debug_Init(void) {}
static inline void UART_Debug_Run(uint32_t now) { (void)now; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_DEBUG__ */
