#ifndef __GRIP_DEBUG__
#define __GRIP_DEBUG__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"

#if grip_debug == 1
void Grip_Debug_Init(void);
void Grip_Debug_Run(uint32_t now);
#else
static inline void Grip_Debug_Init(void) {}
static inline void Grip_Debug_Run(uint32_t now) { (void)now; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __GRIP_DEBUG__ */
