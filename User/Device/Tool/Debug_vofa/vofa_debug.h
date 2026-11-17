#ifndef __VOFA_DEBUG__
#define __VOFA_DEBUG__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"

#if vofa_debug == 1
void Vofa_Debug_Init(void);
void Vofa_Debug_Run(uint32_t now);
#else
static inline void Vofa_Debug_Init(void) {}
static inline void Vofa_Debug_Run(uint32_t now) { (void)now; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_DEBUG__ */
