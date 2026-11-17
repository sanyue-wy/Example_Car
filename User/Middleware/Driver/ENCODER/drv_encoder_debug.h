#ifndef __DRV_ENCODER_DEBUG__
#define __DRV_ENCODER_DEBUG__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"

#if encoder_debug == 1
void Encoder_Debug_Init(void);
void Encoder_Debug_Run(uint32_t now);
#else
static inline void Encoder_Debug_Init(void) {}
static inline void Encoder_Debug_Run(uint32_t now) { (void)now; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __DRV_ENCODER_DEBUG__ */