#ifndef __SERVO_DEBUG__
#define __SERVO_DEBUG__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"

#if servo_debug == 1
void Servo_Debug_Init(void);
void Servo_Debug_Run(uint32_t now);
#else
static inline void Servo_Debug_Init(void) {}
static inline void Servo_Debug_Run(uint32_t now) { (void)now; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_DEBUG__ */
