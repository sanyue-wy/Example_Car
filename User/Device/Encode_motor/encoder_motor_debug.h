#ifndef __ENCODER_MOTOR_DEBUG__
#define __ENCODER_MOTOR_DEBUG__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"

#if encoder_motor_debug == 1
void EncoderMotor_Debug_Init(void);
void EncoderMotor_Debug_Run(uint32_t now);
#else
static inline void EncoderMotor_Debug_Init(void) {}
static inline void EncoderMotor_Debug_Run(uint32_t now) { (void)now; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_MOTOR_DEBUG__ */