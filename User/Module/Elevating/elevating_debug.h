/**
 * @file    elevating_debug.h
 * @brief   丝杆升降平台模块自检(接口层)
 * @version 1.0
 * @date    2026-11-17
 */

#ifndef __ELEVATING_DEBUG_H__
#define __ELEVATING_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"

#if elevating_debug == 1

void Elevating_Debug_Init(void);
void Elevating_Debug_Run(uint32_t now);

#else

static inline void Elevating_Debug_Init(void)
{
}

static inline void Elevating_Debug_Run(uint32_t now)
{
    (void)now;
}

#endif /* elevating_debug == 1 */

#ifdef __cplusplus
}
#endif

#endif /* __ELEVATING_DEBUG_H__ */