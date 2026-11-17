/**
 * @file    debug.h
 * @brief   APP 层调试入口: 分层自检(台架) + 蓝牙可视化(可选)
 *
 * @note    RUN_MODE == RUN_MATCH 时 debug_init/debug_run 是空实现,
 *          所有调试代码被编译掉(见 debug_config.h 的互斥检查)。
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "debug_config.h"

/**
 * @brief  初始化当前选中的调试层(只调一次)
 * @note   内部会先做底层公共初始化(Task_IT_Init / 执行器 / 传感器),
 *         再按 debug_config.h 的宏进入对应层的 Init。
 */
void debug_init(void);

/**
 * @brief  主循环调用, 非阻塞
 */
void debug_run(void);

/**
 * @brief  5ms 控制节拍(由 task_it.c 的 Task_Control_5ms 分发)
 * @note   与 App_ControlTick 二选一, 保证台架调试时控制环也在跑
 */
void Debug_ControlTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */