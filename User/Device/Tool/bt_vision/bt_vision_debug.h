/**
 * @file    bt_vision_debug.h
 * @brief   bt_vision 蓝牙调试上位机(接口层): 把 btdbg 协议库接到本工程的
 *          USART2(HC-05) + 参数表 + 遥测缓存上, 由 debug.c 统一调度
 * @note    上位机: bt-debug-visualizer; 协议: COBS + CRC16 + 0x00 分隔
 * @note    HC-05 必须先用 AT+UART=115200,0,0 改过波特率, 否则全是 CRC 错
 */

#ifndef __BT_VISION_DEBUG__
#define __BT_VISION_DEBUG__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "debug_config.h"
#include "btdbg.h"

#if bt_vision_debug == 1

/* 初始化: 必须在 MX_USART2_UART_Init() 之后调用(debug_init 内) */
void BtVision_Debug_Init(void);

/* 主循环周期调用: 解析命令 / 心跳 / 遥测上报 / 零偏校准采样 */
void BtVision_Debug_Run(uint32_t now);

/* 遥测数据由业务层喂入(主循环上下文), Run 按上报频率统一发送.
 * 未喂入的通道发 0; 陀螺仪必须传"未减零偏"的原始角速度(°/s), 校准用 */
void BtVision_Debug_UpdateMotor(float tgt_pos, float pos, float tgt_spd, float spd, float out);
void BtVision_Debug_UpdateGyro(float gx, float gy, float gz);
void BtVision_Debug_UpdateCar(float spd, float dist, float yaw);

/* 状态切换(0x40 事件立即上报). 库会按状态自动拒绝不允许的参数写入 */
void BtVision_Debug_SetState(btdbg_state_t state, uint8_t reason);

#else /* bt_vision_debug == 0 */

static inline void BtVision_Debug_Init(void) {}
static inline void BtVision_Debug_Run(uint32_t now) { (void)now; }

static inline void BtVision_Debug_UpdateMotor(float tgt_pos, float pos, float tgt_spd,
                                              float spd, float out)
{
    (void)tgt_pos; (void)pos; (void)tgt_spd; (void)spd; (void)out;
}

static inline void BtVision_Debug_UpdateGyro(float gx, float gy, float gz)
{
    (void)gx; (void)gy; (void)gz;
}

static inline void BtVision_Debug_UpdateCar(float spd, float dist, float yaw)
{
    (void)spd; (void)dist; (void)yaw;
}

static inline void BtVision_Debug_SetState(btdbg_state_t state, uint8_t reason)
{
    (void)state; (void)reason;
}

#endif /* bt_vision_debug == 1 */

#ifdef __cplusplus
}
#endif

#endif /* __BT_VISION_DEBUG__ */