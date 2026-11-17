/**
 * @file    debug_vision.h
 * @brief   蓝牙可视化调试服务(HC-05 + bt-debug-visualizer), 基于 btdbg 协议库
 *
 * @note    【本次改动的关键点: 串口从 USART2 改到 USART1】
 *          任务书 v2 §0.5 要求蓝牙必须用独立串口, 不能与 printf 共用。
 *          但本工程 F103C8T6 只有 USART1(printf) / USART2(AT8236 驱动板) /
 *          USART3(未在 CubeMX 开启), 用户指定蓝牙走 **USART1**。
 *          因此这里采用互斥策略: DEBUG_VISION_ENABLE == 1 时,
 *            - 关闭全部文本打印(app.c / debug.c 里的 DBG 宏被编译掉);
 *            - 该串口只承载 btdbg 的 COBS 二进制帧。
 *          两种数据流不会混在同一时刻, 但**不能同时**使用脚本助手看文本。
 *
 * @note    【比赛禁止】RUN_MODE==RUN_MATCH 且本宏为 1 会在 debug_config.h 中
 *          #error; 并且比赛时物理上要拔掉 HC-05。
 *
 * @note    【线程模型】
 *          - btdbg_rx_byte() 只能在中断里调: 本文件把 USART1 空闲中断收到的
 *            字节逐个喂进库, 复用 drv_uart 已有的 on_rx 回调通道;
 *          - btdbg_poll / send_telemetry / set_state 只在主循环调;
 *          - 控制环(5ms 中断)只写 volatile 快照, 不碰库。
 */

#ifndef __DEBUG_VISION_H__
#define __DEBUG_VISION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "debug_config.h"

#if (DEBUG_VISION_ENABLE == 1)

#include <stdint.h>
#include "btdbg.h"
#include "params_car.h"

/**
 * @brief  可视化测试场景
 * @note   这里是"C 里的名字", 编译期的值是 debug_config.h 的 VISION_SCENE_* 宏。
 *         之所以不直接用同名枚举: VISION_SCENE_* 已经是宏, 同名枚举会被宏替换,
 *         表格里写出来的是数字而不是可读名字。
 */
typedef enum
{
    DV_SCENE_LINK     = VISION_SCENE_LINK,      /* V1 链路自检: 只发波形, 不动执行器 */
    DV_SCENE_WHEEL_SPD = VISION_SCENE_WHEEL_SPD,/* V2 底盘速度阶跃            电机会转! */
    DV_SCENE_LIFT_POS = VISION_SCENE_LIFT_POS,  /* V3 升降位置阶跃            升降会动! */
    DV_SCENE_STRAIGHT = VISION_SCENE_STRAIGHT,  /* V4 直线行驶                车会跑!   */
    DV_SCENE_SCRIPT   = VISION_SCENE_SCRIPT,    /* V5 分段脚本试跑            车会跑!   */
    DV_SCENE_OBSERVE  = VISION_SCENE_OBSERVE    /* V6 只观察                  推车看里程计 */
} VisionScene_t;

/**
 * @brief  故障原因码(与 btdbg 的 FAULT reason 一致)
 */
#define DV_FAULT_NONE       0U
#define DV_FAULT_STALL      1U
#define DV_FAULT_LIFT_RANGE 2U
#define DV_FAULT_GEOFENCE   3U
#define DV_FAULT_NO_FEEDBACK 4U

/**
 * @brief  初始化(在 debug_init 末尾调用, 串口已由 App_InitDrivers 打开)
 */
void DebugVision_Init(void);

/**
 * @brief  主循环每圈调用(非阻塞)
 */
void DebugVision_Loop(void);

/**
 * @brief  USART1 接收回调(中断上下文): 逐字节喂给 btdbg
 * @note   由 DebugVision_Init 挂到 uart1_port.on_rx;
 *         全局只能有一个 USART1 的 on_rx, 打开可视化时 uart_debug 必须关闭。
 */
struct UART_PortObject;
void DebugVision_RxBytes(struct UART_PortObject *port, uint8_t *data, uint16_t size);

/**
 * @brief  控制环(5ms 中断)里写遥测快照
 * @note   只写 volatile 字段, 不调用任何 btdbg 函数
 */
void DebugVision_Snapshot(void);

/**
 * @brief  上报故障(供 App 安全监控调用, 主循环上下文)
 */
void DebugVision_SetFault(uint8_t reason);

#endif /* DEBUG_VISION_ENABLE == 1 */

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_VISION_H__ */