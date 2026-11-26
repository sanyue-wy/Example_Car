/**
 * @file    app.h
 * @brief   Warman39 竞赛小车 APP 层入口: 比赛主状态机 + 动作脚本 + 安全监控
 * @note    分层:
 *            APP (app.c)  ->  Module (Chassis / Lift / Bucket)
 *                         ->  Device  (AT8236 驱动板 / encoder_motor / servo)
 *                         ->  Middleware (drv_tim / drv_pwm / drv_uart / drv_encoder / pid)
 *
 *          本文件只对外暴露 Init / Run 两个入口, 其它全部内聚在 app.c。
 */

#ifndef __APP_H__
#define __APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

/*==============================================================================
 * 运行模式选择 (上场前必须改成 RUN_MATCH)
 *============================================================================*/
#define RUN_MODE    RUN_DEBUG

/*==============================================================================
 * 启动按键兜底: CubeMX 尚未配置 PB12 时置 1, 用"调试口收到任意字节"触发启动。
 * 只允许 RUN_DEBUG 使用 —— 比赛规则 FAQ3/R21 禁止比赛中接收上位机指令,
 * 所以 RUN_MATCH 下本宏为 1 会直接编译失败(见下方)。
 *============================================================================*/
#define APP_START_KEY_FALLBACK   0

/*==============================================================================
 * 比赛模式互斥检查
 *============================================================================*/
#if (RUN_MODE != RUN_MATCH) && (RUN_MODE != RUN_DEBUG)
#error "app.h: RUN_MODE 只能是 RUN_MATCH 或 RUN_DEBUG"
#endif

#if (RUN_MODE == RUN_MATCH) && (APP_START_KEY_FALLBACK == 1)
#error "比赛模式禁止用串口触发启动(FAQ3/R21): APP_START_KEY_FALLBACK 必须为 0"
#endif

/*==============================================================================
 * 对外接口
 *============================================================================*/

/**
 * @brief  比赛状态机
 */
typedef enum
{
    APP_BOOT   = 0,  /* 上电初始化: 执行器全部不输出 */
    APP_READY  = 1,  /* 设置期间: 电机禁用, 等启动键, LED 慢闪 */
    APP_RUN    = 2,  /* 比赛轮次: 执行动作脚本 */
    APP_FINISH = 3,  /* 收尾完成: 所有执行器禁用, LED 常亮 */
    APP_FAULT  = 4   /* 故障: 急停, LED 快闪 */
} AppState_e;

/**
 * @brief  故障原因(供 LED/打印/蓝牙上报)
 */
typedef enum
{
    APP_FAULT_NONE       = 0,
    APP_FAULT_STALL      = 1,  /* 电机堵转超时 */
    APP_FAULT_LIFT_RANGE = 2,  /* 升降超行程/超时 */
    APP_FAULT_GEOFENCE   = 3,  /* 里程计估计位置进入赛道边缘禁区 */
    APP_FAULT_FEEDBACK   = 4,  /* 编码器/驱动板无反馈 */
    APP_FAULT_SELF       = 5   /* 内部状态错误 */
} AppFault_e;

/**
 * @brief  初始化比赛业务层(外设已在 main 中 MX_xxx_Init 完成)
 * @note   只登记对象与绑定, **不输出任何 PWM / 不给驱动板下速度指令**;
 *         上电到按启动键之前, 任何代码路径都不得让电机或舵机动作(R11/R14/G27)
 */
void App_Init(void);

/**
 * @brief  只做驱动层 + 执行器初始化, 不进比赛状态机
 * @note   台架调试(debug_init)复用同一套硬件初始化, 避免两套配置漂移。
 *         调用后执行器仍全部处于禁用状态。
 */
void App_InitDrivers(void);

/**
 * @brief  主循环调用, 非阻塞
 * @note   内部是状态机的一次推进, 禁止内部出现 while(!done) 或 HAL_Delay
 */
void App_Run(void);

/**
 * @brief  周期控制任务, 挂到 drv_tim 的 1ms 节拍, 分频 5 -> 5ms 一次
 * @note   只在中断里做采样/闭环/下发; 不做打印、不做动态内存
 */
void App_ControlTick(void);

/**
 * @brief  查询当前状态 / 故障原因(调试与蓝牙可视化使用)
 */
AppState_e App_GetState(void);
uint8_t    App_GetFault(void);
uint32_t   App_GetElapsedMs(void);

/**
 * @brief  外部运行期对象访问(供 Module/APP 内部互查, 也供 debug 层)
 * @note   返回内部静态对象的只读指针; 不在初始化前调用
 */
struct Chassis_Object;
struct Elevating_Object;
struct AppBucket;
struct AT8236_Object;

const struct Chassis_Object   *App_GetChassis(void);
const struct Elevating_Object *App_GetLift(void);
struct AppBucket              *App_GetBucket(void);

/**
 * @brief  驱动板对象(供单电机自检等调试层直接操作通道)
 * @note   返回可写指针; 仅在 App_InitDrivers 成功后非 NULL。
 *          正常业务代码不要用它, 走 Chassis 抽象。
 */
struct AT8236_Object *App_GetBoard(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H__ */