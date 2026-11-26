/**
 * @file    debug_config.h
 * @brief   分层自检开关(台架调试用)。从底层往上逐层验证, 一次只开一个。
 *
 * @note    【与任务书的差异】任务书 §5.9 的层名是按"5 路 PWM 直驱编码电机"写的,
 *          本车底盘由 AT8236 串口驱动板闭环, 主控没有 4 路轮速 PID, 因此:
 *            - motor_open_debug : 改为"驱动板开环速度指令", 电机会转!
 *            - motor_pid_debug  : 改为"驱动板内置 PID 阶跃", 电机会转!
 *          其余层与任务书一一对应。详见 README_APP.md。
 *
 * @note    开关顺序与验证方法:
 *            1. uart_debug        USART1 打印自检(波特率见 uart_config.h)
 *            2. encoder_debug     TIM2 升降编码器: 手转丝杆, 打印计数/方向(不动作)
 *            3. motor_open_debug  AT8236 四轮 +-200mm/s 正反转(车架空!)
 *            4. motor_pid_debug   AT8236 速度阶跃, 串口看超调/稳态误差(车架空!)
 *            5. servo_debug       铲斗 STOW/SCOOP/CARRY/DUMP 循环(舵机会动!)
 *            6. lift_debug        回零 + 0->LEVEL1->HOME, 打印到位误差(丝杆会动!)
 *            7. chassis_debug     直行 1000mm / 原地转 90deg x4, 打印终点误差(车会跑!)
 *            8. script_debug      单段动作脚本试跑(启动键触发, 车会跑!)
 *            9. single_motor_debug 单电机正反转; 必须再把 motor1..4 之一置 1
 *                                  (电机会转! 绕开底盘左右分组, 直接下发通道)
 *          第 10 项 DEBUG_VISION_ENABLE 是"观察通道", 可与上面任意一层同时打开。
 */

#ifndef __DEBUG_CONFIG_H__
#define __DEBUG_CONFIG_H__

#include "app.h"          /* RUN_MODE / RUN_MATCH */

/*==============================================================================
 * 分层自检开关: 一次只开一个
 *============================================================================*/
#ifndef uart_debug
#define uart_debug          0   /* 1. 串口打印/USART1 收发自检 */
#endif
#ifndef encoder_debug
#define encoder_debug       0   /* 2. 升降编码器(TIM2), 不动作 */
#endif
#ifndef motor_open_debug
#define motor_open_debug    0   /* 3. 驱动板开环速度, 电机会转! */
#endif
#ifndef motor_pid_debug
#define motor_pid_debug     0   /* 4. 驱动板速度环阶跃, 电机会转! */
#endif
#ifndef servo_debug
#define servo_debug         1   /* 5. 铲斗舵机循环, 舵机会动! */
#endif
#ifndef lift_debug
#define lift_debug          0   /* 6. 升降回零/位置闭环, 丝杆会动! */
#endif
#ifndef chassis_debug
#define chassis_debug       0   /* 7. 直行/转角标定, 车会跑! */
#endif
#ifndef script_debug
#define script_debug        0   /* 8. 单段动作脚本试跑, 车会跑! */
#endif
#ifndef single_motor_debug
#define single_motor_debug  0  /* 9. 单电机测试, 电机会转! */
#endif

/* ---- 9 的通道选择: single_motor_debug=1 时, 下面四个必须有且只有一个为 1 ----
 * 编号就是驱动板丝印 M1~M4; 本车实物位置见 chassis_map.h:
 *   M1=右前, M2=右后, M3=左后, M4=左前。 */
#ifndef motor1_test
#define motor1_test         0   /* M1 */
#endif
#ifndef motor2_test
#define motor2_test         0   /* M2 */
#endif
#ifndef motor3_test
#define motor3_test         0   /* M3 */
#endif
#ifndef motor4_test
#define motor4_test         0   /* M4 */
#endif

/* ---- 9 的驱动方式: 0=闭环 $spd(mm/s)  1=开环 $pwm(‰) ----
 * 排查"转得不对劲"时两种都跑一次:
 *   闭环抖、开环正常  -> 编码器/反馈有问题(电机与功率级是好的)
 *   两种都抖          -> 电机本体/线缆/该通道功率级 */
#ifndef single_motor_loop
#define single_motor_loop   0   /* 0=速度闭环, 1=开环 PWM */
#endif

/*==============================================================================
 * 10. 蓝牙可视化调试服务(HC-05 + bt-debug-visualizer)
 *    本工程按用户指定接 **USART1**。USART1 同时是文本打印口, 二者共用一条
 *    物理链路会互相污染(COBS 二进制帧 vs ASCII 文本), 因此:
 *      DEBUG_VISION_ENABLE == 1 时, 所有文本打印被编译掉, 该串口只跑 btdbg 帧。
 *    比赛中必须为 0, 且拔掉 HC-05 模块(规则 FAQ3/4, R21)。
 *============================================================================*/
#ifndef DEBUG_VISION_ENABLE
#define DEBUG_VISION_ENABLE     0
#endif

/* 可视化测试场景(仅 DEBUG_VISION_ENABLE==1 时生效, 定义在 debug_vision.h) */
#define VISION_SCENE_LINK       1
#define VISION_SCENE_WHEEL_SPD  1
#define VISION_SCENE_LIFT_POS   2
#define VISION_SCENE_STRAIGHT   3
#define VISION_SCENE_SCRIPT     4
#define VISION_SCENE_OBSERVE    5

#ifndef VISION_SCENE
#define VISION_SCENE            VISION_SCENE_LINK
#endif

/*==============================================================================
 * 编译期互斥检查
 *============================================================================*/
#define DEBUG_LAYER_SUM  (uart_debug + encoder_debug + motor_open_debug + \
                          motor_pid_debug + servo_debug + lift_debug + \
                          chassis_debug + script_debug + single_motor_debug)

#if DEBUG_LAYER_SUM > 1
#error "debug_config.h: 一次只能打开一个分层检测层"
#endif

/* 单电机层: 必须"模式=1 + 恰好一路电机=1" 才编译对应代码 */
#define DEBUG_SINGLE_SUM  (motor1_test + motor2_test + motor3_test + motor4_test)

#if (single_motor_debug == 1) && (DEBUG_SINGLE_SUM != 1)
#error "debug_config.h: single_motor_debug=1 时 motor1_test..motor4_test 必须恰好一个为 1"
#endif

#if (single_motor_debug == 0) && (DEBUG_SINGLE_SUM != 0)
#error "debug_config.h: motor1..4 已置 1 但 single_motor_debug=0, 请把模式也打开"
#endif

/* 比赛固件里不允许任何调试层, 也不允许蓝牙 */
#if (RUN_MODE == RUN_MATCH) && (DEBUG_LAYER_SUM > 0)
#error "debug_config.h: RUN_MODE==RUN_MATCH 时所有分层检测宏必须为 0"
#endif

#if (RUN_MODE == RUN_MATCH) && (DEBUG_VISION_ENABLE == 1)
#error "比赛模式禁止蓝牙调试(规则 FAQ3/4, R21): 请把 DEBUG_VISION_ENABLE 置 0 并拔掉 HC-05"
#endif

/* 会让电机/车运动的场景不能与 v1 的分层宏同时指挥电机 */
#if (DEBUG_VISION_ENABLE == 1) && \
    ((VISION_SCENE == VISION_SCENE_WHEEL_SPD) || \
     (VISION_SCENE == VISION_SCENE_LIFT_POS)  || \
     (VISION_SCENE == VISION_SCENE_STRAIGHT)  || \
     (VISION_SCENE == VISION_SCENE_SCRIPT))
#if (motor_open_debug == 1) || (motor_pid_debug == 1) || (lift_debug == 1) || \
    (chassis_debug == 1) || (script_debug == 1)
#error "debug_config.h: 可视化运动场景与分层运动宏会同时指挥电机, 只能留一个"
#endif
#endif

/* 可视化场景取值检查 */
#if (VISION_SCENE < VISION_SCENE_LINK) || (VISION_SCENE > VISION_SCENE_OBSERVE)
#error "debug_config.h: VISION_SCENE 取值非法"
#endif

#endif /* __DEBUG_CONFIG_H__ */