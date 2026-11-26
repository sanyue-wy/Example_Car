/**
 * @file    app_config.h
 * @brief   Warman39 竞赛小车 APP 层唯一调参入口
 *          硬件参数 / 赛道几何 / 升降高度 / 舵机角度 / 运动与策略参数
 *
 * @note    【填写规则】
 *          1. 本文件不含任何代码, 只有 #define。业务代码一律引用这里的宏。
 *          2. 未知参数写成 APP_CFG_UNSET, 文件末尾的守卫会让编译直接失败 ——
 *             宁可编不过, 也不要带着假数据上场比赛。
 *          3. 改机械参数只需要动本文件; 调控制参数在 app_config.h + 上位机
 *             (调试开启时)两处, 以本文件初值为准。
 *
 * @note    【本工程实际硬件基线】(已与用户确认, 见 README_APP.md)
 *          - MCU : STM32F103C8T6, 64K Flash / 20K RAM, HSE 8MHz x9 = 72MHz
 *          - 底盘: AT8236 四路串口驱动板 (USART2, 115200 8N1), 四轮差速
 *                  驱动板内部闭环速度环, 主控只下发轮速 mm/s —— 不是 PWM 直驱
 *          - 升降: 编码器直流电机 (TIM1_CH1 PWM 20kHz + PB0/PB1 方向 + TIM2 编码器)
 *                  经丝杆转成直线运动, 主控做位置闭环
 *          - 铲斗: 1 路舵机 (TIM3_CH1 / PA6, 50Hz)
 *          - 无陀螺仪: 航向只能靠四轮编码器里程计推算
 */

#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 0. 未填值的哨兵 + 编译期守卫
 *============================================================================*/
#define APP_CFG_UNSET   (-99999)

/* 把"宏是否为 APP_CFG_UNSET"在预处理期判出来 */
#define APP_CFG_IS_UNSET(x)   ((x) == APP_CFG_UNSET)

#define APP_CFG_REQUIRE_SET(macro_name)                                   \
    _Static_assert(!APP_CFG_IS_UNSET(macro_name),                        \
                   "app_config.h: " #macro_name " 仍是 APP_CFG_UNSET, 必须先填实测值")

/*==============================================================================
 * 1. 运行模式 (app.h 里做互斥检查, 这里只给枚举值)
 *============================================================================*/
#define RUN_MATCH   1   /* 比赛固件: 只跑比赛状态机, 任何调试层都必须为 0 */
#define RUN_DEBUG   2   /* 台架固件: 按 debug_config.h 选择要验证的分层       */

/*==============================================================================
 * 2. 底盘类型
 *     本工程驱动板是四轮差速(滑移转向), 不支持横移。
 *     任务书里的麦克纳姆轮方案在本车不可用 —— 脚本中不会出现 STRAFE。
 *============================================================================*/
#define CHASSIS_DIFF4WD   1
#define CHASSIS_TYPE      CHASSIS_DIFF4WD

/*==============================================================================
 * 3. 机械参数
 *============================================================================*/

/* 轮径(mm): JCA25-370-103K + 67mm 轮, 与 AT8236 默认配置一致。
 * 标定方法: chassis_debug 直行 1000mm, 实测走 1050mm -> 67 * 1000/1050 */
#define WHEEL_DIAMETER_MM        67.0f

/* 编码器线数 / 倍频 / 减速比: 驱动板侧的配置, 主控只用来算 CPR
 * (AT8236 自己完成编码器解析并上报 mm/s 与累计计数) */
#define ENCODER_PPR              11
#define ENCODER_MULT             4       /* TI12 四倍频 */
#define GEAR_RATIO               103.0f
#define ENCODER_CPR              ((uint32_t)((float)ENCODER_PPR * ENCODER_MULT * GEAR_RATIO))

/* 左右轮中心距(mm): 装车后实测, 用于里程计航向与原地转弧长换算。
 * 标定方法: chassis_debug 原地转 90deg x4, 实测角度偏差按比例修正 */
#define TRACK_WIDTH_MM           150.0f

/* 前后轴距(mm): 差速底盘不用, 保留给日志/后续扩展 */
#define WHEEL_BASE_MM            120.0f

/* ---- 四轮逻辑位置 -> AT8236 通道(实车接线) ----
 * 映射本体放在 chassis_map.h(调试层也包含同一份, 避免两处不一致):
 *   左前=M4, 左后=M3, 右前=M1, 右后=M2。
 * 换驱动板/重接线只改 chassis_map.h 里那一行, 运动学与里程计不用动。 */
#include "chassis_map.h"

/* ---- 升降丝杆 ---- */
#define LIFT_SCREW_LEAD_MM       2.0f    /* 丝杆导程: 电机输出轴一圈 -> 平台 mm */
#define LIFT_ENCODER_PPR         11
#define LIFT_ENCODER_MULT        4
#define LIFT_GEAR_RATIO          103.0f
#define LIFT_HAS_HOME_SWITCH     0       /* 0: 无下限位, 用堵转回零(低速小占空比) */

/* 升降行程(mm), 以回零后的零点为 0, 向上为正。
 * 取值来源: 现有 elevating.c 的 ELEVATING_MAX_MM = 200, 与丝杆实际行程一致;
 * 若换了长行程丝杆, 改这里并同步 elevating.c 的软限位。
 * @note 200mm < 中间平台台面 262mm, 所以本车默认够不到中间平台(见 §10)。 */
#define LIFT_TRAVEL_MAX_MM       200.0f

/*==============================================================================
 * 4. 引脚 / 定时器映射 (与 CubeMX 实际配置一一对应, 改动需同步 .ioc)
 *============================================================================*/

/* 底盘电机: 由 AT8236 驱动板经 USART2 控制, 主控不占 PWM/GPIO */
#define AT8236_UART_HANDLE       (&huart2)
#define AT8236_MOTOR_TYPE        AT8236_MOTOR_310   /* 310 电机; 520 需改 */
#define AT8236_ENCODER_LINES     11U
#define AT8236_GEAR_RATIO        103U
#define AT8236_DEADZONE          1600U

/* 升降电机: 编码器直流电机直驱丝杆 */
#define LIFT_ENC_TIM             (&htim2)      /* TIM2 Encoder_Interface (PA0/PA1) */
#define LIFT_PWM_TIM             (&htim1)      /* TIM1_CH1 PWM 20kHz (PA8) */
#define LIFT_PWM_CHANNEL         TIM_CHANNEL_1
#define LIFT_DIR_PORT            GPIOB
#define LIFT_DIR_PIN_FWD         GPIO_PIN_0   /* H 桥 IN1 */
#define LIFT_DIR_PIN_REV         GPIO_PIN_1   /* H 桥 IN2 */

/* 铲斗舵机: TIM3_CH1 (PA6) 50Hz。CubeMX 已配 PA6/PA7 为 TIM3_CH1/CH2 复用推挽 */
#define BUCKET_SERVO_TIM         (&htim3)
#define BUCKET_SERVO_CHANNEL     TIM_CHANNEL_1

/* 启动按键: PB12 上拉输入, 按下为低电平(GND)。
 * 【需要在 CubeMX 中补充】PB12 = GPIO_Input, Pull-up。
 * 未配置时可用 app.h 的 APP_START_KEY_FALLBACK 走"调试口触发"。 */
#define START_KEY_PORT           GPIOB
#define START_KEY_PIN            GPIO_PIN_12
#define START_KEY_ACTIVE_STATE   GPIO_PIN_RESET
#define START_KEY_DEBOUNCE_MS    30U

/* 结束指示 LED: PC13(板载蓝灯, 低电平点亮)。
 * 【需要在 CubeMX 中补充】PC13 = GPIO_Output, 初始高电平(灭), 推挽。
 * 若你的板子 PC13 不可用, 改这里即可。 */
#define STATUS_LED_PORT          GPIOC
#define STATUS_LED_PIN           GPIO_PIN_13
#define STATUS_LED_ON_STATE      GPIO_PIN_RESET   /* 低电平点亮 */

/* 蜂鸣器: 0 = 无(不编译蜂鸣器代码) */
#define BEEPER_ENABLE            0

/*==============================================================================
 * 5. 比赛时间 (R12 / G25 / R25)
 *============================================================================*/
#define MATCH_TIME_LIMIT_MS      120000U   /* 规则上限 120 s */
#define MATCH_SAFE_DEADLINE_MS   110000U   /* 到点强制收尾, 留 10 s 余量 */
#define MATCH_HOMING_TIMEOUT_MS   10000U   /* 启动后回零的最长允许时间, 超时判故障 */
#define FINISH_SETTLE_MS         300U      /* 全部禁用后等待再点亮 LED */
#define READY_LED_BLINK_MS       500U      /* 就绪慢闪 1 Hz */
#define FAULT_LED_BLINK_MS       100U      /* 故障快闪 5 Hz */

/*==============================================================================
 * 6. 赛道几何 (mm; 坐标系见 README_APP.md §赛道坐标, 实测后修改)
 *     原点 O: 起始区远离倾倒区的那个角
 *     +X 沿 2400 长边指向平台; +Y 沿 1200 短边指向倾倒区; theta=0 指向 +X
 *============================================================================*/
#define TRACK_X_MAX              2400.0f
#define TRACK_Y_MAX              1200.0f
#define GEOFENCE_MARGIN_MM       80.0f     /* 估计位置超出赛道外缘多少才判越界 */

/* 越界处理(P1-2):
 *   0 = 只记录, 不打断比赛(比赛模式默认)
 *   1 = 急停进 FAULT(调试模式默认)
 * 依据: 本车无陀螺仪, 航向只能靠四轮编码器推算, 滑移转向打滑时漂移可达数度,
 *       位置估计误差随之放大; 而 R24 允许车体伸出赛道外缘。若比赛时按估计值
 *       急停, 一次误判就整轮归零, 代价远高于"轻微越界"本身。
 *       调试模式下仍保留急停, 便于台架发现里程计标定错误。 */
#if (RUN_MODE == RUN_MATCH)
#define GEOFENCE_ACTION          0
#else
#define GEOFENCE_ACTION          1
#endif

/* 倾倒区 (30x30 DAR 围成, 允许接触: R22) */
#define DUMP_X_MIN               0.0f
#define DUMP_X_MAX               400.0f
#define DUMP_Y_MIN               800.0f
#define DUMP_Y_MAX               1200.0f

/* 下层平台区(基面上的划线区)与平台投影 */
#define PLATFORM_X_MIN           1600.0f
#define PLATFORM_X_MAX           2400.0f
#define PLATFORM_Y_MAX           595.0f
#define MID_PLATFORM_TOP_MM      262.0f    /* 中间平台台面高度 */
#define PLATFORM_SUPPORT_INSET_MM 75.0f    /* 四角支撑管距边 */

/* 起始区 / 终点区 */
#define START_ZONE_X_MAX         400.0f
#define FINISH_ZONE_X_MAX        625.0f

/* 出发位姿(车体中心, 朝向 +X) */
#define START_POSE_X_MM          200.0f
#define START_POSE_Y_MM          400.0f
#define START_POSE_THETA_DEG     0.0f

/*==============================================================================
 * 7. 升降高度 (mm, 相对回零零点)
 *============================================================================*/
#define LIFT_H_HOME              0.0f
#define LIFT_H_FLOOR             0.0f    /* 铲斗贴地取料 */
#define LIFT_H_CLEAR             45.0f   /* 越过倾倒区 30mm 墙 */
#define LIFT_H_CARRY             60.0f   /* 运输 */
#define LIFT_H_LEVEL1            20.0f   /* 放下层平台区 */
/* 放中间平台: 铲斗底需高于 MID_PLATFORM_TOP_MM(262) + 余量。
 * 注意: 该值必须 <= LIFT_TRAVEL_MAX_MM, 否则本车物理上够不到(守卫在 §11)。
 * 当前行程 200mm 时 LIFT_H_LEVEL2 不可达 -> STRATEGY_LEVEL2_ENABLE 必须为 0。 */
#define LIFT_H_LEVEL2            300.0f
#define LIFT_ARRIVE_TOL_MM       5.0f    /* L2 判定"真的升到位"的容差 */

/* 零点建立方式(P1-3)
 *   1 = 上电即零点: 设置阶段由队员把平台手动放到最底, 上电后当前位置记为 0,
 *       比赛中**不再回零**。G27 允许初始化阶段操作, 可省下最多 10s 比赛时间。
 *   0 = 启动后堵转回零(旧行为): 低速下行顶到机械下限, 停稳后清零。
 * @note 置 1 的前提是"每次上电前平台确实在最底"; 若平台停在高处上电,
 *       所有高度指令都会整体偏高一截, 且不会有任何报错。 */
#define LIFT_ZERO_AT_POWERUP     1

/*==============================================================================
 * 8. 舵机角度 (度; 0~270 舵机, 500us~2500us)。实测后修改。
 *============================================================================*/
#define BUCKET_ANG_STOW          20
#define BUCKET_ANG_SCOOP         120
#define BUCKET_ANG_CARRY         135
#define BUCKET_ANG_DUMP          200
#define BUCKET_MOVE_MS           400U     /* 舵机无反馈, 固定等待判到位 */

/* 调零角度: 270° 舵机的行程中点, 135° = 1500us。
 * 装摇臂/调零时让舵机停在这里, 机械位置装好后再改回正式角度。 */
#define BUCKET_ANG_ZERO          135
/* 舵机启动前的保持角 */
#define BUCKET_ANG_SAFE          BUCKET_ANG_STOW

/* 上电时 Bucket_Init() 立即输出的角度(之后一直保持, 等 Arm):
 *   调零阶段 -> BUCKET_ANG_ZERO (135°)
 *   正式固件 -> BUCKET_ANG_SAFE (上电就回到安全位)
 * 二选一, 另一行注释掉即可。 */
#define BUCKET_ANG_INIT          BUCKET_ANG_ZERO
/* #define BUCKET_ANG_INIT       BUCKET_ANG_SAFE */

/*==============================================================================
 * 9. 运动参数
 *============================================================================*/
#define CHASSIS_V_MAX_MMS        400.0f
#define CHASSIS_V_SLOW_MMS       150.0f
#define CHASSIS_ACC_MMS2         600.0f
#define CHASSIS_W_MAX_DPS        90.0f
#define CHASSIS_ACC_DPS2         360.0f
#define CHASSIS_POS_TOL_MM       5.0f
#define CHASSIS_ANG_TOL_DEG      1.5f
#define STALL_TIMEOUT_MS         800U     /* 有输出但编码器不动 -> 判堵转 */

/* 直线航向保持(编码器航向替代陀螺仪): 左右累计距离差 -> 角速度修正 */
#define HEADING_HOLD_KP          1.2f     /* deg/mm 误差 -> deg/s */
#define HEADING_HOLD_KD          0.0f
#define HEADING_HOLD_MAX_DPS     30.0f    /* 修正量限幅 */

/* 靠墙校正(只允许靠倾倒区 DAR 内壁, R22 允许接触; 禁止用赛道外缘 R23) */
#define SQUARE_WALL_DPS          40.0f
#define SQUARE_WALL_MAX_MM       120.0f
#define SQUARE_WALL_TIMEOUT_MS   3000U

/*==============================================================================
 * 10. 策略
 *============================================================================*/
/* 中间平台方案(每块 +10 而非 +5)。
 * 本车升降有效行程 200mm < 中间平台台面 262mm, 物理上够不到 -> 默认关闭,
 * 保底走 L1(下层平台区)。换长行程丝杆后:
 *   LIFT_TRAVEL_MAX_MM >= LIFT_H_LEVEL2 且本宏置 1, §11 的守卫会放行。 */
#define STRATEGY_LEVEL2_ENABLE   0
#define COLLECT_TRIPS            1   /* 铲斗一次装不下 6 块时改 2 */
#define APPROACH_GAP_MM          40.0f
#define REACH_MM                 120.0f  /* 铲斗伸到平台上方的前进距离 */
#define RETREAT_CLEARANCE_MM     100.0f  /* 结束时离平台/坡道/支撑管的最小距离 */

/* 收尾行为开关(规则解释待校内组织确认, 见 README_APP.md §未决问题) */
#define FINISH_PARK_LIFT         1   /* 1: 降到 HOME 后禁用升降电机 */
#define FINISH_HOLD_SERVO        1   /* 1: 舵机保持最后固定脉宽 */

/*==============================================================================
 * 11. 编译期守卫: 未填任何一项就直接编译失败
 *============================================================================*/
_Static_assert(!APP_CFG_IS_UNSET(CHASSIS_TYPE),       "app_config.h: CHASSIS_TYPE 未填写");
_Static_assert(!APP_CFG_IS_UNSET(WHEEL_DIAMETER_MM),  "app_config.h: WHEEL_DIAMETER_MM 未填写");
_Static_assert(!APP_CFG_IS_UNSET(TRACK_WIDTH_MM),     "app_config.h: TRACK_WIDTH_MM 未填写");
_Static_assert(!APP_CFG_IS_UNSET(LIFT_SCREW_LEAD_MM), "app_config.h: LIFT_SCREW_LEAD_MM 未填写");
_Static_assert(!APP_CFG_IS_UNSET(LIFT_TRAVEL_MAX_MM), "app_config.h: LIFT_TRAVEL_MAX_MM 未填写");

#if CHASSIS_TYPE != CHASSIS_DIFF4WD
#error "本车 AT8236 驱动板为四轮差速底盘, CHASSIS_TYPE 必须是 CHASSIS_DIFF4WD"
#endif

/* 数值类校验用 _Static_assert: C 预处理器的 #if 不接受浮点常量。
 *
 * 【编译器依赖】下面这些带浮点比较的 _Static_assert **不是标准的整型常量表达式**
 *   _Static_assert(WHEEL_DIAMETER_MM > 0.0f, ...);
 * C11 6.6 要求整型常量表达式, 浮点比较只有 GCC 扩展接受(Clang 也接受)。
 * 在 GCC/arm-none-eabi-gcc 上完全可用; 若换到严格模式的编译器(或开 -Wpedantic),
 * 这里会告警。真需要可移植时应改成整数比较(例如以 0.1mm 为单位定义参数)。
 * 保留现状是因为 F103 工程只用 GCC 工具链。 */
_Static_assert(LIFT_SCREW_LEAD_MM > 0.0f,                  "app_config.h: LIFT_SCREW_LEAD_MM 必须 > 0");
_Static_assert(TRACK_WIDTH_MM > 0.0f,                      "app_config.h: TRACK_WIDTH_MM 必须 > 0");
_Static_assert(WHEEL_DIAMETER_MM > 0.0f,                   "app_config.h: WHEEL_DIAMETER_MM 必须 > 0");
_Static_assert(MATCH_SAFE_DEADLINE_MS < MATCH_TIME_LIMIT_MS,
               "app_config.h: 安全截止时间必须小于比赛时间上限");
_Static_assert(READY_LED_BLINK_MS > 0U,                    "app_config.h: LED 闪烁周期必须 > 0");

/* L2 只能在整个机构够得到中间平台时启用 */
#if (STRATEGY_LEVEL2_ENABLE == 1)
_Static_assert(LIFT_TRAVEL_MAX_MM >= LIFT_H_LEVEL2,
               "app_config.h: 升降行程不够 LIFT_H_LEVEL2, 不能启用中间平台方案");
_Static_assert(LIFT_H_LEVEL2 > MID_PLATFORM_TOP_MM,
               "app_config.h: LIFT_H_LEVEL2 必须高于中间平台台面");
#endif

/* 舵机角度必须落在 0~270 内(整型宏, 可用 #if) */
#if (BUCKET_ANG_STOW < 0) || (BUCKET_ANG_STOW > 270)
#error "app_config.h: BUCKET_ANG_STOW 超出 0~270"
#endif
#if (BUCKET_ANG_SCOOP < 0) || (BUCKET_ANG_SCOOP > 270)
#error "app_config.h: BUCKET_ANG_SCOOP 超出 0~270"
#endif
#if (BUCKET_ANG_CARRY < 0) || (BUCKET_ANG_CARRY > 270)
#error "app_config.h: BUCKET_ANG_CARRY 超出 0~270"
#endif
#if (BUCKET_ANG_DUMP < 0) || (BUCKET_ANG_DUMP > 270)
#error "app_config.h: BUCKET_ANG_DUMP 超出 0~270"
#endif
#if (BUCKET_ANG_INIT < 0) || (BUCKET_ANG_INIT > 270)
#error "app_config.h: BUCKET_ANG_INIT 超出 0~270"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H__ */