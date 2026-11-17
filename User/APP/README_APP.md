# README_APP.md —— Warman39 小车 APP 层说明

> 本文件是 `User/APP` + 新增 Module/Device 层的**唯一运维手册**。
> 生成依据：`AGENT_TASK_Warman39_App层搭建.md`(v1) 与 `AGENT_TASK_Warman39_debug_vision.md`(v2)，
> 并已按**本工程实际硬件**调整（见 §1 与 §9 的差异说明）。

---

## 1. 硬件基线（已被代码固化）

| 项目 | 实际配置 | 证据 |
|---|---|---|
| MCU | STM32F103C8T6，64 KB Flash / 20 KB RAM，HSE 8 MHz ×9 = 72 MHz | `Example_Car.ioc`、链接脚本 |
| 底盘 | **AT8236 四路串口驱动板**，四轮差速；驱动板内部闭环，主控只下发轮速 mm/s | `User/Device/AT8236_driver/` |
| 底盘串口 | **USART2 (PA2/PA3)**，115200 8N1 | `usart.txt`、`at8236.c` |
| 打印串口 | **USART1 (PA9/PA10)**，115200 8N1 | `drv_uart.c` |
| 升降 | 编码器直流电机 → 丝杆；**TIM1_CH1 (PA8) PWM 20 kHz** + **PB0/PB1** 方向 + **TIM2 (PA0/PA1) 编码器** | `elevating.c` |
| 升降行程 | **200 mm**（丝杆导程 2 mm/圈，4532 计数/输出轴圈） | `elevating.c` |
| 铲斗 | 1 路舵机，**TIM3_CH1 (PA6)**，50 Hz（按用户确认从 TIM1 改到 TIM3） | `bucket.c` |
| 陀螺仪 | **无** → 航向只能靠四轮编码器里程计 | 用户确认 |
| 启动键 | **PB12**，上拉输入，按下接 GND —— **CubeMX 尚未配置** | §3 待补清单 |
| 结束 LED | **PC13**，低电平点亮 —— **CubeMX 尚未配置** | §3 待补清单 |
| 蓝牙 | HC-05 接 **USART1**（与打印同口，按用户指定）| §6 |

### 1.1 外设占用表

| 外设 | 用途 | 使用方 | 备注 |
|---|---|---|---|
| USART1 | 文本打印 / 蓝牙可视化 | `app_log.c` 或 `debug_vision.c` | **二者互斥**，见 §6 |
| USART2 | AT8236 驱动板 | `at8236.c` | 空闲中断接收 |
| USART3 | **未使用** | — | PB10/PB11 空闲；P1-5 未采纳（蓝牙仍走 USART1） |
| TIM1 CH1 | 升降电机 PWM 20 kHz | `encoder_motor.c` | PA8 |
| TIM2 | 升降编码器（TI12 ×4） | `drv_encoder.c` | PA0/PA1，**唯一一路硬件编码器** |
| TIM3 CH1 | 铲斗舵机 50 Hz | `bucket.c` | PA6 |
| TIM4 | 系统 1 ms 节拍 | `drv_tim.c` | 控制环 5 ms 分频 |
| GPIOB PB0/PB1 | 升降 H 桥 IN1/IN2 | `encoder_motor.c` | |
| GPIOB PB12 | 启动键 | `app.c` | **待 CubeMX 配置** |
| GPIOC PC13 | 结束指示 LED | `app.c` | **待 CubeMX 配置** |
| SysTick | HAL 时间基 | HAL | 1 ms |

> **说明**：任务书假设的"5 路独立编码器 + 4 路 PWM 直驱"在本硬件上不存在。
> 本工程的对应关系是：4 个轮子 → 驱动板 4 路(M1 左前 / M2 左后 / M3 右前 / M4 右后)，
> 第 5 路（升降）→ TIM1+TIM2。参见 §9 的逐条差异表。

---

## 2. 目录与分层

```
User/
├── APP/
│   ├── app_config.h       ★ 唯一调参入口(硬件参数/赛道/高度/角度/策略)
│   ├── app.h / app.c      比赛状态机 + 动作脚本执行器 + 安全监控
│   ├── app_script.h / .c  L1/L2 动作脚本(纯数据表)
│   ├── app_log.h / .c     极简整数打印(避免 vsnprintf 撑爆 64K Flash)
│   ├── task_it.h / .c     周期任务注册(1 ms 节拍 → 5 ms 控制环)
│   ├── debug.h / debug.c  分层自检(宏开关)
│   ├── debug_config.h     ★ 调试开关 + 编译期互斥检查
│   ├── debug_vision.h/.c  蓝牙可视化(HC-05 + bt-debug-visualizer)
│   └── .legacy_v1_backup/  **旧的"智能分拣机"APP 层代码, 不参与编译**
│                         (依赖 host_comm/motion_sys_XY/tray 等不存在的头文件)
├── Device/
│   ├── AT8236_driver/     四路串口驱动板(复用)
│   ├── Encode_motor/      编码器电机(复用)
│   ├── DC_Motor/          ★ 新增: 执行器统一抽象(使能/急停/堵转)
│   ├── servo/             舵机(分拣机遗留, 本车不用 —— 见 §9)
│   └── Tool/bt_vision/    btdbg 协议库 + 25 项参数表(原样保留)
├── Middleware/
│   ├── Driver/{TIM,PWM,UART,ENCODER}   复用, 未改动
│   └── Algorithm/Controller/pid.c      复用
└── Module/
    ├── Chassis/chassis.c           复用(底盘速度/里程计)
    ├── Chassis/chassis_motion.c    ★ 新增: 非阻塞直线/转角 + 航向保持
    ├── Elevating/elevating.c       复用(位置环/软限位)
    ├── Lift/lift.c                 ★ 新增: 回零流程 + 高度指令
    └── Bucket/bucket.c             ★ 新增: 铲斗舵机
```

### 2.1 仓库里存在但**不参与编译**的源文件

以下文件仍在磁盘上，但 `CMakeLists.txt` 没有收录它们。**不要**误以为它们生效了：

| 文件 | 原因 |
|---|---|
| `User/Module/Grip/grip.c`、`grip_debug.c` | 夹爪机构属另一台车；且 `grip.c` 内部调 `Servo_Init()`，会抢占 TIM1（与升降 PWM 冲突） |
| `User/Device/servo/servo*.c` | 同上，硬绑 TIM1；`bucket.c` 已用 TIM3 替代 |
| `User/Middleware/Algorithm/Controller/td.c` | 跟踪微分器，本工程未使用（无调用方） |
| `User/Device/Tool/Debug_vofa/vofa*.c` | VOFA+ 上位机协议，已被 btdbg 可视化取代 |
| 各 `*_debug.c`（`chassis_debug.c`、`elevating_debug.c`、`drv_uart_debug.c`、`drv_encoder_debug.c`、`at8236_debug.c`、`encoder_motor_debug.c`、`bt_vision_debug.c`） | 旧的分层自检实现，已由 `User/APP/debug.c` 统一接管 |

这些文件互相之间有引用（例如 `grip.c` 引用 `servo.h`），但没有任何一个被 APP 层引用，
所以删除其中任意一个都不会影响本工程编译。若要彻底清理，建议整目录删除而不是逐个删 .c。

---

## 3. CubeMX 待补清单（**必须先做，否则比赛固件会主动进故障态**）

当前 `.ioc` 里 **没有** PB12 / PC13 的配置。按任务书 §0.2 的规定，我没有手改
`MX_GPIO_Init()`，请你在 CubeMX 中补上后重新生成：

| 步骤 | 引脚 | 配置 | 说明 |
|---|---|---|---|
| 1 | **PB12** | `GPIO_Input`，Pull-up，Label=`START_KEY` | 启动键，按下接 GND |
| 2 | **PC13** | `GPIO_Output`，Output Level=High，Push-Pull，Low speed，Label=`STATUS_LED` | 板载蓝灯，低电平点亮 |
| 3 | 生成代码（GCC / CMake） | — | 只需 `MX_GPIO_Init()` 的 USER CODE 区不动 |

另外确认以下项（当前 `.ioc` 已满足，但数值需核对）：

- **USART2**：115200 8N1，**全局中断使能**（AT8236 驱动板）
- **USART1**：115200 8N1，全局中断使能（打印/蓝牙）
- **TIM3 CH1 (PA6)**：PWM 复用推挽。**CubeMX 里的 PSC=71/ARR=19999（20 kHz）不用改**：
  `drv_pwm` 在首通道接入时会按定时器实际时钟重算时间基，50 Hz 下自动落成
  PSC=21 / ARR=65453（实际 50.0004 Hz）；CubeMX 配的只是上电到 `Bucket_Init()` 之前的临时值。
  真正需要保证的只有"PA6 是 TIM3_CH1 且为 AF 推挽"。
  若把 TIM3 挪作他用，改 `app_config.h` 的 `BUCKET_SERVO_TIM/CHANNEL`。
- **TIM2**：Encoder_Interface，TI12 四倍频（升降编码器）
- **TIM1 CH1 (PA8)**：PWM 复用推挽（升降电机）。注意 `User/Device/servo/servo.c`
  **硬绑 TIM1 CH1/CH2/CH3**，与升降 PWM 冲突 —— 本工程从不调用 `Servo_Init()`
  （铲斗走 `bucket.c` + TIM3），保留该文件只为不破坏既有目录结构；
  **不要**在 APP 层调用 `Servo_Init()`/`Grip_Init()`。

> 若暂时没有 PB12/PC13，可在 `app.h` 把 `APP_START_KEY_FALLBACK` 置 1，
> 用"USART1 收到任意字节"触发启动。这**只允许 RUN_DEBUG**；`RUN_MATCH` 下会直接 `#error`。

---

## 4. 比赛流程

```
上电 ──► APP_BOOT ──► APP_READY ──[启动键按下沿, 消抖 30ms]──► APP_RUN ──► APP_FINISH
                         │  LED 慢闪 1Hz            │ 先回零(≤10s)   │  LED 常亮
                         │  所有执行器禁用           │ 再跑脚本         │  电机全部禁用
                         └──────────────► APP_FAULT ◄─┘                │
                                           LED 快闪 5Hz, 急停            └─ 永久停留
```

| 状态 | 执行器 | 安全约束 |
|---|---|---|
| `APP_READY` | 电机禁用、底盘 `$spd:0`、升降占空比 0、舵机保持安全角 | R11/R14/G27：启动前不执行功能 |
| `APP_RUN` | 先回零（升降被占用，不跑脚本），再按脚本执行 | 110 s 硬截止；堵转/升降故障/越界 → FAULT |
| `APP_FINISH` | 分阶段推进（每圈一步）：停底盘 → 升降回 HOME → 等停稳 → 禁用电机 → 静置 → 完成 | R25/R26：**LED 只在电机禁用后点亮** |
| `APP_FAULT` | 立即 `DC_Motor_EmergencyStopAll()`，舵机断开 | 单次触发，不自动复位 |

### 4.1 动作脚本（`app_script.c`，全部为数据表）

`COLLECT → TRAVEL → (PLACE_L2 | PLACE_L1) → RETREAT`

- `STRATEGY_LEVEL2_ENABLE = 0`（默认）：走 `PLACE_L1`（下层平台区，+5/块）
  - **原因：升降有效行程 200 mm < 中间平台台面 262 mm，本车物理上够不到中间平台。**
  - 换长行程丝杆后把 `LIFT_TRAVEL_MAX_MM` 与 `LIFT_H_LEVEL2` 改对、把本宏置 1，
    `app_config.h` §11 的守卫会放行；L2 未到位会自动跳到 L1（`ACT_CHECK_LIFT`）。
- 带 `/* TUNE */` 注释的距离/角度全部是占位值，必须实测（见 §5 调参顺序）。

---

## 5. 调参顺序（严格按下面的顺序做，每步通过再下一步）

台架调试前，先把 `app.h` 的 `RUN_MODE` 保持为 `RUN_DEBUG`，然后在
`debug_config.h` 里**一次只打开一层**。

| 步骤 | 打开的宏 | 做什么 | 判据 / 要改的参数 |
|---|---|---|---|
| 1 | `uart_debug` | 串口助手 115200 发字节 | 应原样回显 |
| 2 | `encoder_debug` | **手转丝杆**（不供电给电机） | 计数随转动变化；正转符号与 `ELEVATING_UP_SIGN` 一致，否则改 `elevating.c` 顶部 |
| 3 | `motor_open_debug` | **车架空**，四轮 +200/-200/0 | 四轮同向、同速；反了就改 `chassis.c` 的 `invert_mask`（**不要改接线**） |
| 4 | `motor_pid_debug` | 车架空，100/200/300 mm/s 阶跃 | 稳态误差与超调可接受；驱动板内环参数用板子上位机 `$mpid` 调 |
| 5 | `servo_debug` | 铲斗四姿态循环 | 记录 STOW/SCOOP/CARRY/DUMP 的实际角度，回填 `app_config.h` |
| 6 | `lift_debug` | 回零 → LEVEL1 → HOME | 打印到位误差应 < 1 mm；回零失败先查 TIM2/PB0/PB1 |
| 7 | `chassis_debug` | 直行 1000 mm、原地转 90°×4 | 用卷尺量：新轮径 = 67 × 1000 / 实测；转角误差按比例修正 `TRACK_WIDTH_MM` |
| 8 | `script_debug` | 按启动键后跑最小脚本 | 确认按键、舵机、底盘联动正常 |
| 9 | `DEBUG_VISION_ENABLE=1` + 场景 | 蓝牙在线调参 | 见 §6 |

### 5.1 关键调参位置速查

| 参数 | 位置 | 影响 |
|---|---|---|
| 轮径 / 轮距 / CPR | `app_config.h` §3 | 里程计与转角精度 |
| 直线速度 / 加速度 / 角速度 | `app_config.h` §9 | 打滑、超调 |
| 航向保持 P/D | `app_config.h` §9 `HEADING_HOLD_*` | 直线跑偏 |
| 堵转阈值时间 | `app_config.h` §9 `STALL_TIMEOUT_MS` | 误报/漏报 |
| 升降高度点 | `app_config.h` §7 | L1/L2/运输/越墙 |
| 舵机四姿态角 | `app_config.h` §8 | 取料/承托/倾倒 |
| 脚本距离/角度 | `app_script.c`（标了 `TUNE`） | 路径与停靠 |
| 升降 PID | `elevating.c` 顶部（或蓝牙 `POS_*`） | 升降超调 |
| 底盘驱动板 PID | 板子上位机 `$mpid` | 轮速响应 |

---

## 6. 蓝牙可视化（v2）使用说明

### 6.1 接线与开关

1. HC-05 的 TX/RX **交叉**接到 **PA10(USART1_RX) / PA9(USART1_TX)**，共地。
2. HC-05 出厂 9600，先用 AT 模式 `AT+UART=115200,0,0`。
3. `app.h` 保持 `RUN_MODE = RUN_DEBUG`；`debug_config.h` 置 `DEBUG_VISION_ENABLE = 1`；
   选 `VISION_SCENE`。
4. 上位机 `bt-debug-visualizer` 打开对应串口。

### 6.2 串口共用（**与任务书 v2 §0.5 的差异，必须了解**）

任务书要求蓝牙用**独立**串口，但本机只有 USART1/USART2 可用（USART3 未开、USART2 属驱动板），
按你的指定接了 USART1。代码里的处理：

- `DEBUG_VISION_ENABLE == 1` 时，`app_log.c` 整个文件变成空实现 → 串口上**只有** COBS 二进制帧；
- 因此**打开蓝牙时不能用串口助手看 `[PASS]/[FAIL]` 文本**，两者不能同时用。

> 如果以后把 USART3 打开，只需改 `app_config.h` 加一句串口宏并让 `debug_vision.c`
> 用 `UART_Port_Init` 绑定 USART3，即可恢复"独立串口"。

### 6.3 场景与安全

| 场景 | 动作 | 触发 | 硬上限 |
|---|---|---|---|
| V1 `LINK` | 只发遥测波形 | 上电即发 | — |
| V2 `WHEEL_SPD` | 四轮按 `TGT_SPEED` 转 | **必须按 PB12** | 15 s |
| V3 `LIFT_POS` | 回零后在 0/LEVEL1 阶跃 | **必须按 PB12** | 15 s |
| V4 `STRAIGHT` | 直行 1000 mm | **必须按 PB12** | 15 s |
| V5 `SCRIPT` | 同上（最小脚本） | **必须按 PB12** | 15 s |
| V6 `OBSERVE` | 完全不动，推车看里程计 | — | — |

- **没有任何"上位机发一帧就让车动"的路径**（任务书 §0.8 / FAQ3）。
- 故障时 `btdbg_set_state(FAULT, reason)`，reason：1 堵转 / 2 升降超程 / 3 越界 / 4 无反馈。
- 遥测只发 `CH_MOTOR`（四轮速度 + 升降高度）与 `CH_CAR`（速度/里程/航向），**不发 `CH_GYRO`**。

### 6.4 参数在本车上的含义（25 项不变，仅改变解释）

| 参数 | 本车含义 | 是否真的生效 |
|---|---|---|
| `POS_*` | **升降位置环**增益 | ✅ 写入 `Elevating.pos_pid` |
| `SPD_*` | **升降速度环**（底盘速度环在驱动板内，主控没有） | ✅ 写入 `Elevating.motor.pid` |
| `YAW_*` | 航向保持 P/D | ⚠️ 本版只记标志，**未热更新**（编译期常量） |
| `WHEEL_DIA` / `ENC_CPR` | 轮径 / 每转计数 | ⚠️ 仅 IDLE 可写，**下次上电生效** |
| `TGT_SPEED` / `ACC_LIMIT` | 可视化场景速度 | ✅ 场景内使用 |
| `GYRO_*` / `CG_OFFSET` | 本车无陀螺仪 | ❌ 不读取 |

> **比赛时必须**：`DEBUG_VISION_ENABLE = 0` **并且拔掉 HC-05**。
> `RUN_MODE == RUN_MATCH && DEBUG_VISION_ENABLE == 1` 会在编译期 `#error`。

---

## 7. P4 静态自查清单

| # | 检查项 | 结论 | 依据 |
|---|---|---|---|
| 1 | 上电到按启动键前，没有任何路径让电机/舵机输出 | ✅ | `App_Init` 只 `Init` 不 `Enable`；`Bucket_Init` 只写安全角；底盘 `Chassis_Init` 下发 `hold` |
| 2 | 只有一个启动输入，只响应一次 | ✅ | `App_StartKeyPressed()` 仅报按下沿 + `start_consumed` 锁存 |
| 3 | 结束 LED 只在所有电机禁用之后点亮 | ✅ | `App_DoFinishSequence()` 的 `FIN_DISABLE` 之后才到 `FIN_DONE` |
| 4 | 110 s 强制收尾存在且不依赖脚本完成 | ✅ | `App_SafetyCheck()` 首条检查 `MATCH_SAFE_DEADLINE_MS` |
| 5 | 比赛模式下没有任何串口/USB 接收会改变运动 | ✅ | `RUN_MATCH` 时 `debug.c` 整段不编译；`APP_START_KEY_FALLBACK` 强制为 0 |
| 6 | 中断回调里没有 `HAL_Delay` / `printf` / 阻塞循环 | ✅ | `App_ControlTick` 只做闭环与寄存器写；btdbg 发送只在主循环 |
| 7 | 所有新 `.c` 在 CMakeLists，所有新目录在 include 列表 | ✅ | `CMakeLists.txt` |
| 7b | 启动键未配置时不会"凭空启动" | ✅ | `App_InitDrivers` 直读 `GPIOB->CRH[19:16]`，非上拉输入 → `s_hw_missing` → MATCH 下直接 FAULT |
| 8 | `app_config.h` 无遗留 `APP_CFG_UNSET` | ✅ | §11 用 `_Static_assert` 守卫；另加 6 条数值校验 |
| 9 | L2 不到位自动转 L1，单步超时都有去向 | ✅ | `ACT_CHECK_LIFT` 返回非 0 → `ON_TO_FALLBACK` → `SEG_PLACE_L1` |
| 10 | 升降必须先回零再跑脚本 | ✅ | `App_BeginHoming()`，回零期间 `homing_wait` 阻止脚本推进 |

### 7.1 编译矩阵（已全部实测）

| 配置 | 结果 |
|---|---|
| `RUN_MATCH` + 全部调试关闭 | ✅ 链接通过，FLASH 57200 B（87.3%） |
| `RUN_MATCH` + `DEBUG_VISION_ENABLE=1` | ✅ 按设计 `#error`（debug_config.h:96） |
| `RUN_MATCH` + `APP_START_KEY_FALLBACK=1` | ✅ 按设计 `#error`（app.h:43） |
| 任意两个分层宏同时开 | ✅ 按设计 `#error`（debug_config.h:87） |
| 8 个分层宏逐个单开 | ✅ 全部通过 |
| `VISION_SCENE` = LINK/WHEEL_SPD/LIFT_POS/STRAIGHT/SCRIPT/OBSERVE | ✅ 6 个场景全部链接通过（49.3~50.4 KB），零告警 |
| `DEBUG_VISION_ENABLE=0` 时产物无 `btdbg_` 符号 | ✅ `arm-none-eabi-nm` 计数 = 0 |
| Debug / Release 零告警（含 6 个蓝牙场景） | ✅ `-Wall -Wextra` 下无任何 warning |

---

## 8. 构建与内存

```bash
cmake --preset Debug   && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
```

| 构建 | FLASH | RAM |
|---|---|---|
| Debug（台架，`RUN_DEBUG`） | 45.7 KB / 64 KB（69.8%） | 4.6 KB / 20 KB（22.7%） |
| Debug（比赛，`RUN_MATCH`） | 57.8 KB / 64 KB（88.2%） | 4.7 KB / 20 KB（23.0%） |
| Release（台架，`RUN_DEBUG`） | 38.6 KB / 64 KB（58.9%） | 4.6 KB / 20 KB（22.7%） |
| **Release（比赛，`RUN_MATCH`）← 推荐上场** | **50.6 KB / 64 KB（77.3%）** | 4.7 KB / 20 KB（23.0%） |

> **Flash 余量**：Debug+`RUN_MATCH` 占 88.2%（57.8 KB），Release+`RUN_MATCH` 只占 77.3%
> （50.6 KB）。**比赛请烧 Release**，Debug 版本只用于台架单步。
> BOM 差异来自 AT8236 驱动内的 `strtof`：它把 newlib 的浮点解析
> （`_strtod_l`/`__gethex`）拉进来。若要更多余量，可把 `at8236.c` 的 `strtof`
> 换成定点解析（当前未做，因为任务书 §0.8 要求不得重写能工作的驱动）。
>
> `CMakeLists.txt` 给 Debug 构建加了 `-Os`：工具链默认的 `-O0` 在 64 KB 上
> **放不下比赛固件**（实测 69.4 KB，超出 3.8 KB）。`-g3` 保留，单步调试不受影响。

### 8.1 上场前的构建步骤

```bash
# 1. 把 app.h 的 RUN_MODE 改成 RUN_MATCH
# 2. 确认 debug_config.h 里 8 个分层宏 + DEBUG_VISION_ENABLE 全为 0
# 3. 确认已拔掉 HC-05
# 4. （P1-3）确认 LIFT_ZERO_AT_POWERUP=1 时，上电前平台已在最底
cmake --preset Release && cmake --build --preset Release
# 产物: build/Release/Example_Car.elf
# 本工程未配置 hex/bin 后处理; 烧写直接选 .elf (STM32CubeProgrammer / ST-Link 都可)
```

烧写后上电应看到 PC13 **慢闪 1 Hz**（READY）。若**快闪 5 Hz**，说明进了 FAULT，
先用 Debug 版看串口日志里的 `[app] FAULT code=` / `HARDWARE MISSING`。

---

## 9. 与任务书的差异（逐条说明，均已按实际硬件调整）

| 任务书条目 | 任务书假设 | 本工程实际 | 处理方式 |
|---|---|---|---|
| §2.1 / §5.3 底盘 | 4 路 PWM 直驱 + 4 路编码器，主控做轮速 PID | AT8236 串口板内闭环 | 复用 `chassis.c` + 适配器；`dc_motor.c` 只做使能/急停/堵转 |
| §5.2 `drv_encoder` | 新增 5 路 | 仅 TIM2 一路硬件编码器 | 复用现有 `drv_encoder.c`，只服务升降 |
| §5.5 `lift` | 位置环 + 限位 | 已有 `elevating.c` | 新增 `lift.c` 在其上加回零/高度指令，不重写 |
| §5.6 `bucket` | 基于 `servo.h` | `servo.c` 硬绑 TIM1_CH1/2/3，且 TIM1 冲突 | 新增 `bucket.c` 直接用 `drv_pwm` 绑 TIM3_CH1 |
| §5.8 `app.c` | 5 路电机速度环 | 见上 | 状态机/脚本/安全监控按任务书结构重写 |
| §5.9 调试层 3/4 | 逐个电机开环 / 速度环阶跃 | 同左 | 改为"驱动板开环指令 / 驱动板速度阶跃" |
| §5.4 麦轮 `STRAFE` | `ACT_STRAFE` | 差速底盘不支持横移 | 脚本中**不出现** `ACT_STRAFE` |
| §5.8.4 L2 中间平台 | 升降行程 ≥ 300 mm | **行程仅 200 mm** | `STRATEGY_LEVEL2_ENABLE = 0`，默认 L1 |
| v2 §0.5 蓝牙串口 | 必须独立串口 | 只剩 USART1 | 按你指定用 USART1 + 文本打印编译期关闭（§6.2） |
| v2 §2 目录 | `User/Middleware/Btdbg/` | 库已在 `User/Device/Tool/bt_vision/` | **原地复用，未搬动**（避免破坏已被引用的路径） |
| — 旧 APP 层 | — | `app.c` 是另一台"智能分拣机"的代码 | 备份到 `User/APP/.legacy_v1_backup/`，不参与编译 |

---

## 10. 未决问题（需要你/队伍确认或实测）

1. **PB12 / PC13 的 CubeMX 配置**（§3）—— 不做则比赛固件直接进 `APP_FAULT`。
   固件会检测 PB12 的 `CRH[19:16]`：若不是上拉输入（0b1000/0b1010）就判为未配置，
   避免悬空引脚随机读数把车放出去。
2. **升降行程**：现有 `elevating.c` 软限位 200 mm，是否与实际丝杆一致？
   若换了 >262 mm 的丝杆，L2 方案可启用（每块多 5 分）。
3. **舵机四姿态角**（`BUCKET_ANG_*`）目前是估计值，必须实测后回填。
4. **轮距 `TRACK_WIDTH_MM = 150`**：`chassis.c` 注释里的默认值，需实测
   （原地转 90°×4 用 `chassis_debug` 标定）。
5. **脚本中的路径距离**：所有 `/* TUNE */` 项需按实际赛道实测。
6. **规则解释待确认**（任务书 §5.8.5）：
   - "结束后舵机保持固定脉宽"是否算"停止所有功能"？
     已做成宏开关 `FINISH_HOLD_SERVO`（默认 1 = 保持）。
   - "结束后升降回 HOME 并禁用电机"是否算"停止"？
     已做成宏开关 `FINISH_PARK_LIFT`（默认 1 = 回 HOME 并禁用）。
7. **蜂鸣器**：`BEEPER_ENABLE = 0`，本车未确认有无蜂鸣器。
8. **L2 降级路径的实测**：默认关闭 L2 后该分支不会被走到，
   启用 L2 前需单独验证 `ACT_CHECK_LIFT` → `SEG_PLACE_L1` 的跳转。
9. **`YAW_*` 参数热更新未实现**：航向保持在 `chassis_motion.c` 里是编译期常量；
   若需在线调，需要在 `chassis_motion` 增加 setter。
10. **`WHEEL_DIA` / `ENC_CPR` 热更新**：目前只接受 IDLE 写入，下次上电生效。
11. **AT8236 驱动板的 `$upload` 上报**：`App_InitDrivers()` 现在会主动下发
    `$upload:1,0,1#`（累计编码器 + 速度，不开 delta）。
    **如果实车出现"直行动作永远走不完"或一启动就报堵转 FAULT**：
    先确认驱动板波特率/接线，再确认板子固件是否接受这条指令
    （串口应回 `uploadOK`）。不开上报时 `Chassis` 的反馈恒为 0，
    里程计不走、堵转检测会判"有指令无反馈"。
12. **`CMOTION_LAG_S = 0.06f`**（P0-4 指令延迟补偿）：需实车标定。
    标定方法：`chassis_debug` 直行 1000 mm @400 mm/s 各 5 次，
    若终点误差系统性偏大（冲过头）就加大；偏小（没走够）就减小。
13. **`LIFT_ZERO_AT_POWERUP = 1`**（P1-3）：每次上电前**必须**把平台放到最底。
    若平台停在高处上电，所有高度指令都会整体偏高且**不会报错**。
    不放心就改回 0，用堵转回零（多花最多 10 s 比赛时间）。
14. **`GEOFENCE_ACTION`**（P1-2）：比赛模式默认 0（只记录），调试模式默认 1（急停）。
    采用的取舍见 `app_config.h` 中的长注释：无陀螺仪时位置估计误差大，
    一次误判就整轮归零，代价高于轻微越界本身。**需校内规则确认 R24 的解释。**

---

## 11. v3 审查修复记录（提交 b1a5930 之后的改动）

> 依据 `AGENT_TASK_Warman39_修复v3.md`，按 P0→P1→P2 逐项修复。
> 每项只改必要代码，未做重构。验证方式写在每项末尾。

### P0-1 急停/停车指令可能丢失 ✅

**改动的函数**：`Chassis_Hold` / `Chassis_Coast` / `Chassis_Task` / `Chassis_SetEnabled`（新增）/
`DC_Motor_EmergencyStopAll` / `DC_Motor_Enable`

- 运行期发送**只在 `Chassis_Task`（中断上下文）**发生。`Chassis_Hold/Coast` 改为只置
  `hold_pending` / `coast_pending` 标志与模式，不碰串口。
- `Chassis_Task` 开头先处理 pending：**发送成功才清标志**，失败下一拍重试；
  处理 pending 时直接 `return`，保证停车帧优先于速度帧。
- `CHASSIS_MODE_HOLD` 期间按 `command_period_ms`（50 ms）**周期重发 `$spd:0`**，
  驱动板丢一帧也会在 50 ms 内被纠正。
- `STOPPING` 降速到 0 后不再当场发送，改为置 `hold_pending` 走同一通道。
- 初始化期（`Chassis_Init` 里的那次 hold）仍是直接发送 —— 那时中断节拍还没起来，无竞争。

**验证**：`motor_open_debug` 运行中，主循环每 37 ms 调一次
`DC_Motor_EmergencyStopAll()` → `DC_Motor_ClearFaults()` → `DC_Motor_Enable(FL,true)`，
连续 100 次，驱动板必须 100/100 停下；`chassis_stats.cmd_busy` 增长不再导致停车失败。

### P0-2 升降回零必然触发 RANGE 故障 ✅

**改动的函数**：`Lift_UpdateFault` / `Lift_Tick`（清零分支）

- 回零的 `LIFT_HOMING_DOWN` / `LIFT_HOMING_SETTLE` 期间**跳过** RANGE 检查。
- RANGE 只在 `ELEVATING_STATE_MOVING` 时生效（静止时编码器抖动不升级为故障）。
- `Elevating_SetZero()` 之后清 `LIFT_FAULT_RANGE` 位。
- `LIFT_FAULT_TIMEOUT` / `LIFT_FAULT_MOTOR` 不受影响，仍然任何状态都检查。

**验证**：`lift_debug` 中先手动把平台摇到 50 mm 处再上电，回零 → LEVEL1 → HOME
全程 `Lift_GetFault() == 0`。

### P0-3 底盘动作超时被当成"完成" ✅

**改动的函数**：`ChassisMotion_Succeeded`（新增）/ `App_StepDone` / `App_RunScript`

- 新增 `ChassisMotion_Succeeded()`，只对 `CMOTION_DONE` 返回 true；
  `ChassisMotion_IsDone()` 语义保持"已停止"（含 TIMEOUT/FAULT）。
- `App_StepDone` 的 `ACT_MOVE/ACT_TURN` 改用 `Succeeded()`。
- `App_RunScript` 增加一条：底层动作已结束但未成功 → 按超时处理，
  直接进入该步的 `on_timeout`（`ON_TO_FALLBACK` / `ON_TO_FINISH`）。

**验证**：脚本里放一步 `ACT_MOVE 3000 mm, timeout 1000 ms, ON_TO_FINISH`，
必须在 1 s 后进入 `APP_FINISH`，而不是继续下一步。

### P0-4 直线/转向没有减速段 ✅

**改动的函数**：`ChassisMotion_Tick`（两个分支）/ `CMotion_Finish` /
`ChassisMotionReport_t`（新增 `error` 字段）/ `chassis_debug` 的打印

- 直线：`v_cmd = min(s_speed, sqrtf(2 * CHASSIS_ACC_MMS2 * remaining))`，
  再减去 `v_cmd * CMOTION_LAG_S` 作为指令延迟补偿，下限 `CMOTION_MIN_V`。
- 转向：同式，用 `CHASSIS_ACC_DPS2` 与 `CMOTION_MIN_W`。
- 到位改用 `Chassis_Hold`（P0-1 版本），不再用带斜坡的 `Chassis_Stop`。
- `report.error = target - achieved`，`chassis_debug` 每步都打印。

**验证**：`chassis_debug` 直行 1000 mm @400 与 @150 mm/s 各 5 次，
卷尺量终点误差 < 15 mm；原地转 90° 误差 < 3°（编码器口径）。

### P0-5 转角 ≥180° 走不完 ✅

**改动的函数**：`ChassisMotion_Tick`（TURN 分支）/ `CMotion_Begin`

- 新增 `s_turn_accum` / `s_last_yaw`：每拍 `s_turn_accum += WrapDeg(yaw - yaw_prev)`，
  用**未折返的累积值**判断进度与剩余量，不再用折回到 ±180° 的差值。
- `chassis_debug` 增加 `+270°` / `-270°` 两步专门验证。

**验证**：`chassis_debug` 原地转 +270° 与 −270° 各一次，都能正常 `DONE`。

### P1-1 比赛中的 FAULT 不应"快闪等待" ✅

**改动的函数**：`App_Run` 的 `APP_FAULT` 分支 / `App_DoFaultFinish`（新增）/
`App_SetState`

- `RUN_MODE == RUN_MATCH`：FAULT = 急停 → 等停稳 → 禁用 → 静置 `FINISH_SETTLE_MS`
  → **LED 常亮**（与正常结束一致）。跳过 `FIN_PARK_LIFT`：故障时升降可能已不可信。
- `RUN_DEBUG` 保持快闪 5 Hz，便于现场区分。
- 故障码仍保存在 `s_app.fault`（RAM），赛后用 Debug 版读。

**验证**：MATCH 版制造一次人为故障（例如拨开启动键强行进 RUN 后断电编码器），
LED 应在 `FINISH_SETTLE_MS` 后常亮而不是一直闪。

### P1-2 Geofence 误报 ✅

**改动的函数**：`ChassisMotion_PoseInField` / `App_SafetyCheck` /
`AppContext_t`（新增 `geofence_warned`）

- 新增 `GEOFENCE_ACTION`：比赛 = 0（只记录一次，不打断），调试 = 1（急停）。
- 判定改为**向外**：`超出赛道外缘 GEOFENCE_MARGIN_MM` 才算越界
  （原来是从边缘向内收 80 mm，等于把有效场地缩小了一圈）。

**验证**：`chassis_debug` 开环推到 `x = 2500 mm`，比赛配置下只打印一次 warn；
调试配置下进 FAULT。

### P1-3 启动后回零占用比赛时间 ✅

**改动的函数**：`Lift_Init` / `Lift_Tick` / `Lift_StartHoming` / `App_Run`（READY→RUN）

- 新增 `LIFT_ZERO_AT_POWERUP`（默认 1）：上电位置即零点，`homing_state` 直接置 `DONE`，
  比赛中不再回零。置 0 时保留原堵转回零。
- 回零判据加强：下行阶段累计移动 ≥ `LIFT_HOMING_MOVE_MIN_CNT × 10`（40 计数）后，
  "500 ms 不动"才算到底；否则判回零失败，避免"12% 占空比推不动"被当成"到底"。

**验证**：`LIFT_ZERO_AT_POWERUP=1` 时按启动键后日志应为
`START | zero at powerup, skip homing`，不出现 `homing...`。

### P1-4 动作启动的竞态 ✅

**改动的函数**：`CMotion_Begin` / `ChassisMotion_Stop` / `ChassisMotion_Hold`

- `CMotion_Begin` 把所有字段写完，**最后**才写 `s_state = CMOTION_RUN`，
  并用 `__disable_irq()/__enable_irq()` 包住整段（只做赋值）。
- `ChassisMotion_Stop/Hold` 同样先落状态再动 Chassis，并关中断保护。

**验证**：`script_debug` 连续跑 50 次"300 mm 前进"，每一步都应完整走完，
不应出现"瞬间完成、实际没动"的记录。

### P1-5 蓝牙独立串口 ❌ 未采纳

**不做的原因**：已与用户确认——HC-05 保持接 **USART1**，不改 USART3。
`DEBUG_VISION_ENABLE == 1` 时文本日志仍编译为空实现。
USART3（PB10/PB11）在 `.ioc` 中保持未占用，后续若改接线可随时启用。

### P2-1 禁用后仍可被下发速度指令 ✅

- 已在 P0-1 一并实现：`Chassis` 新增 `enabled` 字段与 `Chassis_SetEnabled()`。
  `enabled == 0` 时 `Chassis_Task` **强制 HOLD 并直接 return**，任何速度指令都不会下发。
  `DC_Motor_Enable(..., false)` 与 `DC_Motor_EmergencyStopAll()` 都会调用它。

### P2-2 回零期间不检查 110 s 截止 ✅

- `App_Run` 的 `APP_RUN` 分支在回零等待**之前**先判 `MATCH_SAFE_DEADLINE_MS`。
  `App_SafetyCheck` 里保留同一检查（覆盖脚本执行期），日志只在入口处打一次。

### P2-3 浮点 `_Static_assert` 依赖 GCC 扩展 ✅

- 不是 bug，已在 `app_config.h` §11 增加说明：浮点比较不属于 C11 的整型常量表达式，
  仅 GCC/Clang 接受；换编译器或开 `-Wpedantic` 会告警。当前工程只用 GCC，保留现状。

### P2-4 README 外设表补 USART3

- **不适用**：P1-5 未采纳，USART3 不占用。§1.1 表中已注明 "USART3 未使用"。

### v3 修复后的编译结果

| 配置 | FLASH | RAM | 告警 |
|---|---|---|---|
| Debug / `RUN_DEBUG` | 45.7 KB（69.8%） | 4.6 KB（22.7%） | 0 |
| Debug / `RUN_MATCH` | 57.8 KB（88.2%） | 4.7 KB（23.0%） | 0 |
| Release / `RUN_DEBUG` | 38.6 KB（58.9%） | 4.6 KB（22.7%） | 0 |
| **Release / `RUN_MATCH`** | **50.6 KB（77.3%）** | 4.7 KB（23.0%） | 0 |