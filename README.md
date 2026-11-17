# Example_Car —— 第 39 届 Warman 竞赛小车

STM32F103C8T6（64 KB Flash / 20 KB RAM，72 MHz）主控固件：四轮差速底盘 + 升降丝杆 + 铲斗舵机，
比赛状态机与动作脚本执行器，附带蓝牙可视化在线调参与分层台架自检。

> **APP 层的完整说明（外设占用表、CubeMX 待补清单、调参顺序、未决问题）见
> [`User/APP/README_APP.md`](User/APP/README_APP.md)** —— 本文件只做仓库导航。

## 硬件基线

| 部件 | 配置 |
|---|---|
| MCU | STM32F103C8T6，HSE 8 MHz ×9 = 72 MHz |
| 底盘 | AT8236 四路串口驱动板，四轮差速（驱动板内闭环，主控下发 mm/s） |
| 升降 | 编码器直流电机 → 丝杆，TIM1_CH1 PWM + TIM2 编码器，行程 200 mm |
| 铲斗 | 1 路舵机，TIM3_CH1 (PA6)，50 Hz |
| 启动键 / 状态灯 | PB12 上拉输入 / PC13 推挽输出（**需在 CubeMX 中补配**） |
| 陀螺仪 | 无，航向靠四轮编码器里程计 |

## 目录

```
Core/         CubeMX 生成代码（只改 USER CODE 区）
User/
  APP/        比赛状态机、动作脚本、周期任务、调试层、蓝牙可视化
  Device/     AT8236 驱动板、编码器电机、直流电机抽象
  Module/     底盘、运动原语、升降、铲斗
  Middleware/ drv_tim / drv_pwm / drv_uart / drv_encoder / pid
Drivers/      ST HAL + CMSIS（STM32Cube 原样）
cmake/        工具链与 CubeMX 生成的构建脚本
```

## 构建

```bash
cmake --preset Debug   && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
```

需要 `arm-none-eabi-gcc` + Ninja。产物为 `build/<preset>/Example_Car.elf`。

| 配置 | FLASH | RAM |
|---|---|---|
| Debug（台架自检） | 45.6 KB（69.6%） | 4.6 KB |
| Debug（比赛） | 57.2 KB（87.3%） | 4.7 KB |
| **Release（比赛，推荐上场）** | **50.0 KB（76.3%）** | 4.7 KB |

`build/` 与 `Drivers/CMSIS/{Lib,DSP,NN,Core_A,RTOS,RTOS2}` 不入库（后者合计约 48 MB，
本构建只引用 `CMSIS/Include` 与 `CMSIS/Device/ST/STM32F1xx/Include`，见 `.gitignore`）。
若从本仓库克隆后要在 CubeMX 里重新生成代码，请自行补回这些目录或直接用 STM32CubeMX 打开 `.ioc`。

## 比赛模式的硬性约束

- `RUN_MODE == RUN_MATCH` 时，任何调试层与蓝牙可视化都会在编译期 `#error`；
- 比赛中不接收任何上位机指令，串口只允许单向打印（规则 FAQ3 / R21）；
- **上场前必须拔掉 HC-05 蓝牙模块**；
- 结束后状态 LED 只在所有电机确认禁用之后点亮。

## 许可

本仓库不附带开源许可证，保留所有权利。
`Drivers/` 下的 ST HAL 与 CMSIS 代码分别遵循其原有 BSD-3-Clause / Apache-2.0 许可。