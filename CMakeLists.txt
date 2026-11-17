cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Example_Car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

#==============================================================================
#  User 层源码
#
#  分层:
#    Middleware  驱动层(drv_tim/drv_pwm/drv_uart/drv_encoder) + 算法(pid)
#    Device      设备层(AT8236 驱动板 / 编码器电机 / 舵机 / 直流电机抽象)
#    Module      模块层(底盘 / 底盘运动原语 / 升降 / 铲斗)
#    APP         应用层(比赛状态机 / 脚本 / 周期任务 / 调试)
#
#  新增 .c 必须加到这里; 新增目录必须加到下面的 include 列表。
#==============================================================================
set(USER_APP_DIR      ${CMAKE_CURRENT_SOURCE_DIR}/User/APP)
set(USER_DEVICE_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/User/Device)
set(USER_MODULE_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/User/Module)
set(USER_MIDDLE_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/User/Middleware)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # ---- APP 层 ----
    ${USER_APP_DIR}/app.c
    ${USER_APP_DIR}/app_script.c
    ${USER_APP_DIR}/app_log.c
    ${USER_APP_DIR}/task_it.c
    ${USER_APP_DIR}/debug.c
    ${USER_APP_DIR}/debug_vision.c

    # ---- Middleware: 驱动 ----
    ${USER_MIDDLE_DIR}/Driver/TIM/drv_tim.c
    ${USER_MIDDLE_DIR}/Driver/PWM/drv_pwm.c
    ${USER_MIDDLE_DIR}/Driver/UART/drv_uart.c
    ${USER_MIDDLE_DIR}/Driver/ENCODER/drv_encoder.c

    # ---- Middleware: 算法 ----
    ${USER_MIDDLE_DIR}/Algorithm/Controller/pid.c

    # ---- Device ----
    ${USER_DEVICE_DIR}/AT8236_driver/at8236.c
    ${USER_DEVICE_DIR}/Encode_motor/encoder_motor.c
    ${USER_DEVICE_DIR}/DC_Motor/dc_motor.c
    ${USER_DEVICE_DIR}/servo/servo.c
    ${USER_DEVICE_DIR}/Tool/bt_vision/btdbg.c
    ${USER_DEVICE_DIR}/Tool/bt_vision/params_car.c

    # ---- Module ----
    ${USER_MODULE_DIR}/Chassis/chassis.c
    ${USER_MODULE_DIR}/Chassis/chassis_motion.c
    ${USER_MODULE_DIR}/Elevating/elevating.c
    ${USER_MODULE_DIR}/Lift/lift.c
    ${USER_MODULE_DIR}/Bucket/bucket.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${USER_APP_DIR}
    ${USER_MIDDLE_DIR}/Driver/TIM
    ${USER_MIDDLE_DIR}/Driver/PWM
    ${USER_MIDDLE_DIR}/Driver/UART
    ${USER_MIDDLE_DIR}/Driver/ENCODER
    ${USER_MIDDLE_DIR}/Algorithm/Controller
    ${USER_DEVICE_DIR}/AT8236_driver
    ${USER_DEVICE_DIR}/Encode_motor
    ${USER_DEVICE_DIR}/DC_Motor
    ${USER_DEVICE_DIR}/servo
    ${USER_DEVICE_DIR}/Tool/bt_vision
    ${USER_MODULE_DIR}/Chassis
    ${USER_MODULE_DIR}/Elevating
    ${USER_MODULE_DIR}/Lift
    ${USER_MODULE_DIR}/Bucket
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# 把 warning 当作可读信息打印, 但先不 -Werror(第三方 HAL 会产生告警)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
)

#-----------------------------------------------------------------------------
# Debug 构建在 STM32F103C8T6(64K Flash)上放不下比赛固件:
#   工具链默认 Debug = -O0, 生成的代码比 -Os 多出约 30KB(本次实测 69.4KB vs 38.4KB,
#   超出 64KB 上限)。AT8236 驱动内的 strtof/vsnprintf 又额外拉入 newlib 的
#   浮点解析与格式化代码, 进一步放大差异。
#   因此在 Debug 下改用 -Os: 仍保留 -g3 全部调试信息, 单步/断点/变量查看不受影响,
#   只是不做表达式级优化, 便于在片上调试。GCC 以最后一个 -O 选项为准。
#-----------------------------------------------------------------------------
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Os)
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)