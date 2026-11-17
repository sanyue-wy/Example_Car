/**
 * @file    params_car.c
 * @brief   bt_vision 调试参数表. 控制代码直接读这些变量即可, 上位机写入后立即生效.
 * @note    改参数: 同时改这里和上位机的 car_primary.json (ID/类型/范围必须一致).
 */
#include "params_car.h"

float    g_pos_kp         = 2.4f;   /* 位置环 Kp  */
float    g_pos_ki         = 0.0f;   /* 位置环 Ki  */
float    g_pos_kd         = 0.1f;   /* 位置环 Kd  */
float    g_pos_i_lim      = 200.0f; /* 位置环积分限幅 °/s */
float    g_pos_out_lim    = 800.0f; /* 位置环输出限幅 °/s */
float    g_pos_deadband   = 0.5f;   /* 位置环死区 ° */
float    g_spd_kp         = 8.0f;   /* 速度环 Kp  */
float    g_spd_ki         = 1.5f;   /* 速度环 Ki  */
float    g_spd_kd         = 0.0f;   /* 速度环 Kd  */
float    g_spd_i_lim      = 50.0f;  /* 速度环积分限幅 % */
float    g_spd_out_lim    = 90.0f;  /* 速度环输出限幅 % */
float    g_yaw_kp         = 1.0f;   /* 转向环 Kp  */
float    g_yaw_kd         = 0.05f;  /* 转向环 Kd  */
float    g_gyro_bias_x    = 0.0f;   /* 陀螺仪零偏 X °/s */
float    g_gyro_bias_y    = 0.0f;   /* 陀螺仪零偏 Y °/s */
float    g_gyro_bias_z    = 0.0f;   /* 陀螺仪零偏 Z °/s */
uint8_t  g_gyro_filt      = 0;      /* 滤波算法  */
float    g_gyro_cf_alpha  = 0.98f;  /* 互补滤波系数  */
float    g_gyro_lpf_hz    = 30.0f;  /* 低通截止频率 Hz */
float    g_cg_offset      = 0.0f;   /* 重心偏移 mm */
float    g_wheel_dia      = 65.0f;  /* 轮径 mm */
uint16_t g_enc_cpr        = 1320;   /* 编码器每转计数 count */
float    g_tgt_speed      = 0.5f;   /* 目标速度 m/s */
float    g_acc_limit      = 1.2f;   /* 加速度限制 m/s² */
uint8_t  g_report_hz      = 50;     /* 上报频率 Hz */

const btdbg_param_t g_car_params[] = {
    BTDBG_PARAM("POS_KP",        g_pos_kp,        BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 10),
    BTDBG_PARAM("POS_KI",        g_pos_ki,        BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 5),
    BTDBG_PARAM("POS_KD",        g_pos_kd,        BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 5),
    BTDBG_PARAM("POS_I_LIM",     g_pos_i_lim,     BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 1000),
    BTDBG_PARAM("POS_OUT_LIM",   g_pos_out_lim,   BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 1500),
    BTDBG_PARAM("POS_DEADBAND",  g_pos_deadband,  BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 5),
    BTDBG_PARAM("SPD_KP",        g_spd_kp,        BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 50),
    BTDBG_PARAM("SPD_KI",        g_spd_ki,        BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 20),
    BTDBG_PARAM("SPD_KD",        g_spd_kd,        BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 5),
    BTDBG_PARAM("SPD_I_LIM",     g_spd_i_lim,     BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 100),
    BTDBG_PARAM("SPD_OUT_LIM",   g_spd_out_lim,   BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 100),
    BTDBG_PARAM("YAW_KP",        g_yaw_kp,        BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 20),
    BTDBG_PARAM("YAW_KD",        g_yaw_kd,        BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 5),
    BTDBG_PARAM("GYRO_BIAS_X",   g_gyro_bias_x,   BTDBG_F32, BTDBG_PERSIST, BTDBG_W_IDLE    , -5, 5),
    BTDBG_PARAM("GYRO_BIAS_Y",   g_gyro_bias_y,   BTDBG_F32, BTDBG_PERSIST, BTDBG_W_IDLE    , -5, 5),
    BTDBG_PARAM("GYRO_BIAS_Z",   g_gyro_bias_z,   BTDBG_F32, BTDBG_PERSIST, BTDBG_W_IDLE    , -5, 5),
    BTDBG_PARAM("GYRO_FILT",     g_gyro_filt,     BTDBG_U8, BTDBG_PERSIST, BTDBG_W_IDLE    , 0, 2),
    BTDBG_PARAM("GYRO_CF_ALPHA", g_gyro_cf_alpha, BTDBG_F32, BTDBG_PERSIST, BTDBG_W_IDLE    , 0, 1),
    BTDBG_PARAM("GYRO_LPF_HZ",   g_gyro_lpf_hz,   BTDBG_F32, BTDBG_PERSIST, BTDBG_W_IDLE    , 1, 200),
    BTDBG_PARAM("CG_OFFSET",     g_cg_offset,     BTDBG_F32, BTDBG_PERSIST, BTDBG_W_IDLE    , -20, 20),
    BTDBG_PARAM("WHEEL_DIA",     g_wheel_dia,     BTDBG_F32, BTDBG_PERSIST, BTDBG_W_IDLE    , 20, 200),
    BTDBG_PARAM("ENC_CPR",       g_enc_cpr,       BTDBG_U16, BTDBG_PERSIST, BTDBG_W_IDLE    , 1, 65535),
    BTDBG_PARAM("TGT_SPEED",     g_tgt_speed,     BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0, 2),
    BTDBG_PARAM("ACC_LIMIT",     g_acc_limit,     BTDBG_F32, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 0.1, 5),
    BTDBG_PARAM("REPORT_HZ",     g_report_hz,     BTDBG_U8, BTDBG_RAM    , BTDBG_W_IDLE_RUN, 1, 100),
};
const uint16_t g_car_param_count = (uint16_t)(sizeof g_car_params / sizeof g_car_params[0]);