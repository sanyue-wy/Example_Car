/**
 * @file    params_car.h
 * @brief   bt_vision 调试参数表(与上位机 bt-debug-visualizer 的
 *          config/pdef/HC-05/car_primary.json 一一对应)
 * @note    控制代码直接读这里的 g_* 变量, 上位机 SET 写入后立即生效。
 *          改参数必须同时改本文件与上位机的 car_primary.json(ID/类型/范围一致)。
 */
#ifndef PARAMS_CAR_H
#define PARAMS_CAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "btdbg.h"

extern float    g_pos_kp;
extern float    g_pos_ki;
extern float    g_pos_kd;
extern float    g_pos_i_lim;
extern float    g_pos_out_lim;
extern float    g_pos_deadband;
extern float    g_spd_kp;
extern float    g_spd_ki;
extern float    g_spd_kd;
extern float    g_spd_i_lim;
extern float    g_spd_out_lim;
extern float    g_yaw_kp;
extern float    g_yaw_kd;
extern float    g_gyro_bias_x;
extern float    g_gyro_bias_y;
extern float    g_gyro_bias_z;
extern uint8_t  g_gyro_filt;
extern float    g_gyro_cf_alpha;
extern float    g_gyro_lpf_hz;
extern float    g_cg_offset;
extern float    g_wheel_dia;
extern uint16_t g_enc_cpr;
extern float    g_tgt_speed;
extern float    g_acc_limit;
extern uint8_t  g_report_hz;

extern const btdbg_param_t g_car_params[];
extern const uint16_t      g_car_param_count;

#ifdef __cplusplus
}
#endif

#endif /* PARAMS_CAR_H */