/**
 * @file    chassis_map.h
 * @brief   实车四轮 -> AT8236 驱动板通道的映射(无依赖, APP 层与调试层共用)
 *
 * @note    逻辑轮序固定(运动学/里程计/遥测都用这个顺序):
 *            0 = 左前(FL), 1 = 左后(RL), 2 = 右前(FR), 3 = 右后(RR)
 *          数组元素 = 驱动板物理通道号: 0=M1, 1=M2, 2=M3, 3=M4。
 *
 *          实车实测(2026-11-24 照片确认, 铲斗为车头, 顺时针编号 4/1/3/2):
 *
 *                        车头(铲斗)
 *               [左前 M4]        [右前 M1]
 *               [左后 M3]        [右后 M2]
 *
 *          即 左前=M4(通道3), 左后=M3(通道2), 右前=M1(通道0), 右后=M2(通道1)。
 *
 * @note    换驱动板/重新接线时只改这里一行, chassis 运动学不用动。
 *          chassis_debug 也直接包含本文件, 保证自检与比赛固件用同一份映射。
 */

#ifndef __CHASSIS_MAP_H__
#define __CHASSIS_MAP_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CHASSIS_WHEEL_MAP
#define CHASSIS_WHEEL_MAP        {3U, 2U, 0U, 1U}
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_MAP_H__ */