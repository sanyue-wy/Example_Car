/**
 * @file    app_script.c
 * @brief   比赛动作脚本数据表
 *
 * @note    【待实测项】带 "TUNE:" 注释的数值必须按实际赛道/机构修改:
 *            - 取料前进/后退距离(与倾倒区尺寸、铲斗长度相关)
 *            - 去平台的走廊路径(起点朝向 +Y 走廊, 再转向 +X, 再转向 -Y)
 *            - 停靠点与平台边的间距
 */

/* Includes ------------------------------------------------------------------*/
#include "app_script.h"
#include "bucket.h"     /* BUCKET_STOW / SCOOP / CARRY / DUMP 枚举值 */
#include <stddef.h>     /* NULL */

/*==============================================================================
 * 公共段: 取料 (倾倒区 30x30 DAR, 允许接触 R22)
 *   起始位姿 (200, 400, 0deg), 倾倒区在 +Y 一侧 (Y 800~1200)
 *============================================================================*/
static const ActStep_t s_seg_collect[] =
{
    /* 1. 抬铲越过 30mm 墙 */
    { ACT_BUCKET,     BUCKET_STOW,  0,                  0U,    ON_TO_NEXT,     "bucket stow" },
    { ACT_LIFT,       (int32_t)LIFT_H_CLEAR, 0,         6000U, ON_TO_NEXT,     "lift to clear" },

    /* 2. 转向倾倒区(+Y = theta 90deg), 慢速靠近 */
    { ACT_TURN,       90,            (int32_t)CHASSIS_W_MAX_DPS, 8000U, ON_TO_NEXT, "turn +Y" },
    { ACT_MOVE,       250,           (int32_t)CHASSIS_V_SLOW_MMS, 0U, ON_TO_FALLBACK, "approach dump" }, /* TUNE */

    /* 3. 铲斗进倾倒区取料 */
    { ACT_BUCKET,     BUCKET_SCOOP,  0,                  0U,    ON_TO_NEXT,     "bucket scoop" },
    { ACT_LIFT,       (int32_t)LIFT_H_FLOOR, 0,         6000U, ON_TO_NEXT,     "lift to floor" },
    { ACT_MOVE,       120,           (int32_t)(CHASSIS_V_SLOW_MMS / 2), 0U, ON_TO_FALLBACK, "push into dump" }, /* TUNE */

    /* 4. 承托: 完全支撑 -> SUPPORT(R31-R33) */
    { ACT_BUCKET,     BUCKET_CARRY,  0,                  0U,    ON_TO_NEXT,     "bucket carry" },
    { ACT_LIFT,       (int32_t)LIFT_H_CARRY, 0,         6000U, ON_TO_NEXT,     "lift carry" },

    /* 5. 退出倾倒区 */
    { ACT_MOVE,       -200,          (int32_t)CHASSIS_V_SLOW_MMS, 0U, ON_TO_FALLBACK, "back out of dump" }, /* TUNE */
    { ACT_TURN,       -90,           (int32_t)CHASSIS_W_MAX_DPS, 8000U, ON_TO_NEXT, "turn back +X" },
};

/*==============================================================================
 * 公共段: 去平台
 *   走 Y 走廊(避开 Y 0~400 的坡道), 到达平台前方的停靠点
 *   实测: 初始路径是"沿 +X 走到 X≈1900, 再转向 -Y 面对平台长边"
 *============================================================================*/
static const ActStep_t s_seg_travel[] =
{
    /* 从倾倒区附近(Y≈650)沿 +X 前进到平台前方 */
    { ACT_MOVE,       1500,          (int32_t)CHASSIS_V_MAX_MMS, 0U, ON_TO_FALLBACK, "run to platform" }, /* TUNE */
    { ACT_TURN,       90,            (int32_t)CHASSIS_W_MAX_DPS, 8000U, ON_TO_NEXT, "face -Y" },
    { ACT_MOVE,       250,           (int32_t)CHASSIS_V_SLOW_MMS, 0U, ON_TO_FALLBACK, "approach platform" }, /* TUNE */
};

/*==============================================================================
 * L2 段: 中间平台(+10/块)  —— 本车行程不够, 默认不启用
 *============================================================================*/
static const ActStep_t s_seg_place_l2[] =
{
    /* 先升后靠近(避免撞平台边) */
    { ACT_LIFT,       (int32_t)LIFT_H_LEVEL2, 0,        12000U, ON_TO_FALLBACK, "lift L2" },
    { ACT_CHECK_LIFT, (int32_t)LIFT_H_LEVEL2, (int32_t)LIFT_ARRIVE_TOL_MM, 0U, ON_TO_FALLBACK, "check L2" },
    { ACT_MOVE,       (int32_t)REACH_MM, (int32_t)(CHASSIS_V_SLOW_MMS / 2), 0U, ON_TO_FALLBACK, "reach onto L2" },
    { ACT_BUCKET,     BUCKET_DUMP,   600,               0U,    ON_TO_NEXT,     "dump slow" },
    { ACT_WAIT,       300,           0,                 0U,    ON_TO_NEXT,     "wait settle" },
    { ACT_MOVE,       -(int32_t)REACH_MM, (int32_t)(CHASSIS_V_SLOW_MMS / 2), 0U, ON_TO_FALLBACK, "back off L2" },
    { ACT_BUCKET,     BUCKET_STOW,   0,                 0U,    ON_TO_NEXT,     "stow" },
    { ACT_LIFT,       (int32_t)LIFT_H_HOME, 0,          8000U, ON_TO_NEXT,     "lift home" },
};

/*==============================================================================
 * L1 段: 下层平台区(+5/块)  —— 保底方案
 *============================================================================*/
static const ActStep_t s_seg_place_l1[] =
{
    { ACT_LIFT,       (int32_t)LIFT_H_LEVEL1, 0,        6000U, ON_TO_FALLBACK, "lift L1" },
    { ACT_MOVE,       (int32_t)REACH_MM, (int32_t)(CHASSIS_V_SLOW_MMS / 2), 0U, ON_TO_FALLBACK, "reach onto L1" }, /* TUNE */
    { ACT_BUCKET,     BUCKET_DUMP,   600,               0U,    ON_TO_NEXT,     "dump slow L1" },
    { ACT_WAIT,       300,           0,                 0U,    ON_TO_NEXT,     "wait settle" },
    { ACT_MOVE,       -(int32_t)REACH_MM, (int32_t)(CHASSIS_V_SLOW_MMS / 2), 0U, ON_TO_FALLBACK, "back off L1" },
    { ACT_BUCKET,     BUCKET_STOW,   0,                 0U,    ON_TO_NEXT,     "stow L1" },
    { ACT_LIFT,       (int32_t)LIFT_H_HOME, 0,          8000U, ON_TO_NEXT,     "lift home" },
};

/*==============================================================================
 * 收尾段: 后退离开平台/坡道/支撑管(R34: 结束时不得接触)
 *============================================================================*/
static const ActStep_t s_seg_retreat[] =
{
    { ACT_MOVE,       -(int32_t)RETREAT_CLEARANCE_MM, (int32_t)CHASSIS_V_SLOW_MMS, 0U, ON_TO_FINISH, "retreat" },
    { ACT_BUCKET,     BUCKET_STOW,   0,                 0U,    ON_TO_NEXT,     "final stow" },
};

/*==============================================================================
 * 段表
 *============================================================================*/
static const ScriptSegDef_t s_segments[SEG_NUM] =
{
    [SEG_COLLECT]  = { "COLLECT",  s_seg_collect,  (uint16_t)(sizeof(s_seg_collect)  / sizeof(s_seg_collect[0]))  },
    [SEG_TRAVEL]   = { "TRAVEL",   s_seg_travel,   (uint16_t)(sizeof(s_seg_travel)   / sizeof(s_seg_travel[0]))   },
    [SEG_PLACE_L2] = { "PLACE_L2", s_seg_place_l2, (uint16_t)(sizeof(s_seg_place_l2) / sizeof(s_seg_place_l2[0])) },
    [SEG_PLACE_L1] = { "PLACE_L1", s_seg_place_l1, (uint16_t)(sizeof(s_seg_place_l1) / sizeof(s_seg_place_l1[0])) },
    [SEG_RETREAT]  = { "RETREAT",  s_seg_retreat,  (uint16_t)(sizeof(s_seg_retreat)  / sizeof(s_seg_retreat[0]))  },
};

/* Exported functions ---------------------------------------------------------*/

const ScriptSegDef_t *AppScript_GetSegment(uint8_t index)
{
    if (index >= (uint8_t)SEG_NUM)
    {
        return NULL;
    }
    return &s_segments[index];
}

uint8_t AppScript_ProgramMax(void)
{
    return (uint8_t)SEG_NUM;
}

uint8_t AppScript_GetProgram(uint8_t *out, uint8_t max)
{
    uint8_t n = 0;

    if ((out == NULL) || (max == 0U))
    {
        return 0U;
    }

    out[n++] = (uint8_t)SEG_COLLECT;
    out[n++] = (uint8_t)SEG_TRAVEL;

#if (STRATEGY_LEVEL2_ENABLE == 1)
    out[n++] = (uint8_t)SEG_PLACE_L2;
#else
    out[n++] = (uint8_t)SEG_PLACE_L1;
#endif

    out[n++] = (uint8_t)SEG_RETREAT;

    return (n <= max) ? n : max;
}

uint8_t AppScript_FallbackSegment(void)
{
    /* L2 失败一律降级到 L1, 保证不丢 COLLECT+SUPPORT 分 */
    return (uint8_t)SEG_PLACE_L1;
}