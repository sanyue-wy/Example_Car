/**
 * @file    app_script.h
 * @brief   比赛动作脚本: 纯数据表 + 供执行器遍历的段定义
 *
 * @note    【设计原则】脚本是数据, 执行器是逻辑。调参只改这里的距离/角度,
 *          不改 app.c 的执行流程。
 *
 * @note    【数值来源】距离/角度均为任务书 §5.8.4 的结构, 具体数值是占位值,
 *          必须按实测修改(见 README_APP.md 的调参顺序)。每段都标了
 *          "待实测" 的项。
 */

#ifndef __APP_SCRIPT_H__
#define __APP_SCRIPT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_config.h"

/*==============================================================================
 * 动作类型
 *============================================================================*/
typedef enum
{
    ACT_MOVE       = 0,   /* p1 = 距离 mm(正=前进), p2 = 速度 mm/s(0=SLOW) */
    ACT_TURN       = 1,   /* p1 = 角度 deg(正=逆时针), p2 = 角速度 deg/s */
    ACT_WAIT       = 2,   /* p1 = ms */
    ACT_LIFT       = 3,   /* p1 = 高度 mm */
    ACT_BUCKET     = 4,   /* p1 = BucketPose_e, p2 = 慢速时间 ms(0=默认) */
    ACT_CHECK_LIFT = 5,   /* p1 = 期望高度 mm, p2 = 容差 mm; 不满足 -> 降级 */
    ACT_END        = 6
} ActType_t;

/* 单步超时后的去向 */
typedef enum
{
    ON_TO_NEXT     = 0,   /* 继续下一条(不推荐用于运动步) */
    ON_TO_FALLBACK = 1,   /* 跳到降级段 */
    ON_TO_FINISH   = 2    /* 直接进入收尾 */
} ActTimeoutAction_e;

typedef struct
{
    ActType_t type;
    int32_t   p1;
    int32_t   p2;
    uint16_t  timeout_ms;   /* 0 = 用自动估算 */
    uint8_t   on_timeout;   /* ActTimeoutAction_e */
    const char *tag;        /* 调试打印用 */
} ActStep_t;

/*==============================================================================
 * 段索引(program 表的顺序)
 *============================================================================*/
typedef enum
{
    SEG_COLLECT = 0,
    SEG_TRAVEL,
    SEG_PLACE_L2,
    SEG_PLACE_L1,
    SEG_RETREAT,
    SEG_NUM
} ScriptSegment_e;

typedef struct
{
    const char      *name;
    const ActStep_t *steps;
    uint16_t         count;
} ScriptSegDef_t;

/**
 * @brief  取某段的定义
 */
const ScriptSegDef_t *AppScript_GetSegment(uint8_t index);

/**
 * @brief  取"从比赛开始到结束"的默认程序(段索引序列)
 * @param  out   输出缓冲, 至少 AppScript_ProgramMax() 个
 * @retval 实际长度
 * @note   STRATEGY_LEVEL2_ENABLE==0 时不会包含 SEG_PLACE_L2
 */
uint8_t AppScript_GetProgram(uint8_t *out, uint8_t max);

uint8_t AppScript_ProgramMax(void);

/**
 * @brief  降级段索引(SEG_PLACE_L2 失败时跳到哪里)
 */
uint8_t AppScript_FallbackSegment(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SCRIPT_H__ */