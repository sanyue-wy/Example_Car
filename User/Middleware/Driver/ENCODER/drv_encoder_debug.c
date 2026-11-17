#include "debug_config.h"

#if encoder_debug == 1

#include "drv_encoder_debug.h"
#include "drv_encoder.h"
#include "drv_uart.h"
#include "main.h"
#include "tim.h"

/* ---- 第15层 encoder_debug: 编码器输入自检(TIM2, 纯输入, 不会让机构动作) ----
   编码器没有执行器, 无法像舵机那样自动"转到某角度再回读", 因此自检采用
   CNT 注入: 直接写 TIM2->CNT 为已知值, 再调 ENCODER_Update, 核对待测对象
   算出的 delta/total 是否与手算一致 —— 覆盖正/反向、±半个量程内大步、
   正向/反向回绕、方向取反、圈数/转速换算、Reset 与统计。
   判据: 每项 delta/total 与期望值完全相等 -> [PASS], 否则 [FAIL]; 轮末汇总。
   注意: 自检期间不要转动编码器输入轴, 也不要使能带编码器的电机, 否则硬件
   边沿会叠加到注入值上, 导致误判。自检结束后每秒打印实时计数, 手动转轴
   可直观验证方向与速度。 ------------------------------------------------ */

/* 本工程 JCA25-370-103K: 11 线 × 4 倍频 × 103 减速比 = 4532 */
#define ENCODER_DEBUG_CPR      4532U  /* 自检用"每圈计数", 按实际编码器参数改 */
#define ENCODER_DEBUG_LIVE_MS  1000U  /* 实时计数打印周期(ms) */
#define ENCODER_DEBUG_EPS      0.0001f/* 浮点换算比较容差 */

typedef enum {
    ENC_DBG_ST_IDLE = 0, /* Init 失败, 不再运行 */
    ENC_DBG_ST_LIVE      /* 自检已通过, 实时计数监控 */
} EncoderDebugState_t;

/* 调试对象: 与业务侧对象分属不同实例, 但同绑 TIM2 时驱动会拒绝第二个对象 */
static ENCODER_Object s_enc_dbg;
static EncoderDebugState_t s_enc_dbg_state = ENC_DBG_ST_IDLE;
static uint32_t s_enc_dbg_tick = 0U;

static uint32_t s_enc_chk_steps = 0U; /* 本轮核对项数 */
static uint32_t s_enc_chk_err   = 0U; /* 本轮不一致项数 */

static void Encoder_DebugCheck(const char *tag, int32_t expected, int32_t actual)
{
    s_enc_chk_steps++;
    if (expected == actual)
    {
        UART_Port_Printf(&uart1_port, "[enc] [PASS] %s exp=%ld act=%ld\r\n",
                         tag, (long)expected, (long)actual);
    }
    else
    {
        s_enc_chk_err++;
        UART_Port_Printf(&uart1_port, "[enc] [FAIL] %s exp=%ld act=%ld\r\n",
                         tag, (long)expected, (long)actual);
    }
}

static void Encoder_DebugCheckFloat(const char *tag, float expected, float actual)
{
    s_enc_chk_steps++;
    if ((expected - actual) <= ENCODER_DEBUG_EPS && (actual - expected) <= ENCODER_DEBUG_EPS)
    {
        UART_Port_Printf(&uart1_port, "[enc] [PASS] %s exp=%.3f act=%.3f\r\n",
                         tag, expected, actual);
    }
    else
    {
        s_enc_chk_err++;
        UART_Port_Printf(&uart1_port, "[enc] [FAIL] %s exp=%.3f act=%.3f\r\n",
                         tag, expected, actual);
    }
}

/**
  * @brief  用 CNT 注入跑一遍计数逻辑自检
  * @note   全部注入值之间的正向/反向跨度均 <= 半个量程(32768),
  *         这是驱动能正确判向的前提, 超出的情况驱动文档里已声明不可用
  */
static void Encoder_DebugSelfTest(void)
{
    uint32_t sms;
    uint32_t upd_before;
    uint32_t rst_before;

    s_enc_chk_steps = 0U;
    s_enc_chk_err = 0U;

    /* 1. CubeMX 配置核对: Encoder_Interface + TI12(四倍频) + 16 位量程 */
    sms = TIM2->SMCR & TIM_SMCR_SMS;
    UART_Port_Printf(&uart1_port,
                     "[enc] cfg: sms=%lu (3=TI12/x4) arr=%lu span=%lu cpr=%lu\r\n",
                     (unsigned long)sms,
                     (unsigned long)TIM2->ARR,
                     (unsigned long)s_enc_dbg.period_ticks,
                     (unsigned long)s_enc_dbg.counts_per_rev);
    Encoder_DebugCheck("mode=TI12", (int32_t)TIM_ENCODERMODE_TI12, (int32_t)sms);
    Encoder_DebugCheck("span=65536", 65536, (int32_t)s_enc_dbg.period_ticks);

    /* 2. Reset 基线: 硬件 CNT 与对象状态同时归零 */
    ENCODER_Reset(&s_enc_dbg);
    Encoder_DebugCheck("reset raw", 0, (int32_t)ENCODER_GetRawCount(&s_enc_dbg));
    Encoder_DebugCheck("reset total", 0, ENCODER_GetCount(&s_enc_dbg));
    Encoder_DebugCheck("reset delta", 0, ENCODER_GetDelta(&s_enc_dbg));

    /* 3. 正向小步: CNT 0 -> 100 */
    __HAL_TIM_SET_COUNTER(&htim2, 100U);
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheck("fwd +100", 100, ENCODER_GetDelta(&s_enc_dbg));
    Encoder_DebugCheck("total=100", 100, ENCODER_GetCount(&s_enc_dbg));

    /* 4. 反向小步: CNT 100 -> 60 */
    __HAL_TIM_SET_COUNTER(&htim2, 60U);
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheck("rev -40", -40, ENCODER_GetDelta(&s_enc_dbg));
    Encoder_DebugCheck("total=60", 60, ENCODER_GetCount(&s_enc_dbg));

    /* 5. 半个量程内的最大正向步与反向步: 60 -> 32060 -> 60 */
    __HAL_TIM_SET_COUNTER(&htim2, 32060U);
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheck("fwd +32000", 32000, ENCODER_GetDelta(&s_enc_dbg));

    __HAL_TIM_SET_COUNTER(&htim2, 60U);
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheck("rev -32000", -32000, ENCODER_GetDelta(&s_enc_dbg));
    Encoder_DebugCheck("total=60", 60, ENCODER_GetCount(&s_enc_dbg));

    /* 6. 分段走到顶部: 60 -> 32060 -> 64060 -> 65500 */
    __HAL_TIM_SET_COUNTER(&htim2, 32060U);
    ENCODER_Update(&s_enc_dbg);
    __HAL_TIM_SET_COUNTER(&htim2, 64060U);
    ENCODER_Update(&s_enc_dbg);
    __HAL_TIM_SET_COUNTER(&htim2, 65500U);
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheck("total=65500", 65500, ENCODER_GetCount(&s_enc_dbg));

    /* 7. 正向回绕: 65500 -> 0, 应为 +36 而不是 -65500 */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheck("wrap fwd +36", 36, ENCODER_GetDelta(&s_enc_dbg));
    Encoder_DebugCheck("total=65536", 65536, ENCODER_GetCount(&s_enc_dbg));

    /* 8. 反向回绕: 0 -> 65500, 应为 -36 */
    __HAL_TIM_SET_COUNTER(&htim2, 65500U);
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheck("wrap rev -36", -36, ENCODER_GetDelta(&s_enc_dbg));
    Encoder_DebugCheck("total=65500", 65500, ENCODER_GetCount(&s_enc_dbg));

    /* 9. 方向取反: 65500 -> 10 的 +46 应变成 -46, 累计随之减 46 */
    ENCODER_SetInvert(&s_enc_dbg, 1U);
    __HAL_TIM_SET_COUNTER(&htim2, 10U);
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheck("invert delta", -46, ENCODER_GetDelta(&s_enc_dbg));
    Encoder_DebugCheck("invert total", 65454, ENCODER_GetCount(&s_enc_dbg));
    ENCODER_SetInvert(&s_enc_dbg, 0U);

    /* 10. 圈数/转速换算: 1133 计数 / 4532 CPR = 0.25 圈, 1s 周期 = 0.25 r/s */
    ENCODER_Reset(&s_enc_dbg);
    __HAL_TIM_SET_COUNTER(&htim2, 1133U);
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheckFloat("revolutions=0.25", 0.25f, ENCODER_GetRevolutions(&s_enc_dbg));
    Encoder_DebugCheckFloat("speed=0.25rps", 0.25f, ENCODER_GetSpeedRps(&s_enc_dbg, 1000U));
    Encoder_DebugCheckFloat("speed dt=0", 0.0f, ENCODER_GetSpeedRps(&s_enc_dbg, 0U));

    /* 11. 统计自增: 每次 Update +1, 每次 Reset +1 */
    upd_before = s_enc_dbg.stats.update_count;
    ENCODER_Update(&s_enc_dbg);
    Encoder_DebugCheck("stats update+1", (int32_t)(upd_before + 1U),
                       (int32_t)s_enc_dbg.stats.update_count);

    rst_before = s_enc_dbg.stats.reset_count;
    ENCODER_Reset(&s_enc_dbg);
    Encoder_DebugCheck("stats reset+1", (int32_t)(rst_before + 1U),
                       (int32_t)s_enc_dbg.stats.reset_count);

    /* 收尾: 回到干净零点, 实时监控从 0 开始 */
    ENCODER_Reset(&s_enc_dbg);

    UART_Port_Printf(&uart1_port, "[enc] self-test done: %lu checks, %lu failed%s\r\n",
                     (unsigned long)s_enc_chk_steps,
                     (unsigned long)s_enc_chk_err,
                     (s_enc_chk_err == 0U) ? " [PASS]" : " [FAIL]");
}

void Encoder_Debug_Init(void)
{
    uint8_t ret;

    /* TIM2 已是 Encoder_Interface(TI12), MX_TIM2_Init 在 debug_init 之前完成 */
    ret = ENCODER_Init(&s_enc_dbg, &htim2, "TIM2_ENC", ENCODER_DEBUG_CPR, 0U);
    if (ret != 0U)
    {
        s_enc_dbg_state = ENC_DBG_ST_IDLE;
        UART_Port_Printf(&uart1_port,
                         "[enc] [FAIL] init ret=%u (2=定时器已被占用, 3=非编码器模式)\r\n",
                         (unsigned)ret);
        return;
    }

    UART_Port_Printf(&uart1_port,
                     "[enc] init ok: TIM2 ENC | test: CNT inject fwd/rev/wrap/invert/convert\r\n");

    Encoder_DebugSelfTest();

    s_enc_dbg_state = ENC_DBG_ST_LIVE;
    s_enc_dbg_tick = HAL_GetTick();
}

void Encoder_Debug_Run(uint32_t now)
{
    if (s_enc_dbg_state != ENC_DBG_ST_LIVE)
    {
        return;
    }

    if (now - s_enc_dbg_tick >= ENCODER_DEBUG_LIVE_MS)
    {
        s_enc_dbg_tick = now;
        ENCODER_Update(&s_enc_dbg);
        /* 手动转轴: total 应随方向增减, 速度按 1s 采样计算; 转轴越快 |rps| 越大 */
        UART_Port_Printf(&uart1_port,
                         "[enc] live | raw=%lu total=%ld delta=%ld rev=%.2f rps=%.2f | upd=%lu rst=%lu\r\n",
                         (unsigned long)ENCODER_GetRawCount(&s_enc_dbg),
                         (long)ENCODER_GetCount(&s_enc_dbg),
                         (long)ENCODER_GetDelta(&s_enc_dbg),
                         ENCODER_GetRevolutions(&s_enc_dbg),
                         ENCODER_GetSpeedRps(&s_enc_dbg, ENCODER_DEBUG_LIVE_MS),
                         (unsigned long)s_enc_dbg.stats.update_count,
                         (unsigned long)s_enc_dbg.stats.reset_count);
    }
}

#endif /* encoder_debug == 1 */