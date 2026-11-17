#include "debug_config.h"

#if grip_debug == 1

#include "grip_debug.h"
#include "grip.h"
#include "servo.h"
#include "debug_shared.h"
#include "main.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "task_it.h"
#include "tim.h"
#include <string.h>

/* ---- 第10层 grip_debug: 夹爪机构(双舵机: 舵机1 Z 轴升降 + 舵机2 开合) ----
   每步两拍: 下发后立刻核对(回报角度 / 该通道 CCR 回读脉宽 / 忙碌标志)
   -> 等动作时长走完 -> 到位核对(角度 + 脉宽 + 终态)。
   判据: 期望角度、模块回报角度、回读脉宽(±容差)三者一致, 且忙碌标志
   动作中置起、到位后自动落下 -> 轮末汇总 [PASS]/[FAIL]。(舵机会动!) */
#define GRIP_TEST_TO_MS    3000U  /* 单步超时(动作时长估算留足富余) */
#define GRIP_TEST_GAP_MS   2000U  /* 轮末间隔 */
typedef enum {
    GT_INIT = 0,      /* 等上电姿态就绪, 开一轮 */
    GT_Z_DOWN,        /* Z 轴降到取件位 */
    GT_Z_UP,          /* Z 轴升回安全位 */
    GT_JAW_OPEN,      /* 夹爪张开 */
    GT_JAW_CLOSE_MAX, /* 夹爪 100% 力度闭合 */
    GT_JAW_CLOSE_MIN, /* 夹爪 0% 力度闭合(保底轻夹行程) */
    GT_CLAMP,         /* 越界钳位 + 非法力度拒绝 */
    GT_REPORT         /* 轮末汇总, 间隔后开下一轮 */
} GripTestState_t;
static GripTestState_t s_gt_state = GT_INIT;
static uint32_t s_gt_tick;
static uint8_t  s_gt_sent;  /* 当前步已下发(区分"下发核对"与"到位核对"两拍) */
static uint32_t s_gt_steps; /* 一轮内核对步数 */
static uint32_t s_gt_err;   /* 一轮内不一致步数 */

static uint8_t  Debug_GripPulseOk(const char *tag, uint32_t channel, uint16_t deg);
static void     GripTest_Report(const char *tag, uint8_t ok);

/**
  * @brief  核对某通道回读脉宽与期望角度是否一致(不一致当场打 [FAIL] 明细)
  * @retval 1 一致(±DEBUG_SERVO_PULSE_TOL_US); 0 不一致
  */
static uint8_t Debug_GripPulseOk(const char *tag, uint32_t channel, uint16_t deg)
{
  uint32_t exp;
  uint32_t act;
  uint8_t  ok = Debug_PulseCheck(Servo_GetHwChannel((ServoChannel_e)channel), deg, &exp, &act);

  if (!ok)
  {
    UART_Port_Printf(&uart1_port,
                     "[grip] [FAIL] %s: ch%lu deg=%u exp=%ldus act=%ldus\r\n",
                     tag, (unsigned long)(channel + 1U), (unsigned)deg,
                     (long)exp, (long)act);
  }
  return ok;
}

/* 记一次核对结果并打印(轮内计数, 轮末汇总) */
static void GripTest_Report(const char *tag, uint8_t ok)
{
  Debug_TestReport("grip", tag, &s_gt_steps, &s_gt_err, ok);
}

void Grip_Debug_Init(void)
{
    /* 夹爪机构(TIM1 两路舵机): Grip_Init 内含 Servo_Init + 上电姿态
       (Z 抬起到安全位 + 夹爪张开), debug_init 在 MX_TIM1_Init 之后调用, 时序满足 */
    Grip_Init();
    s_gt_state = GT_INIT;
    s_gt_tick  = HAL_GetTick();
    s_gt_sent  = 0U;
    s_gt_steps = 0U;
    s_gt_err   = 0U;
    UART_Port_Printf(&uart1_port,
                     "[grip] init ok: z=ch%u jaw=ch%u | test: Z down/up + jaw open/close(100%%->0%%) + clamp\r\n",
                     (unsigned)(GRIP_Z_SERVO_CH + 1U), (unsigned)(GRIP_JAW_SERVO_CH + 1U));
}

void Grip_Debug_Run(uint32_t now)
{
    /* ---- 第10层 grip_debug: 夹爪机构(双舵机)测试状态机 ----
       每步两拍: 下发后立刻核对(回报角度/该通道脉宽/忙碌标志) ->
       等动作时长走完 -> 到位核对(角度 + 脉宽 + 终态), 轮末汇总 [PASS]/[FAIL] */
    switch (s_gt_state)
    {
    case GT_INIT:
        if (!s_gt_sent)
        {
            s_gt_sent = 1U;
            Grip_Open();  /* 每轮起点摆回已知姿态: 夹爪张开 + Z 抬起 */
            Grip_ZUp();
            s_gt_tick = now;
            break;
        }
        if (Grip_IsBusy() && (now - s_gt_tick < GRIP_TEST_TO_MS))
        {
            break; /* 等姿态到位(上一轮停在 0% 闭合 + Z 270°) */
        }
        UART_Port_Printf(&uart1_port,
                         "[grip] round start: Z -> %u deg, jaw open -> 100%%/0%% close, clamp checks\r\n",
                         (unsigned)GRIP_Z_DOWN_DEG);
        s_gt_sent  = 0U;
        s_gt_state = GT_Z_DOWN;
        s_gt_tick  = now;
        break;

    case GT_Z_DOWN:
        if (!s_gt_sent)
        {
            uint8_t ok = 1U;
            s_gt_sent = 1U;
            Grip_ZDown();
            ok &= (uint8_t)((Grip_GetZAngle() == GRIP_Z_DOWN_DEG) ? 1U : 0U);
            ok &= Debug_GripPulseOk("Z down", GRIP_Z_SERVO_CH, GRIP_Z_DOWN_DEG);
            ok &= (uint8_t)Grip_ZIsBusy();
            GripTest_Report("Z down: cmd angle+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_ZIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS) /* 动作时长估算偏差过大或舵机没动 */
            {
                GripTest_Report("Z down: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_Z_UP;
            }
            break;
        }
        GripTest_Report("Z down: settled",
                        Debug_GripPulseOk("Z down settled", GRIP_Z_SERVO_CH, GRIP_Z_DOWN_DEG));
        s_gt_sent  = 0U;
        s_gt_state = GT_Z_UP;
        break;

    case GT_Z_UP:
        if (!s_gt_sent)
        {
            uint8_t ok = 1U;
            s_gt_sent = 1U;
            Grip_ZUp();
            ok &= (uint8_t)((Grip_GetZAngle() == GRIP_Z_UP_DEG) ? 1U : 0U);
            ok &= Debug_GripPulseOk("Z up", GRIP_Z_SERVO_CH, GRIP_Z_UP_DEG);
            ok &= (uint8_t)Grip_ZIsBusy();
            GripTest_Report("Z up: cmd angle+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_ZIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                GripTest_Report("Z up: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_JAW_OPEN;
            }
            break;
        }
        GripTest_Report("Z up: settled",
                        Debug_GripPulseOk("Z up settled", GRIP_Z_SERVO_CH, GRIP_Z_UP_DEG));
        s_gt_sent  = 0U;
        s_gt_state = GT_JAW_OPEN;
        break;

    case GT_JAW_OPEN:
        if (!s_gt_sent)
        {
            uint8_t ok = 1U;
            s_gt_sent = 1U;
            Grip_Open();
            ok &= (uint8_t)((Grip_GetJawAngle() == GRIP_JAW_OPEN_DEG) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetJawState() == GRIP_JAW_OPENING) ? 1U : 0U);
            ok &= Debug_GripPulseOk("jaw open", GRIP_JAW_SERVO_CH, GRIP_JAW_OPEN_DEG);
            ok &= (uint8_t)Grip_JawIsBusy();
            GripTest_Report("jaw open: cmd angle+state+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_JawIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                GripTest_Report("jaw open: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_JAW_CLOSE_MAX;
            }
            break;
        }
        GripTest_Report("jaw open: settled(state+pulse)",
                        (uint8_t)(((Grip_GetJawState() == GRIP_JAW_OPEN) ? 1U : 0U) &
                                  Debug_GripPulseOk("jaw open settled", GRIP_JAW_SERVO_CH,
                                                    GRIP_JAW_OPEN_DEG)));
        s_gt_sent  = 0U;
        s_gt_state = GT_JAW_CLOSE_MAX;
        break;

    case GT_JAW_CLOSE_MAX:
        if (!s_gt_sent)
        {
            uint8_t ok = 1U;
            s_gt_sent = 1U;
            Grip_Close(100U); /* 100% 力度 = 全闭角(GRIP_JAW_CLOSE_DEG) */
            ok &= (uint8_t)((Grip_GetJawAngle() == GRIP_JAW_CLOSE_DEG) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetJawState() == GRIP_JAW_CLOSING) ? 1U : 0U);
            ok &= Debug_GripPulseOk("jaw close 100%", GRIP_JAW_SERVO_CH, GRIP_JAW_CLOSE_DEG);
            ok &= (uint8_t)Grip_JawIsBusy();
            GripTest_Report("jaw close 100%: cmd angle+state+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_JawIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                GripTest_Report("jaw close 100%: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_JAW_CLOSE_MIN;
            }
            break;
        }
        /* 力度 >0 的闭合到位按开环口径报"夹住"(HELD) */
        GripTest_Report("jaw close 100%: settled(state+pulse)",
                        (uint8_t)(((Grip_GetJawState() == GRIP_JAW_HELD) ? 1U : 0U) &
                                  Debug_GripPulseOk("jaw close 100% settled", GRIP_JAW_SERVO_CH,
                                                    GRIP_JAW_CLOSE_DEG)));
        s_gt_sent  = 0U;
        s_gt_state = GT_JAW_CLOSE_MIN;
        break;

    case GT_JAW_CLOSE_MIN:
        if (!s_gt_sent)
        {
            uint8_t  ok  = 1U;
            uint16_t exp = Grip_CloseAngle(0U); /* 闭合固定为 0° */
            s_gt_sent = 1U;
            Grip_Close(0U);
            ok &= (uint8_t)((exp == GRIP_JAW_CLOSE_DEG) ? 1U : 0U); /* 闭合固定为 0° */
            ok &= (uint8_t)((Grip_GetJawAngle() == exp) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetJawState() == GRIP_JAW_CLOSING) ? 1U : 0U);
            ok &= Debug_GripPulseOk("jaw close 0%", GRIP_JAW_SERVO_CH, exp);
            ok &= (uint8_t)Grip_JawIsBusy();
            GripTest_Report("jaw close 0%: cmd angle+state+pulse+busy", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_JawIsBusy())
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                GripTest_Report("jaw close 0%: settled(timeout)", 0U);
                s_gt_sent  = 0U;
                s_gt_state = GT_CLAMP;
            }
            break;
        }
        /* 力度 =0 的闭合到位报"合上"(CLOSED, 未夹持) */
        GripTest_Report("jaw close 0%: settled(state+pulse)",
                        (uint8_t)(((Grip_GetJawState() == GRIP_JAW_CLOSED) ? 1U : 0U) &
                                  Debug_GripPulseOk("jaw close 0% settled", GRIP_JAW_SERVO_CH,
                                                    Grip_CloseAngle(0U))));
        s_gt_sent  = 0U;
        s_gt_state = GT_CLAMP;
        break;

    case GT_CLAMP:
        if (!s_gt_sent)
        {
            Grip_Result_e rz;
            Grip_Result_e rc;
            uint8_t ok = 1U;

            s_gt_sent = 1U;
            rz = Grip_MoveZ(300U);  /* 超上限 -> 钳到 270° */
            rc = Grip_Close(150U);  /* 力度 >100% -> 拒绝, 保持上一步输出 */
            ok &= (uint8_t)((rz == GRIP_OK) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetZAngle() == SERVO_ANGLE_270_MAX) ? 1U : 0U);
            ok &= Debug_GripPulseOk("Z clamp 300->270", GRIP_Z_SERVO_CH, SERVO_ANGLE_270_MAX);
            ok &= (uint8_t)((rc == GRIP_ERR_PARAM) ? 1U : 0U);
            ok &= (uint8_t)((Grip_GetJawAngle() == Grip_CloseAngle(0U)) ? 1U : 0U);
            GripTest_Report("Z clamp 300->270 + force>100 rejected", ok);
            s_gt_tick = now;
            break;
        }
        if (Grip_IsBusy()) /* 等 Z 轴 300->270 这一步走完再收轮 */
        {
            if (now - s_gt_tick >= GRIP_TEST_TO_MS)
            {
                s_gt_sent  = 0U;
                s_gt_state = GT_REPORT;
            }
            break;
        }
        s_gt_sent  = 0U;
        s_gt_state = GT_REPORT;
        break;

    case GT_REPORT:
        if (now - s_gt_tick >= GRIP_TEST_GAP_MS)
        {
            UART_Port_Printf(&uart1_port, "[grip] round done: steps=%lu err=%lu [%s]\r\n",
                             (unsigned long)s_gt_steps, (unsigned long)s_gt_err,
                             (s_gt_err == 0U) ? "PASS" : "FAIL");
            s_gt_steps = 0U;
            s_gt_err   = 0U;
            s_gt_sent  = 0U;
            s_gt_state = GT_INIT;
            s_gt_tick  = now;
        }
        break;

    default:
        s_gt_state = GT_INIT;
        break;
    }
}

#endif /* grip_debug == 1 */
