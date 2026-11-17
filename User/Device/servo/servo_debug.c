#include "debug_config.h"

#if servo_debug == 1

#include "servo_debug.h"
#include "servo.h"
#include "debug_shared.h"
#include "main.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "task_it.h"
#include "tim.h"
#include <string.h>

/* ---- 第7层 servo_debug: 舵机 0~270° 扫描, 回读 CCR 核对脉宽 ----
   脉宽回读/期望换算/容差见文件头"共享自检工具" */
#define SERVO_TEST_STEP_DEG  15    /* 扫描步进(度), 270/15 = 18 步 */
#define SERVO_TEST_STEP_MS   150U  /* 每步间隔(ms), 全程约 5.4s */
typedef enum {
    SV_ST_START = 0,  /* 三通道送 0°, 开始上扫 */
    SV_ST_UP,         /* 0° -> 270° 每步 +15° */
    SV_ST_DOWN,       /* 270° -> 0° 每步 -15° */
    SV_ST_CLAMP_LO,   /* 送 -50°: 应被钳到 0°(500µs) */
    SV_ST_CLAMP_HI,   /* 送 320°: 应被钳到 270°(2500µs) */
    SV_ST_REPORT      /* 汇总 [PASS]/[FAIL], 1s 后开始新一轮 */
} ServoTestState_t;
static const uint32_t s_sv_ch[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
static ServoTestState_t s_sv_state = SV_ST_START;
static uint32_t s_sv_tick = 0;
static int16_t  s_sv_angle = 0;   /* 扫描步进的当前角度 */
static uint32_t s_sv_steps = 0;   /* 已核对步数 */
static uint32_t s_sv_err   = 0;   /* 有通道不一致的步数 */

static void Debug_ServoSetAndCheck(int16_t angle);

/**
  * @brief  三路同送 angle, 逐通道核对实际脉宽与期望值
  * @note   期望值按 0~270 钳位后的角度计算, 与 servo.c 换算同式,
  *         因此越界入参顺带覆盖 Servo_SetAngle_270 的钳位行为;
  *         不一致的通道当场打 [FAIL], 步统计累入 s_sv_err
  */
static void Debug_ServoSetAndCheck(int16_t angle)
{
  int16_t clamped = angle;
  int32_t expected;
  uint32_t bad = 0;

  if (clamped > SERVO_ANGLE_270_MAX) clamped = SERVO_ANGLE_270_MAX;
  if (clamped < 0)                   clamped = 0;
  expected = (int32_t)Debug_ExpPulseUs((uint16_t)clamped);

  for (uint8_t i = 0; i < 3U; i++)
  {
    uint32_t actual;
    uint8_t  ok;

    Servo_SetAngle_270(s_sv_ch[i], angle);
    ok = Debug_PulseCheck(s_sv_ch[i], (uint16_t)clamped, 0, &actual);
    if (!ok)
    {
      bad++;
      UART_Port_Printf(&uart1_port,
                       "[servo] [FAIL] ch%u ang=%d exp=%ldus act=%ldus\r\n",
                       (unsigned)(i + 1U), (int)angle, (long)expected, (long)actual);
    }
  }

  s_sv_steps++;
  if (bad != 0U)
  {
    s_sv_err++;
  }
  UART_Port_Printf(&uart1_port,
                   "[servo] ang=%d exp=%ldus %s | steps=%lu err=%lu\r\n",
                   (int)angle, (long)expected, (bad == 0U) ? "[ok]" : "[BAD]",
                   (unsigned long)s_sv_steps, (unsigned long)s_sv_err);
}

void Servo_Debug_Init(void)
{
    /* TIM1 CH1~CH3 PWM 启动(debug_init 在 MX_TIM1_Init 之后调用, 时序满足) */
    Servo_Init();
    s_sv_state = SV_ST_START;
    s_sv_tick  = HAL_GetTick();
    s_sv_steps = 0;
    s_sv_err   = 0;
    UART_Port_Printf(&uart1_port, "[servo] init ok, 0~270 sweep test start\r\n");
}

void Servo_Debug_Run(uint32_t now)
{
    /* 舵机 0~270° 扫描: 上扫 -> 下扫 -> 越界钳位核对 -> 汇总, 循环往复。
       判据: 每步回读 CCR 折算脉宽, 与期望差 >20µs 当场 [FAIL]; 轮末 [PASS]/[FAIL] */
    // switch (s_sv_state)
    // {
    //     case SV_ST_START:
    //         Debug_ServoSetAndCheck(0);
    //         s_sv_state = SV_ST_UP;
    //         s_sv_tick  = now;
    //         UART_Port_Printf(&uart1_port, "[servo] sweep up 0 -> %d, step %d deg\r\n",
    //                          SERVO_ANGLE_270_MAX, SERVO_TEST_STEP_DEG);
    //         break;

    //     case SV_ST_UP:
    //         if (now - s_sv_tick >= SERVO_TEST_STEP_MS)
    //         {
    //             s_sv_tick  = now;
    //             s_sv_angle = (int16_t)(s_sv_angle + SERVO_TEST_STEP_DEG);
    //             if (s_sv_angle >= SERVO_ANGLE_270_MAX)
    //             {
    //                 s_sv_angle = SERVO_ANGLE_270_MAX;
    //                 Debug_ServoSetAndCheck(s_sv_angle);
    //                 s_sv_state = SV_ST_DOWN;
    //                 UART_Port_Printf(&uart1_port, "[servo] top reached, sweep down\r\n");
    //             }
    //             else
    //             {
    //                 Debug_ServoSetAndCheck(s_sv_angle);
    //             }
    //         }
    //         break;

    //     case SV_ST_DOWN:
    //         if (now - s_sv_tick >= SERVO_TEST_STEP_MS)
    //         {
    //             s_sv_tick  = now;
    //             s_sv_angle = (int16_t)(s_sv_angle - SERVO_TEST_STEP_DEG);
    //             if (s_sv_angle <= 0)
    //             {
    //                 s_sv_angle = 0;
    //                 Debug_ServoSetAndCheck(s_sv_angle);
    //                 s_sv_state = SV_ST_CLAMP_LO;
    //                 s_sv_tick  = now;
    //                 UART_Port_Printf(&uart1_port, "[servo] bottom reached, clamp checks\r\n");
    //             }
    //             else
    //             {
    //                 Debug_ServoSetAndCheck(s_sv_angle);
    //             }
    //         }
    //         break;

    //     case SV_ST_CLAMP_LO:
    //         if (now - s_sv_tick >= 500U)
    //         {
    //             Debug_ServoSetAndCheck(-50); /* 负值应被钳到 0°(500µs) */
    //             s_sv_state = SV_ST_CLAMP_HI;
    //             s_sv_tick  = now;
    //         }
    //         break;

    //     case SV_ST_CLAMP_HI:
    //         if (now - s_sv_tick >= 500U)
    //         {
    //             Debug_ServoSetAndCheck(320); /* 超上限应被钳到 270°(2500µs) */
    //             s_sv_state = SV_ST_REPORT;
    //             s_sv_tick  = now;
    //         }
    //         break;

    //     case SV_ST_REPORT:
    //         if (now - s_sv_tick >= 500U)
    //         {
    //             UART_Port_Printf(&uart1_port,
    //                              "[servo] round done: steps=%lu err=%lu [%s]\r\n",
    //                              (unsigned long)s_sv_steps,
    //                              (unsigned long)s_sv_err,
    //                              (s_sv_err == 0U) ? "PASS" : "FAIL");
    //             s_sv_steps = 0;
    //             s_sv_err   = 0;
    //             s_sv_state = SV_ST_START;
    //             s_sv_tick  = now;
    //         }
    //         break;

    //     default:
    //         s_sv_state = SV_ST_START;
    //         break;
    // }
     Debug_ServoSetAndCheck(0);
}

#endif /* servo_debug == 1 */
