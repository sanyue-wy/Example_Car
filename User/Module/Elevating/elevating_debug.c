/**
 * @file    elevating_debug.c
 * @brief   丝杆升降平台模块自检(实现层)
 *          位置闭环 + 相对移动 + 软限位拒绝; [PASS]/[FAIL] 输出
 * @version 1.0
 * @date    2026-11-17
 *
 * 硬件: TIM1_CH1(PA8) 20kHz PWM + PB0/PB1 H 桥 + TIM2 编码器,
 *       与 encoder_motor_debug 相同, 二者只能开一个。
 *
 * !!! 丝杆会动! 运行前确认:
 *   1. 平台在行程内、上方无遮挡, 人手能随时断电;
 *   2. 平台先停在机械下限位附近(本测试把它当前位置当作零点);
 *   3. 先目视确认方向: 位置从 0 走向 +5mm 时平台应"向上"。
 *      若实际向下, 只改 elevating.c 顶部的 ELEVATING_UP_SIGN 为 -1.0f;
 *   4. 导程 ELEVATING_SCREW_LEAD_MM / 行程 / PID 也在同文件顶部按实车改。
 */

#include "debug_config.h"

#if elevating_debug == 1

#include "elevating_debug.h"
#include "elevating.h"
#include "drv_uart.h"

/* Private macros ------------------------------------------------------------*/

#define EVT_TASK_PERIOD_MS   10U      /* 模块 Task 周期 */
#define EVT_STATUS_PERIOD_MS 1000U    /* 状态打印周期 */
#define EVT_DELTA_MM         5.0f     /* 绝对移动测试距离 */
#define EVT_REL_MM           2.0f     /* 相对移动测试距离 */
#define EVT_TOL_MM           0.60f    /* 到位位置容差 */
#define EVT_WAIT_MS          15000U   /* 单个动作等待上限(模块自身还有超时) */

/* Private types -------------------------------------------------------------*/

typedef enum
{
  EVT_ST_INIT = 0,
  EVT_ST_UP_WAIT,
  EVT_ST_BACK_WAIT,
  EVT_ST_REL_UP_WAIT,
  EVT_ST_REL_BACK_WAIT,
  EVT_ST_LIMIT,
  EVT_ST_REPORT,
  EVT_ST_DONE
} Evt_State_e;

typedef struct
{
  Elevating_Object lift;

  Evt_State_e state;
  uint32_t    state_tick;
  uint32_t    last_task_ms;
  uint32_t    last_status_ms;
  uint32_t    pass_count;
  uint32_t    fail_count;
} Evt_Context_t;

/* Private variables ---------------------------------------------------------*/

static Evt_Context_t s_evt;

/* Private function declarations ---------------------------------------------*/

static void Evt_CheckPosition(const char *tag, float expected);
static void Evt_StartMove(float target_mm, Evt_State_e next, uint32_t now);
static void Evt_StartRelativeMove(float delta_mm, Evt_State_e next, uint32_t now);
static void Evt_PrintStatus(void);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  核对当前位置并累计 [PASS]/[FAIL]
  */
static void Evt_CheckPosition(const char *tag, float expected)
{
  float   pos = Elevating_GetPositionMm(&s_evt.lift);
  float   err = pos - expected;
  uint8_t ok  = (FLOAT_ABS(err) <= EVT_TOL_MM) ? 1U : 0U;

  if (ok != 0U)
  {
    s_evt.pass_count++;
    UART_Port_Printf(&uart1_port, "[PASS] %s: pos=%.2fmm target=%.2fmm err=%.2fmm\r\n",
                     tag, pos, expected, err);
  }
  else
  {
    s_evt.fail_count++;
    UART_Port_Printf(&uart1_port, "[FAIL] %s: pos=%.2fmm target=%.2fmm err=%.2fmm\r\n",
                     tag, pos, expected, err);
  }
}

/**
  * @brief  发起绝对移动; 失败则直接进入汇总
  */
static void Evt_StartMove(float target_mm, Evt_State_e next, uint32_t now)
{
  uint8_t ret = Elevating_MoveTo(&s_evt.lift, target_mm);

  if (ret != 0U)
  {
    s_evt.fail_count++;
    UART_Port_Printf(&uart1_port, "[FAIL] MoveTo(%.2f) ret=%u\r\n", target_mm, ret);
    s_evt.state = EVT_ST_REPORT;
    return;
  }

  UART_Port_Printf(&uart1_port, "[elev] moving to %.2f mm ...\r\n", target_mm);
  s_evt.state     = next;
  s_evt.state_tick = now;
}

/**
  * @brief  发起相对移动; 失败则直接进入汇总
  */
static void Evt_StartRelativeMove(float delta_mm, Evt_State_e next, uint32_t now)
{
  uint8_t ret = Elevating_MoveRelative(&s_evt.lift, delta_mm);

  if (ret != 0U)
  {
    s_evt.fail_count++;
    UART_Port_Printf(&uart1_port, "[FAIL] MoveRelative(%.2f) ret=%u\r\n", delta_mm, ret);
    s_evt.state = EVT_ST_REPORT;
    return;
  }

  UART_Port_Printf(&uart1_port, "[elev] relative move %+.2f mm ...\r\n", delta_mm);
  s_evt.state      = next;
  s_evt.state_tick = now;
}

/**
  * @brief  运行中每秒打印一次位置/速度/占空比
  */
static void Evt_PrintStatus(void)
{
  UART_Port_Printf(&uart1_port,
                   "[elev] state=%u pos=%.2fmm tgt=%.2fmm v=%.2fmm/s rps=%.3f duty=%d\r\n",
                   (unsigned)s_evt.lift.state,
                   s_evt.lift.position_mm,
                   s_evt.lift.target_mm,
                   s_evt.lift.speed_mm_s,
                   EncoderMotor_GetSpeedRps(&s_evt.lift.motor),
                   (int)s_evt.lift.motor.duty);
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  自检初始化(只登记对象, 不动作)
  */
void Elevating_Debug_Init(void)
{
  s_evt.state          = EVT_ST_INIT;
  s_evt.state_tick     = 0U;
  s_evt.last_task_ms   = 0U;
  s_evt.last_status_ms = 0U;
  s_evt.pass_count     = 0U;
  s_evt.fail_count     = 0U;

  UART_Port_Printf(&uart1_port,
                   "\r\n[elev] == Screw lift self-test ==\r\n"
                   "[elev] !! lead screw moves: keep hands clear !!\r\n"
                   "[elev] platform must start near the mechanical bottom stop\r\n"
                   "[elev] direction: position -> + means platform should go UP\r\n"
                   "[elev] if reversed, set ELEVATING_UP_SIGN = -1.0f in elevating.c\r\n");
}

/**
  * @brief  自检周期任务(建议主循环每 1ms 调用, 内部自行按 10ms 调度)
  * @param  now 系统毫秒时基
  */
void Elevating_Debug_Run(uint32_t now)
{
  /* 模块 Task 固定 10ms */
  if ((now - s_evt.last_task_ms) >= EVT_TASK_PERIOD_MS)
  {
    s_evt.last_task_ms = now;
    Elevating_Task(&s_evt.lift);
  }

  if (s_evt.state != EVT_ST_DONE &&
      s_evt.state != EVT_ST_INIT &&
      (now - s_evt.last_status_ms) >= EVT_STATUS_PERIOD_MS)
  {
    s_evt.last_status_ms = now;
    Evt_PrintStatus();
  }

  switch (s_evt.state)
  {
    case EVT_ST_INIT:
    {
      uint8_t ret = Elevating_Init(&s_evt.lift);

      if (ret != 0U)
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port,
                         "[FAIL] Elevating_Init ret=%u (motor_ret=%u)\r\n",
                         ret, s_evt.lift.last_motor_ret);
        s_evt.state = EVT_ST_REPORT;
        break;
      }

      /* 当前位置作为软件零点(平台应停在机械下限位附近) */
      Elevating_SetZero(&s_evt.lift);
      UART_Port_Printf(&uart1_port, "[elev] init ok, zero set\r\n");

      Evt_StartMove(EVT_DELTA_MM, EVT_ST_UP_WAIT, now);
      break;
    }

    case EVT_ST_UP_WAIT:
    {
      if (Elevating_GetState(&s_evt.lift) == ELEVATING_STATE_ERROR)
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port, "[FAIL] up move -> ERROR (timeout/limit/fault)\r\n");
        Elevating_Stop(&s_evt.lift);
        s_evt.state = EVT_ST_REPORT;
      }
      else if (Elevating_IsArrived(&s_evt.lift) != 0U)
      {
        Evt_CheckPosition("abs move up", EVT_DELTA_MM);
        Evt_StartMove(0.0f, EVT_ST_BACK_WAIT, now);
      }
      else if ((now - s_evt.state_tick) > EVT_WAIT_MS)
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port, "[FAIL] up move wait timeout\r\n");
        Elevating_Stop(&s_evt.lift);
        s_evt.state = EVT_ST_REPORT;
      }
      break;
    }

    case EVT_ST_BACK_WAIT:
    {
      if (Elevating_GetState(&s_evt.lift) == ELEVATING_STATE_ERROR)
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port, "[FAIL] return move -> ERROR\r\n");
        Elevating_Stop(&s_evt.lift);
        s_evt.state = EVT_ST_REPORT;
      }
      else if (Elevating_IsArrived(&s_evt.lift) != 0U)
      {
        Evt_CheckPosition("abs move back", 0.0f);
        Evt_StartRelativeMove(EVT_REL_MM, EVT_ST_REL_UP_WAIT, now);
      }
      else if ((now - s_evt.state_tick) > EVT_WAIT_MS)
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port, "[FAIL] return move wait timeout\r\n");
        Elevating_Stop(&s_evt.lift);
        s_evt.state = EVT_ST_REPORT;
      }
      break;
    }

    case EVT_ST_REL_UP_WAIT:
    {
      if (Elevating_GetState(&s_evt.lift) == ELEVATING_STATE_ERROR)
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port, "[FAIL] relative up -> ERROR\r\n");
        Elevating_Stop(&s_evt.lift);
        s_evt.state = EVT_ST_REPORT;
      }
      else if (Elevating_IsArrived(&s_evt.lift) != 0U)
      {
        Evt_CheckPosition("relative move up", EVT_REL_MM);
        Evt_StartRelativeMove(-EVT_REL_MM, EVT_ST_REL_BACK_WAIT, now);
      }
      else if ((now - s_evt.state_tick) > EVT_WAIT_MS)
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port, "[FAIL] relative up wait timeout\r\n");
        Elevating_Stop(&s_evt.lift);
        s_evt.state = EVT_ST_REPORT;
      }
      break;
    }

    case EVT_ST_REL_BACK_WAIT:
    {
      if (Elevating_GetState(&s_evt.lift) == ELEVATING_STATE_ERROR)
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port, "[FAIL] relative back -> ERROR\r\n");
        Elevating_Stop(&s_evt.lift);
        s_evt.state = EVT_ST_REPORT;
      }
      else if (Elevating_IsArrived(&s_evt.lift) != 0U)
      {
        Evt_CheckPosition("relative move back", 0.0f);
        s_evt.state = EVT_ST_LIMIT;
      }
      else if ((now - s_evt.state_tick) > EVT_WAIT_MS)
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port, "[FAIL] relative back wait timeout\r\n");
        Elevating_Stop(&s_evt.lift);
        s_evt.state = EVT_ST_REPORT;
      }
      break;
    }

    case EVT_ST_LIMIT:
    {
      float   bad_target = Elevating_GetMaxMm(&s_evt.lift) + 10.0f;
      uint8_t ret        = Elevating_MoveTo(&s_evt.lift, bad_target);
      uint8_t ok         = (ret == 2U &&
                            Elevating_GetState(&s_evt.lift) != ELEVATING_STATE_MOVING) ? 1U : 0U;

      if (ok != 0U)
      {
        s_evt.pass_count++;
        UART_Port_Printf(&uart1_port,
                         "[PASS] soft limit reject: MoveTo(%.1f) ret=%u\r\n",
                         bad_target, ret);
      }
      else
      {
        s_evt.fail_count++;
        UART_Port_Printf(&uart1_port,
                         "[FAIL] soft limit reject: MoveTo(%.1f) ret=%u state=%u\r\n",
                         bad_target, ret, (unsigned)Elevating_GetState(&s_evt.lift));
        Elevating_Stop(&s_evt.lift);
      }

      s_evt.state = EVT_ST_REPORT;
      break;
    }

    case EVT_ST_REPORT:
    {
      Elevating_Stop(&s_evt.lift);
      UART_Port_Printf(&uart1_port, "[elev] ---- result: pass=%lu fail=%lu ----\r\n",
                       (unsigned long)s_evt.pass_count,
                       (unsigned long)s_evt.fail_count);
      UART_Port_Printf(&uart1_port, "[elev] %s\r\n",
                       (s_evt.fail_count == 0U) ? "[PASS] elevating self-test"
                                                : "[FAIL] elevating self-test");
      s_evt.state = EVT_ST_DONE;
      break;
    }

    case EVT_ST_DONE:
    default:
      break;
  }
}

#endif /* elevating_debug == 1 */