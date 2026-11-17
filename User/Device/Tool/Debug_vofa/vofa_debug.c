#include "debug_config.h"

#if vofa_debug == 1

#include "vofa_debug.h"
#include "vofa.h"
#include "main.h"
#include "drv_uart.h"
#include "drv_can.h"
#include "drv_tim.h"
#include "task_it.h"
#include "tim.h"
#include <string.h>

static PID_Controller_t s_vofa_pid;   /* 速度环测试实例 */
static float s_vofa_speed_target = 0; /* VOFA 命令 Spe 写入这里 */

void Vofa_Debug_Init(void)
{
    Vofa_Init(&vofa, &uart1_port);            /* 接管 USART1 接收(与 uart_debug 互斥) */
    PID_Init(&s_vofa_pid, 1.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 0.001f);
    Vofa_BindSpeedPid(&vofa, &s_vofa_pid, &s_vofa_speed_target, &s_vofa_pid.MaxOut);
    UART_Port_Printf(&uart1_port, "[vofa] init ok, try 'Spe=10!'\r\n");
}

void Vofa_Debug_Run(uint32_t now)
{
    {
        static uint32_t s_last = 0;
        if (now - s_last >= 50U) /* 20Hz 波形, 与 VOFA_TX_PERIOD_MS 一致 */
        {
            s_last = now;
            float wave = (float)(TIM_Timer_GetIrqCount(&tim7_timer) % 1000U); /* 锯齿波 0~999 */
            float ch[4] = {wave, s_vofa_speed_target, s_vofa_pid.Kp, s_vofa_pid.Output};
            Vofa_SendFloats(&vofa, ch, 4);                /* VOFA+ 设成 justfloat 接收 */
        }
    }
}

#endif /* vofa_debug == 1 */
