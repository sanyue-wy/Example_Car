/**
 * @file    vofa.c
 * @brief   VOFA+ 上位机调试工具(实现层)
 *          参考 dev_vofa 实现改写, 输出与接收全部走 drv_uart 对象层
 * @version 1.0
 * @date    2026-09-25
 */

/* Includes ------------------------------------------------------------------*/
#include "vofa.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* justfloat 组帧缓冲上限: 最大通道数(帧 = 4*count + 4 字节) */
#define VOFA_MAX_CH      16
#define VOFA_FRAME_BUF   (VOFA_MAX_CH * 4 + 4)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Vofa_t vofa;

/**
  * @brief  命令名 -> Vofa_Para_t 字段 偏移表(表驱动, 扩展命令只需加一行)
  * @note   no_arg=1 的命令不带 "=数值", 收到即把 value 置为 1(置位/清零语义见 GO/STP)
  */
static const struct
{
  const char *cmd;
  size_t      offset;
  uint8_t     no_arg;
} s_cmd_map[] = {
    {"P1", offsetof(Vofa_Para_t, speed_kp), 0U},
    {"I1", offsetof(Vofa_Para_t, speed_ki), 0U},
    {"D1", offsetof(Vofa_Para_t, speed_kd), 0U},
    {"P2", offsetof(Vofa_Para_t, position_kp), 0U},
    {"I2", offsetof(Vofa_Para_t, position_ki), 0U},
    {"D2", offsetof(Vofa_Para_t, position_kd), 0U},
    {"Spe", offsetof(Vofa_Para_t, speed_target), 0U},
    {"Ang", offsetof(Vofa_Para_t, angle_target), 0U},
    {"MAX", offsetof(Vofa_Para_t, limit), 0U},
    {"GO", offsetof(Vofa_Para_t, run_flag), 1U},
    {"STP", offsetof(Vofa_Para_t, run_flag), 1U},
    /* 后续扩展命令在这里加一行 */
};

/* Private function declarations ---------------------------------------------*/

static void Vofa_ParseCommand(Vofa_t *v);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  解析一帧完整命令, 暂存后立即推送
  *         两种帧形:
  *           带参: "名字=数值!"  例 Spe=300!  P1=4.99!  MAX=5000!
  *           无参: "名字!"       例 GO!  STP!      (查表 no_arg=1)
  * @note   在中断上下文执行: 不做任何打印(阻塞打印在中断里可能与
  *         SysTick 死锁), 参考实现逐字节打印的调试代码已移除
  */
static void Vofa_ParseCommand(Vofa_t *v)
{
  char cmd[8];
  char num[24];
  uint8_t head = 0xFFU;
  uint8_t tail = 0xFFU;
  uint8_t cmd_len;
  uint8_t num_len;
  uint8_t hit = 0U;
  float value = 0.0f;

  /* 定位帧头 '=' 与帧尾 '!'; 无参命令没有 '=', 此时 head 保持 0xFF */
  for (uint8_t i = 0; i < v->rx_line; i++)
  {
    if (v->cmd_buff[i] == VOFA_FRAME_HEAD && head == 0xFFU)
    {
      head = i;
    }
    if (v->cmd_buff[i] == VOFA_FRAME_TAIL)
    {
      tail = i;
      break;
    }
  }
  if (tail == 0xFFU || tail == 0U)
  {
    return; /* 帧尾不对, 丢弃 */
  }

  if (head == 0xFFU)
  {
    /* 无参帧: 帧尾之前整段都是命令名 */
    cmd_len = tail;
    num_len = 0U;
  }
  else if (head == 0U)
  {
    /* 命令名为空(收到 "=5!" 这种噪声): 命令名要从 cmd_buff[0] 取, 与数值区
       的 '=' 重叠, 不判掉的话后面 strcmp 会拿一个空名字乱匹配, 把增益改成 0。 */
    return;
  }
  else
  {
    cmd_len = head;             /* 帧头之前是命令名 */
    num_len = tail - head - 1U; /* 帧头与帧尾之间是数值 */
  }

  if (cmd_len == 0U || cmd_len >= sizeof(cmd))
  {
    return;
  }
  memcpy(cmd, v->cmd_buff, cmd_len);
  cmd[cmd_len] = '\0';

  /* 先查表: 无参命令不碰 atof(省一次浮点解析), 带参命令再取数值 */
  for (uint8_t i = 0; i < sizeof(s_cmd_map) / sizeof(s_cmd_map[0]); i++)
  {
    if (strcmp(cmd, s_cmd_map[i].cmd) != 0)
    {
      continue;
    }
    if (s_cmd_map[i].no_arg != 0U)
    {
      /* GO! 置 1, STP! 置 0 */
      value = (strcmp(cmd, "STP") == 0) ? 0.0f : 1.0f;
    }
    else
    {
      if (num_len == 0U || num_len >= sizeof(num))
      {
        return; /* 带参命令缺数值 */
      }
      memcpy(num, &v->cmd_buff[head + 1U], num_len);
      num[num_len] = '\0';
      /* 数值里必须至少含有一个数字: atof("abc") 会静默返回 0, 那样一条打错的
         "P2=abc!" 就把角度环增益清零了 —— 调参界面上宁可不动作, 也不能悄悄
         把增益改成 0 让电机突然卸力。
         用 strpbrk(在串里找)而不是 strspn(只匹配开头连续段): 后者对 "-2.5"
         会因为开头是负号直接返回 0, 把所有负数命令都误杀("P2=-2.5!"/"Ang=-90.5!")。
         注意别漏了负号、小数点、指数这些合法写法。 */
      if (strpbrk(num, "0123456789") == NULL)
      {
        return;
      }
      value = (float)atof(num);
    }
    *(float *)((uint8_t *)&v->para + s_cmd_map[i].offset) = value;
    hit = 1U;
    break;
  }

  if (hit != 0U)
  {
    Vofa_ApplyParams(v);
  }
  /* 未匹配的命令静默丢弃 */
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  初始化 VOFA 工具并接管串口接收
  */
void Vofa_Init(Vofa_t *v, UART_PortObject *port)
{
  if (v == NULL)
  {
    return;
  }

  *v = (Vofa_t){0};
  v->port = port;

  if (port != NULL)
  {
    port->on_rx = Vofa_UartRxCallback; /* 该串口接收由 VOFA 独占 */
  }
}

/**
  * @brief  绑定速度环(命令 P1/I1/D1/Spe/MAX)
  * @note   同时解绑位置环: 一次只调一环, 避免 P2/I2/D2 误改角度环参数
  */
void Vofa_BindSpeedPid(Vofa_t *v, PID_Controller_t *pid, float *target, float *limit)
{
  if (v == NULL)
  {
    return;
  }

  v->speed_pid = pid;
  v->speed_target = target;
  v->limit_out = limit;
  v->position_pid = NULL; /* 一次只调一环 */
  v->angle_target = NULL;

  /* 用 PID 当前值回填暂存, 防止没收到命令就推送零值把增益清掉 */
  if (pid != NULL)
  {
    v->para.speed_kp = pid->Kp;
    v->para.speed_ki = pid->Ki;
    v->para.speed_kd = pid->Kd;
    if (limit != NULL)
    {
      v->para.limit = *limit;
    }
  }
}

/**
  * @brief  绑定位置环(命令 P2/I2/D2/Ang/MAX)
  * @note   同时解绑速度环: 一次只调一环, 避免 P1/I1/D1 误改内环参数
  */
void Vofa_BindPositionPid(Vofa_t *v, PID_Controller_t *pid, float *target, float *limit)
{
  if (v == NULL)
  {
    return;
  }

  v->position_pid = pid;
  v->angle_target = target;
  v->limit_out = limit;
  v->speed_pid = NULL; /* 一次只调一环 */
  v->speed_target = NULL;

  if (pid != NULL)
  {
    v->para.position_kp = pid->Kp;
    v->para.position_ki = pid->Ki;
    v->para.position_kd = pid->Kd;
    if (limit != NULL)
    {
      v->para.limit = *limit;
    }
  }
}

/**
  * @brief  串口接收回调: 装配命令帧, 收到帧尾即解析
  * @note   中断上下文执行; 同一事件里的多个命令帧会依次处理
  */
void Vofa_UartRxCallback(UART_PortObject *port, uint8_t *data, uint16_t size)
{
  Vofa_t *v = &vofa;

  if (v->port == NULL || v->port != port || data == NULL)
  {
    return;
  }

  for (uint16_t i = 0; i < size; i++)
  {
    /* 修复参考实现的缓冲溢出: 帧尾一直不来时强制复位重新同步 */
    if (v->rx_line >= VOFA_CMD_MAX)
    {
      v->rx_line = 0;
    }

    /* 残帧重同步: 待收缓冲里已经有 '=' 却迟迟等不到帧尾(噪声/半帧), 此时又来了
       一条新命令的开头, 说明上一帧已经废了 —— 丢弃残帧, 从当前字节重新组帧。
       两种触发(满足其一即重同步):
         a) 来的是字母 —— 数值里不会出现字母, 所以"含 '=' 的残帧 + 字母"不可能
            合法续传。修的是 "P2=1" + "P2=3.0!" 被拼成 "P2=1P2=3.0" -> atof 得 1.0,
            电机拿着错误的增益跑;
         b) 来的是第二个 '=' —— 数值里不会出现 '='。修的是 "P2=" + "=5!" 被拼成
            "P2==5!" -> atof("=5") 静默得 0, 角度环增益被清零、电机直接卸力。
            注意这条必须先于 (a) 生效: "P2=" 后到的正好是 '=' 而不是字母。
       正常拆帧("P2" + "=3.0!" / "P2=3" + ".0!")的下一字节都不是字母也不是 '=',
       不受影响。 */
    if (v->rx_line > 0U &&
        memchr(v->cmd_buff, VOFA_FRAME_HEAD, v->rx_line) != NULL &&
        (data[i] == VOFA_FRAME_HEAD ||
         (data[i] >= 'A' && data[i] <= 'Z')))
    {
      v->rx_line = 0;
    }

    v->cmd_buff[v->rx_line] = data[i];
    v->rx_line++;

    if (data[i] == VOFA_FRAME_TAIL)
    {
      Vofa_ParseCommand(v);
      /* 无条件复位: Vofa_ParseCommand 遇到格式不对的命令(缺数值/没有数字/命令名
         超长)是提前 return 的, 如果复位放在解析成功之后, 那种残帧就会留在缓冲里,
         下一条命令被拼在它后面一起解析(实测 "P2=abc!" + "P2=-2.5!" -> atof("abc!P2=-2.5")
         = 0, 角度环增益被清成 0)。帧尾已经到过, 残帧必定作废, 无条件清。 */
      v->rx_line = 0;
      memset(v->cmd_buff, 0, sizeof(v->cmd_buff));
      /* 不 break: 一个事件里背靠背的多帧命令继续依次解析 */
    }
  }
}

/**
  * @brief  把暂存参数推送到已绑定的 PID 与目标变量
  * @note   单个 float 在 Cortex-M4 上是原子写, 中断里推送安全
  */
void Vofa_ApplyParams(Vofa_t *v)
{
  if (v == NULL)
  {
    return;
  }

  if (v->speed_pid != NULL)
  {
    v->speed_pid->Kp = v->para.speed_kp;
    v->speed_pid->Ki = v->para.speed_ki;
    v->speed_pid->Kd = v->para.speed_kd;
  }
  if (v->position_pid != NULL)
  {
    v->position_pid->Kp = v->para.position_kp;
    v->position_pid->Ki = v->para.position_ki;
    v->position_pid->Kd = v->para.position_kd;
  }
  if (v->speed_target != NULL)
  {
    *v->speed_target = v->para.speed_target;
  }
  if (v->angle_target != NULL)
  {
    *v->angle_target = v->para.angle_target;
  }
  if (v->limit_out != NULL)
  {
    /* MAX 语义随绑定对象: 角度环 max_out(限速 r/min) 或速度环 MaxOut(限流 mA) */
    *v->limit_out = v->para.limit;
  }
}

/**
  * @brief  发送一帧波形数据(50ms 节流)
  */
void Vofa_Display(Vofa_t *v,
                  float position_target, float position_actual, float position_out,
                  float speed_target, float speed_actual, float speed_out)
{
  if (v == NULL || v->port == NULL)
  {
    return;
  }

  /* 50ms 节流 */
  if (uwTick - v->tx_tick < VOFA_TX_PERIOD_MS)
  {
    return;
  }
  v->tx_tick = uwTick;

#if VOFA_DATA_FORMAT == 1
  /* justfloat: 6 个小端浮点 + 帧尾 00 00 80 7F */
  {
    float values[6] = {
        position_target, position_actual, position_out,
        speed_target, speed_actual, speed_out};
    Vofa_SendFloats(v, values, 6);
  }
#else
  /* firewater: 文本 CSV, VOFA+ 按逗号分隔解析 */
  UART_Port_Printf(v->port,
                   "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                   position_target, position_actual, position_out,
                   speed_target, speed_actual, speed_out);
#endif
}

/**
  * @brief  以 justfloat 格式发送任意个浮点数
  * @note   组帧缓冲必须是静态的: UART_Port_Send 走 DMA,
  *         函数返回后 DMA 仍在读该缓冲, 栈上的临时帧会被覆写;
  *         因此上一帧未发完时直接丢弃本拍, 绝不能先改写缓冲再发送
  * @retval 0 成功; 1 参数错误; 2 上一帧仍在发送, 本拍丢弃
  */
uint8_t Vofa_SendFloats(Vofa_t *v, const float *data, uint16_t count)
{
  static uint8_t frame[VOFA_FRAME_BUF];

  if (v == NULL || v->port == NULL || data == NULL ||
      count == 0U || count > VOFA_MAX_CH)
  {
    return 1;
  }

  /* 上一帧 DMA 还在读 frame: 先判忙, 忙则丢弃, 不能 memcpy */
  if (UART_Port_TxBusy(v->port))
  {
    return 2;
  }

  memcpy(frame, data, count * 4U);
  frame[count * 4U + 0U] = 0x00;
  frame[count * 4U + 1U] = 0x00;
  frame[count * 4U + 2U] = 0x80;
  frame[count * 4U + 3U] = 0x7F;

  return UART_Port_Send(v->port, frame, (uint16_t)(count * 4U + 4U));
}
