/**
 * @file    at8236.c
 * @brief   四路 AT8236 编码器电机驱动板设备驱动(实现层)
 * @version 1.0
 * @date    2026-11-17
 */

/* Includes ------------------------------------------------------------------*/
#include "at8236.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/

#define AT8236_ACK_NAME_COUNT 10U

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 与 encoder_motor / drv_uart 同构, 并做了如下取舍:
  * 1. 一块驱动板封装为一个 AT8236_Object, 内嵌 UART_PortObject, 不重复实现
  *    串口收发; 业务层只调 AT8236_SetSpeed/SetPwm 等语义接口;
  * 2. 驱动板是 ASCII 流协议, DMA 空闲中断一次可能给来半帧/多帧, 因此在
  *    串口回调里做 $# 组帧状态机, 完整帧才解析; 组帧与解析都在中断内完成,
  *    只写本对象的整字变量, 任务侧读到的值不会撕裂;
  * 3. 运行指令($spd/$pwm)没有应答, 配置指令才有 "<指令>OK" 回显, 所以
  *    ack_frames 只统计已知配置指令的回显, 数据帧按前缀分类计数;
  * 4. 发送用单缓冲 + DMA: 上一帧未发完直接返回 BUSY 而不覆盖缓冲 —— 驱动板
  *    指令最长 30 字节(@115200 约 2.6ms), 正常 10ms 周期调用不会碰到;
  * 5. 可选指令看门狗: Task 仍在跑但业务层忘了刷新速度时, 超时自动停车,
  *    解决 $spd 保持型指令"发一次就一直转"的问题。
  ******************************************************************************
  */

static const char *const s_ack_names[AT8236_ACK_NAME_COUNT] =
{
  "mtype", "deadzone", "mline", "mphase", "wdiameter",
  "mpid", "upload", "read_flash", "read_vol", "flash_reset"
};

/* Private function declarations ---------------------------------------------*/

static void    AT8236_RxCallback(UART_PortObject *port, uint8_t *data, uint16_t size);
static void    AT8236_FeedByte(AT8236_Object *dev, uint8_t byte);
static void    AT8236_DispatchFrame(AT8236_Object *dev);
static uint8_t AT8236_IsAckFrame(const char *frame);
static uint8_t AT8236_ParseI32x4(const char *text, int32_t *out);
static uint8_t AT8236_ParseF32x4(const char *text, float *out);
static uint8_t AT8236_Send(AT8236_Object *dev, const char *fmt, ...);
static int16_t AT8236_ClampCmd(AT8236_Object *dev, int16_t value, int16_t limit);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  串口接收回调(USART 中断上下文): 把本批字节送进组帧状态机
  * @param  port 端口对象(即 &dev->uart)
  * @param  data 接收缓冲
  * @param  size 本批实际字节数
  */
static void AT8236_RxCallback(UART_PortObject *port, uint8_t *data, uint16_t size)
{
  AT8236_Object *dev;

  if (port == NULL || data == NULL)
  {
    return;
  }

  /* uart 是 AT8236_Object 的成员, 由端口对象反查驱动板对象 */
  dev = (AT8236_Object *)((uint8_t *)port - offsetof(AT8236_Object, uart));

  for (uint16_t i = 0; i < size; i++)
  {
    AT8236_FeedByte(dev, data[i]);
  }
}

/**
  * @brief  单字节组帧: $ 开始, # 结束, 中间内容原样保存
  * @note   超长帧只置 overflow, 继续等 # 后整帧丢弃, 不会把半个帧当数据
  */
static void AT8236_FeedByte(AT8236_Object *dev, uint8_t byte)
{
  if (byte == (uint8_t)'$')
  {
    dev->receiving = 1U;
    dev->overflow = 0U;
    dev->frame_len = 0U;
    return;
  }

  if (dev->receiving == 0U)
  {
    return; /* 帧外数据(# 后的 OK、串口噪声等)丢弃 */
  }

  if (byte == (uint8_t)'#')
  {
    dev->receiving = 0U;

    if (dev->overflow != 0U)
    {
      dev->stats.overflow++;
      return;
    }

    dev->frame[dev->frame_len] = '\0';
    AT8236_DispatchFrame(dev);
    return;
  }

  if (dev->frame_len < (uint16_t)(AT8236_FRAME_MAX - 1U))
  {
    dev->frame[dev->frame_len] = (char)byte;
    dev->frame_len++;
  }
  else
  {
    dev->overflow = 1U;
  }
}

/**
  * @brief  前缀匹配(忽略大小写), 驱动板指令大小写都接受、回显也照抄
  */
static uint8_t AT8236_StartsWithCI(const char *text, const char *prefix)
{
  while (*prefix != '\0')
  {
    char a = *text++;
    char b = *prefix++;

    if ((a >= 'A') && (a <= 'Z'))
    {
      a = (char)(a - 'A' + 'a');
    }
    if ((b >= 'A') && (b <= 'Z'))
    {
      b = (char)(b - 'A' + 'a');
    }
    if (a != b)
    {
      return 0U;
    }
  }
  return 1U;
}

static const char *AT8236_SkipSpace(const char *text)
{
  while ((*text == ' ') || (*text == '\t'))
  {
    text++;
  }
  return text;
}

/**
  * @brief  解析 "a,b,c,d" 四个整数(允许逗号/首尾空格)
  * @retval 1 成功; 0 字段数或格式不符
  */
static uint8_t AT8236_ParseI32x4(const char *text, int32_t *out)
{
  int32_t value[4];
  const char *p = text;

  for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
  {
    char *end = NULL;
    long v;

    p = AT8236_SkipSpace(p);
    v = strtol(p, &end, 10);
    if (end == p)
    {
      return 0U;
    }
    value[i] = (int32_t)v;
    p = end;

    p = AT8236_SkipSpace(p);
    if (i < (AT8236_MOTOR_NUM - 1U))
    {
      if (*p != ',')
      {
        return 0U;
      }
      p++;
    }
    else if (*p != '\0')
    {
      return 0U;
    }
  }

  for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
  {
    out[i] = value[i];
  }
  return 1U;
}

/**
  * @brief  解析 "a,b,c,d" 四个浮点数(速度帧用)
  * @retval 1 成功; 0 字段数或格式不符
  */
static uint8_t AT8236_ParseF32x4(const char *text, float *out)
{
  float value[4];
  const char *p = text;

  for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
  {
    char *end = NULL;
    float v;

    p = AT8236_SkipSpace(p);
    v = strtof(p, &end);
    if (end == p)
    {
      return 0U;
    }
    value[i] = v;
    p = end;

    p = AT8236_SkipSpace(p);
    if (i < (AT8236_MOTOR_NUM - 1U))
    {
      if (*p != ',')
      {
        return 0U;
      }
      p++;
    }
    else if (*p != '\0')
    {
      return 0U;
    }
  }

  for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
  {
    out[i] = value[i];
  }
  return 1U;
}

/**
  * @brief  判断帧是否是配置指令的回显帧(驱动板成功时回 "<指令>OK")
  * @note   回显截到 # 为止, 形如 "mtype:2"; "OK" 在帧外, 由 FeedByte 丢弃
  */
static uint8_t AT8236_IsAckFrame(const char *frame)
{
  for (uint8_t i = 0U; i < AT8236_ACK_NAME_COUNT; i++)
  {
    size_t len = strlen(s_ack_names[i]);

    if (AT8236_StartsWithCI(frame, s_ack_names[i]) != 0U)
    {
      char tail = frame[len];

      if ((tail == '\0') || (tail == ':') || (tail == '='))
      {
        return 1U;
      }
    }
  }
  return 0U;
}

/**
  * @brief  完整帧分发: 按前缀写入对象, 未知帧留作诊断
  */
static void AT8236_DispatchFrame(AT8236_Object *dev)
{
  const char *frame = dev->frame;
  int32_t i32[AT8236_MOTOR_NUM];
  float f32[AT8236_MOTOR_NUM];

  dev->stats.rx_frames++;
  dev->frame_seq++;
  dev->last_rx_ms = HAL_GetTick();

  if (AT8236_StartsWithCI(frame, "MAll:") != 0U)
  {
    if (AT8236_ParseI32x4(frame + 5, i32) != 0U)
    {
      for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
      {
        dev->encoder_total[i] = i32[i];
      }
      dev->stats.total_frames++;
    }
    else
    {
      dev->stats.parse_errors++;
    }
  }
  else if (AT8236_StartsWithCI(frame, "MTEP:") != 0U)
  {
    if (AT8236_ParseI32x4(frame + 5, i32) != 0U)
    {
      for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
      {
        dev->encoder_delta[i] = i32[i];
      }
      dev->stats.delta_frames++;
    }
    else
    {
      dev->stats.parse_errors++;
    }
  }
  else if (AT8236_StartsWithCI(frame, "MSPD:") != 0U)
  {
    if (AT8236_ParseF32x4(frame + 5, f32) != 0U)
    {
      for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
      {
        dev->speed_mmps[i] = f32[i];
      }
      dev->stats.speed_frames++;
    }
    else
    {
      dev->stats.parse_errors++;
    }
  }
  else if (AT8236_StartsWithCI(frame, "Battery:") != 0U)
  {
    char *end = NULL;
    float volts = strtof(frame + 8, &end);

    if (end == frame + 8)
    {
      dev->stats.parse_errors++;
    }
    else
    {
      dev->battery_v = volts;
      dev->stats.battery_frames++;
    }
  }
  else
  {
    if (AT8236_IsAckFrame(frame) != 0U)
    {
      dev->stats.ack_frames++;
    }
  }

  /* 最近一条非数据帧原文, 便于调试直连时确认驱动板实际回了什么 */
  if ((AT8236_StartsWithCI(frame, "MAll:") == 0U) &&
      (AT8236_StartsWithCI(frame, "MTEP:") == 0U) &&
      (AT8236_StartsWithCI(frame, "MSPD:") == 0U) &&
      (AT8236_StartsWithCI(frame, "Battery:") == 0U))
  {
    size_t len = strlen(frame);

    if (len >= AT8236_FRAME_MAX)
    {
      len = AT8236_FRAME_MAX - 1U;
    }
    memcpy(dev->last_response, frame, len);
    dev->last_response[len] = '\0';
  }
}

/**
  * @brief  组帧 + DMA 发送一条指令
  * @retval AT8236_OK / AT8236_ERR_PARAM / AT8236_ERR_BUSY
  * @note   发送缓冲是对象内的单缓冲, 上一帧未发完时拒绝新帧(不覆盖在途数据)
  */
static uint8_t AT8236_Send(AT8236_Object *dev, const char *fmt, ...)
{
  va_list ap;
  int len;
  uint8_t ret;

  if ((dev == NULL) || (dev->initialized == 0U) || (fmt == NULL))
  {
    return (uint8_t)AT8236_ERR_PARAM;
  }

  if (UART_Port_TxBusy(&dev->uart) != 0U)
  {
    dev->stats.tx_busy++;
    return (uint8_t)AT8236_ERR_BUSY;
  }

  va_start(ap, fmt);
  len = vsnprintf(dev->tx_buffer, sizeof(dev->tx_buffer), fmt, ap);
  va_end(ap);

  if ((len <= 0) || (len >= (int)sizeof(dev->tx_buffer)))
  {
    return (uint8_t)AT8236_ERR_PARAM;
  }

  ret = UART_Port_Send(&dev->uart, (uint8_t *)dev->tx_buffer, (uint16_t)len);
  if (ret != 0U)
  {
    dev->stats.tx_busy++;
    return (uint8_t)AT8236_ERR_BUSY;
  }

  dev->stats.tx_ok++;
  return (uint8_t)AT8236_OK;
}

/**
  * @brief  运行指令限幅(驱动板对越界值直接忽略, 这里主动钳位并计数)
  */
static int16_t AT8236_ClampCmd(AT8236_Object *dev, int16_t value, int16_t limit)
{
  if (value > limit)
  {
    dev->stats.sat_count++;
    return limit;
  }
  if (value < (int16_t)(-limit))
  {
    dev->stats.sat_count++;
    return (int16_t)(-limit);
  }
  return value;
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  填入本工程 JCA25-370-103K + 67mm 轮的默认配置
  * @param  cfg   待填配置
  * @param  huart 串口句柄(本工程 &huart2)
  * @param  name  调试名, NULL 用 "AT8236"
  */
void AT8236_DefaultConfig(AT8236_Config_t *cfg, UART_HandleTypeDef *huart, const char *name)
{
  if (cfg == NULL)
  {
    return;
  }

  memset(cfg, 0, sizeof(*cfg));
  cfg->name              = (name != NULL) ? name : "AT8236";
  cfg->huart             = huart;
  cfg->motor_type        = AT8236_MOTOR_310;
  cfg->encoder_lines     = 11U;
  cfg->gear_ratio        = 103U;
  cfg->wheel_diameter_mm = 67.0f;
  cfg->deadzone          = 1600U;
  cfg->pid_kp            = 0.8f;   /* 驱动板出厂默认, 仅 ConfigPID 时下发 */
  cfg->pid_ki            = 0.06f;
  cfg->pid_kd            = 0.5f;
  cfg->command_timeout_ms = 0U;    /* 默认不自动停车 */
}

/**
  * @brief  初始化驱动板对象并启动串口 DMA 空闲接收
  * @param  dev 驱动板对象(必须静态分配, 只初始化一次)
  * @param  cfg 配置(整份拷贝)
  * @retval AT8236_OK / AT8236_ERR_PARAM / AT8236_ERR_UART
  * @note   串口需要已由 MX_USART2_UART_Init 初始化, 且 USART2 的 RX/TX DMA
  *         已在 CubeMX 中打开; 失败原因见 dev->uart_init_ret
  *         (1 参数错, 2 注册表满, 3 HAL 启动 DMA 接收失败)
  */
uint8_t AT8236_Init(AT8236_Object *dev, const AT8236_Config_t *cfg)
{
  AT8236_Config_t local;
  uint8_t ret;

  if ((dev == NULL) || (cfg == NULL) || (cfg->huart == NULL))
  {
    return (uint8_t)AT8236_ERR_PARAM;
  }

  if (dev->initialized != 0U)
  {
    return (uint8_t)AT8236_OK; /* 幂等: 重复 Init 不重置在途 DMA */
  }

  local = *cfg; /* 允许 cfg 指向 dev 内部, 先留副本再清对象 */
  memset(dev, 0, sizeof(*dev));
  dev->cfg = local;

  ret = UART_Port_Init(&dev->uart, local.huart,
                       (local.name != NULL) ? local.name : "AT8236",
                       AT8236_RxCallback,
                       dev->rx_buffer, (uint16_t)sizeof(dev->rx_buffer));
  dev->uart_init_ret = ret;

  if (ret != 0U)
  {
    return (uint8_t)AT8236_ERR_UART;
  }

  dev->initialized = 1U;
  return (uint8_t)AT8236_OK;
}

/**
  * @brief  周期任务: 运行指令看门狗
  * @param  dev    驱动板对象
  * @param  now_ms 当前毫秒(HAL_GetTick 或同源节拍)
  * @note   cfg.command_timeout_ms == 0 时本函数不做任何事;
  *         超时后按最近一次指令类型补发 0 速度/0 PWM, 发送失败则下拍重试
  */
void AT8236_Task(AT8236_Object *dev, uint32_t now_ms)
{
  uint8_t ret;

  if ((dev == NULL) || (dev->initialized == 0U))
  {
    return;
  }

  if ((dev->cfg.command_timeout_ms == 0U) || (dev->command_active == 0U))
  {
    return;
  }

  if ((now_ms - dev->last_command_ms) < dev->cfg.command_timeout_ms)
  {
    return;
  }

  if (dev->command_is_pwm != 0U)
  {
    ret = AT8236_SetPwm(dev, 0, 0, 0, 0);
  }
  else
  {
    ret = AT8236_Stop(dev);
  }

  if (ret == (uint8_t)AT8236_OK)
  {
    dev->stats.timeout_stops++;
  }
}

/**
  * @brief  配置电机类型(写入驱动板 flash)
  */
uint8_t AT8236_ConfigMotorType(AT8236_Object *dev, AT8236_MotorType_e type)
{
  uint8_t ret;

  if ((dev == NULL) || ((uint8_t)type < 1U) || ((uint8_t)type > 4U))
  {
    return (uint8_t)AT8236_ERR_PARAM;
  }

  ret = AT8236_Send(dev, "$mtype:%u#", (unsigned)type);
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->cfg.motor_type = type;
  }
  return ret;
}

/**
  * @brief  配置编码器相位线数(每转信号数, JCA25-370 为 11)
  */
uint8_t AT8236_ConfigEncoderLines(AT8236_Object *dev, uint16_t lines)
{
  uint8_t ret;

  if ((dev == NULL) || (lines == 0U) || (lines > 1000U))
  {
    return (uint8_t)AT8236_ERR_RANGE;
  }

  ret = AT8236_Send(dev, "$mline:%u#", (unsigned)lines);
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->cfg.encoder_lines = lines;
  }
  return ret;
}

/**
  * @brief  配置减速比(JCA25-370-103K 为 103)
  */
uint8_t AT8236_ConfigGearRatio(AT8236_Object *dev, uint16_t ratio)
{
  uint8_t ret;

  if ((dev == NULL) || (ratio == 0U))
  {
    return (uint8_t)AT8236_ERR_RANGE;
  }

  ret = AT8236_Send(dev, "$mphase:%u#", (unsigned)ratio);
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->cfg.gear_ratio = ratio;
  }
  return ret;
}

/**
  * @brief  配置轮径(mm), 影响速度指令与速度上报的换算
  */
uint8_t AT8236_ConfigWheelDiameter(AT8236_Object *dev, float diameter_mm)
{
  uint8_t ret;

  if ((dev == NULL) || !(diameter_mm >= 1.0f) || !(diameter_mm <= 1000.0f))
  {
    return (uint8_t)AT8236_ERR_RANGE;
  }

  ret = AT8236_Send(dev, "$wdiameter:%.3f#", (double)diameter_mm);
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->cfg.wheel_diameter_mm = diameter_mm;
  }
  return ret;
}

/**
  * @brief  配置 PWM 死区(0~3600), 用于消除电机最小震荡
  */
uint8_t AT8236_ConfigDeadzone(AT8236_Object *dev, uint16_t deadzone)
{
  uint8_t ret;

  if ((dev == NULL) || (deadzone > 3600U))
  {
    return (uint8_t)AT8236_ERR_RANGE;
  }

  ret = AT8236_Send(dev, "$deadzone:%u#", (unsigned)deadzone);
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->cfg.deadzone = deadzone;
  }
  return ret;
}

/**
  * @brief  配置速度环 PID
  * @note   驱动板收到该指令会重启一次并停止正在转的电机, 属正常现象;
  *         出厂 PID(P0.8/I0.06/D0.5)一般不需要改
  */
uint8_t AT8236_ConfigPID(AT8236_Object *dev, float kp, float ki, float kd)
{
  uint8_t ret;

  if ((dev == NULL) ||
      !(kp >= 0.0f) || !(kp <= 1000.0f) ||
      !(ki >= 0.0f) || !(ki <= 1000.0f) ||
      !(kd >= 0.0f) || !(kd <= 1000.0f))
  {
    return (uint8_t)AT8236_ERR_RANGE;
  }

  ret = AT8236_Send(dev, "$mpid:%.3f,%.3f,%.3f#",
                    (double)kp, (double)ki, (double)kd);
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->cfg.pid_kp = kp;
    dev->cfg.pid_ki = ki;
    dev->cfg.pid_kd = kd;
    /* 驱动板重启会清掉运行指令和上报开关, 同步清本地状态 */
    dev->command_active = 0U;
    dev->upload_total = 0U;
    dev->upload_delta = 0U;
    dev->upload_speed = 0U;
  }
  return ret;
}

/**
  * @brief  设置上报开关
  * @param  encoder_total 1=上报累计编码器 $MAll
  * @param  encoder_delta 1=上报 10ms 计数   $MTEP
  * @param  speed         1=上报速度 mm/s    $MSPD
  * @note   三类同时打开时 115200 波特率占用很高, 建议按当前测试只开需要的
  */
uint8_t AT8236_SetUpload(AT8236_Object *dev, uint8_t encoder_total,
                         uint8_t encoder_delta, uint8_t speed)
{
  uint8_t ret;

  if (dev == NULL)
  {
    return (uint8_t)AT8236_ERR_PARAM;
  }

  encoder_total = (encoder_total != 0U) ? 1U : 0U;
  encoder_delta = (encoder_delta != 0U) ? 1U : 0U;
  speed         = (speed != 0U) ? 1U : 0U;

  ret = AT8236_Send(dev, "$upload:%u,%u,%u#",
                    (unsigned)encoder_total, (unsigned)encoder_delta,
                    (unsigned)speed);
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->upload_total = encoder_total;
    dev->upload_delta = encoder_delta;
    dev->upload_speed = speed;
  }
  return ret;
}

/**
  * @brief  查询电池电压, 应答帧 $Battery:7.40V#
  */
uint8_t AT8236_QueryBattery(AT8236_Object *dev)
{
  return AT8236_Send(dev, "$read_vol#");
}

/**
  * @brief  查询驱动板 flash 变量(应答格式随固件版本, 原文存 last_response)
  */
uint8_t AT8236_QueryFlash(AT8236_Object *dev)
{
  return AT8236_Send(dev, "$read_flash#");
}

/**
  * @brief  恢复驱动板出厂设置
  * @note   驱动板会重启, 本对象缓存的上报开关与运行指令状态一并清零
  */
uint8_t AT8236_ResetFactory(AT8236_Object *dev)
{
  uint8_t ret;

  ret = AT8236_Send(dev, "$flash_reset#");
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->upload_total = 0U;
    dev->upload_delta = 0U;
    dev->upload_speed = 0U;
    dev->command_active = 0U;
  }
  return ret;
}

/**
  * @brief  闭环速度控制(mm/s), 范围 ±1000, 越界自动钳位
  * @note   驱动板不回应答; 给 0 时 PID 仍锁住轮子(区别于 AT8236_Coast)
  */
uint8_t AT8236_SetSpeed(AT8236_Object *dev, int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
  int16_t speed[AT8236_MOTOR_NUM];

  speed[0] = m1;
  speed[1] = m2;
  speed[2] = m3;
  speed[3] = m4;
  return AT8236_SetSpeedArray(dev, speed);
}

uint8_t AT8236_SetSpeedArray(AT8236_Object *dev, const int16_t speed[AT8236_MOTOR_NUM])
{
  int16_t cmd[AT8236_MOTOR_NUM];
  uint8_t ret;

  if ((dev == NULL) || (speed == NULL))
  {
    return (uint8_t)AT8236_ERR_PARAM;
  }

  for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
  {
    cmd[i] = AT8236_ClampCmd(dev, speed[i], (int16_t)AT8236_SPEED_LIMIT);
  }

  ret = AT8236_Send(dev, "$spd:%d,%d,%d,%d#",
                    (int)cmd[0], (int)cmd[1], (int)cmd[2], (int)cmd[3]);
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->command_active = ((cmd[0] != 0) || (cmd[1] != 0) ||
                           (cmd[2] != 0) || (cmd[3] != 0)) ? 1U : 0U;
    dev->command_is_pwm = 0U;
    dev->last_command_ms = HAL_GetTick();
  }
  return ret;
}

/**
  * @brief  开环 PWM 控制, 范围 ±3600, 越界自动钳位
  * @note   驱动板不回应答; 给 0 时滑行(区别于 AT8236_Stop)
  */
uint8_t AT8236_SetPwm(AT8236_Object *dev, int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
  int16_t pwm[AT8236_MOTOR_NUM];

  pwm[0] = m1;
  pwm[1] = m2;
  pwm[2] = m3;
  pwm[3] = m4;
  return AT8236_SetPwmArray(dev, pwm);
}

uint8_t AT8236_SetPwmArray(AT8236_Object *dev, const int16_t pwm[AT8236_MOTOR_NUM])
{
  int16_t cmd[AT8236_MOTOR_NUM];
  uint8_t ret;

  if ((dev == NULL) || (pwm == NULL))
  {
    return (uint8_t)AT8236_ERR_PARAM;
  }

  for (uint8_t i = 0U; i < AT8236_MOTOR_NUM; i++)
  {
    cmd[i] = AT8236_ClampCmd(dev, pwm[i], (int16_t)AT8236_PWM_LIMIT);
  }

  ret = AT8236_Send(dev, "$pwm:%d,%d,%d,%d#",
                    (int)cmd[0], (int)cmd[1], (int)cmd[2], (int)cmd[3]);
  if (ret == (uint8_t)AT8236_OK)
  {
    dev->command_active = ((cmd[0] != 0) || (cmd[1] != 0) ||
                           (cmd[2] != 0) || (cmd[3] != 0)) ? 1U : 0U;
    dev->command_is_pwm = 1U;
    dev->last_command_ms = HAL_GetTick();
  }
  return ret;
}

/**
  * @brief  闭环停车: $spd:0,0,0,0#, PID 保持轮子不动
  */
uint8_t AT8236_Stop(AT8236_Object *dev)
{
  return AT8236_SetSpeed(dev, 0, 0, 0, 0);
}

/**
  * @brief  滑行停车: $pwm:0,0,0,0#, 输出断开靠惯性停下
  */
uint8_t AT8236_Coast(AT8236_Object *dev)
{
  return AT8236_SetPwm(dev, 0, 0, 0, 0);
}

/* Getter ---------------------------------------------------------------------*/

int32_t AT8236_GetEncoderTotal(const AT8236_Object *dev, uint8_t index)
{
  if ((dev == NULL) || (index >= AT8236_MOTOR_NUM))
  {
    return 0;
  }
  return dev->encoder_total[index];
}

int32_t AT8236_GetEncoderDelta(const AT8236_Object *dev, uint8_t index)
{
  if ((dev == NULL) || (index >= AT8236_MOTOR_NUM))
  {
    return 0;
  }
  return dev->encoder_delta[index];
}

float AT8236_GetSpeedMmps(const AT8236_Object *dev, uint8_t index)
{
  if ((dev == NULL) || (index >= AT8236_MOTOR_NUM))
  {
    return 0.0f;
  }
  return dev->speed_mmps[index];
}

float AT8236_GetBatteryVoltage(const AT8236_Object *dev)
{
  if (dev == NULL)
  {
    return 0.0f;
  }
  return dev->battery_v;
}

/**
  * @brief  在线判断: 最近 timeout_ms 内收到过任意完整帧
  * @note   只有打开了 $upload 上报(或刚发过配置/查询指令)才会有帧,
  *         运行指令 $spd/$pwm 本身不产生应答; now_ms 必须与 HAL_GetTick 同源
  */
uint8_t AT8236_IsOnline(const AT8236_Object *dev, uint32_t now_ms, uint32_t timeout_ms)
{
  if ((dev == NULL) || (dev->initialized == 0U) || (dev->frame_seq == 0U))
  {
    return 0U;
  }

  return ((now_ms - dev->last_rx_ms) <= timeout_ms) ? 1U : 0U;
}

/**
  * @brief  最近一条非数据帧原文(配置回显/查询应答), 调试用
  */
const char *AT8236_GetLastResponse(const AT8236_Object *dev)
{
  if (dev == NULL)
  {
    return "";
  }
  return dev->last_response;
}
