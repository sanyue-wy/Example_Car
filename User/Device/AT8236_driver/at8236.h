/**
 * @file    at8236.h
 * @brief   四路 AT8236 编码器电机驱动板设备驱动(接口层)
 *          串口 ASCII 协议: 4 路速度/PWM 控制 + 编码器/速度/电压上报
 * @version 1.0
 * @date    2026-11-17
 */

#ifndef __AT8236_H__
#define __AT8236_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "usart.h"
#include "drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

#define AT8236_MOTOR_NUM          4U     /* 驱动板 4 路电机 M1~M4 */
#define AT8236_SPEED_LIMIT        1000   /* 速度指令范围 ±1000 (mm/s) */
#define AT8236_PWM_LIMIT          3600   /* PWM 指令范围 ±3600 (‰) */
#define AT8236_RX_BUFFER_SIZE     128U   /* DMA 接收缓冲, 按一帧/多帧到达预留 */
#define AT8236_FRAME_MAX          64U    /* 单帧内容上限($ 与 # 之间) */
#define AT8236_TX_BUFFER_SIZE     48U    /* 最长指令 $spd:-1000,-1000,-1000,-1000# */

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 建模思路与 encoder_motor 一致: 一块驱动板 = 一个 AT8236_Object 对象。
  * 数据(串口对象/接收解析/4 路反馈/统计) + 行为(配置/控制/查询) 封装在一起,
  * 业务层不出现 ASCII 指令、逗号分隔和 $/# 帧格式。
  *
  * 分层:
  *   AT8236(设备层) -> drv_uart(传输层)
  *   串口端口对象内嵌在驱动板对象中, 由 drv_uart 注册表登记 ——
  *   驱动板对象必须静态分配, 不能在栈上创建或在运行中搬移。
  *
  * 协议要点(详见《4 路电机驱动板控制指令》):
  *   - 波特率 115200, 8N1, 无硬件流控;
  *   - 配置指令(写入驱动板 flash, 断电保存, 成功回 "<指令>OK"):
  *       $mtype:x#      电机类型 1:520  2:310  3:TT(带编码器)  4:TT(不带编码器)
  *       $deadzone:xxxx# PWM 死区 0~3600
  *       $mline:xx#      编码器每转相位线数(本电机 11)
  *       $mphase:xx#     减速比(本电机 103)
  *       $wdiameter:xx#  轮径 mm, 只影响速度(mm/s)换算, 不影响编码器计数
  *       $mpid:p,i,d#    PID, 每次写入驱动板会重启并停转电机
  *       $upload:a,b,c#  上报开关: a=总编码器  b=10ms 实时编码器  c=速度
  *       $read_flash#   查询 flash 变量      $read_vol#   查询电池电压
  *       $flash_reset#  恢复出厂(驱动板重启)
  *   - 运行指令(电机动, 驱动板不返回任何数据):
  *       $spd:m1,m2,m3,m4#   闭环速度 mm/s, ±1000, 给 0 时 PID 仍锁住轮子
  *       $pwm:m1,m2,m3,m4#   开环 PWM ‰,   ±3600, 给 0 时滑行
  *   - 上报帧(仅带编码器电机, 由 $upload 打开):
  *       $MAll:m1,m2,m3,m4#  轮子累计编码器计数
  *       $MTEP:m1,m2,m3,m4#  10ms 实时编码器计数
  *       $MSPD:m1,m2,m3,m4#  轮子速度 mm/s
  *       $Battery:7.40V#     $read_vol# 的应答
  *   - 驱动板丝印 M1~M4 只代表板上通道号 0~3; 与小车轮位的实际对应关系由
  *     实车接线决定, 本车见 app_config.h / chassis_map.h 的 CHASSIS_WHEEL_MAP
  *     (不要假设 M1 一定是左前)。
  *
  * 硬件/工程约束(无法封装掉, 迁移时注意):
  * 1. 本工程用 USART2(PA2=TX -> 驱动板 RX2, PA3=RX <- 驱动板 TX2), GND 共地;
  *    驱动板逻辑电平 3.3V, 不要接 5V 串口电平;
  * 2. drv_uart 的接收是 DMA + 空闲线中断, 因此 CubeMX 必须给 USART2 同时开
  *    RX/TX DMA(STM32F103: DMA1 Channel6/Channel7)并使能对应 NVIC 中断,
  *    否则 UART_Port_Init 返回 3, 本驱动 Init 失败(见 uart_init_ret);
  * 3. 配置指令带 flash 写, 两条之间至少间隔 ~100ms; PID 指令会让驱动板重启,
  *    正常使用不要反复下发, 保持驱动板出厂 PID 即可;
  * 4. $spd 是"保持型"指令: 发一次后驱动板持续按该速度闭环运行, 不会超时停车;
  *    需要停车必须显式发 $spd:0,0,0,0# 或 $pwm:0,0,0,0#。本驱动的
  *    command_timeout_ms 只在本对象 Task 仍被调用时才起作用;
  * 5. 本工程电机 JCA25-370-103K: 11 线霍尔编码器、减速比 103、12V 空载
  *    85rpm(约 0.3m/s @67mm 轮), 因此速度指令实际可用范围约 ±300mm/s,
  *    再大只会饱和; 编码器分辨率 11×4×103 = 4532 计数/输出轴圈;
  * 6. 电机类型选 2(310): 按手册, 编码器 A/B 分别接驱动板 A/B 口时选 310;
  * 若现场 A/B 接反导致闭环飞车, 先调换电机线序, 不要用改 mtype 来掩盖;
  * 7. 驱动板单路持续 4A, 电机 12V 供电与主控 3.3V 必须共地。
  ******************************************************************************
  */

/**
  * @brief  驱动板支持的电机类型
  */
typedef enum
{
  AT8236_MOTOR_520        = 1, /* 520 电机 */
  AT8236_MOTOR_310        = 2, /* 310 电机(编码器 A/B 正常接线) */
  AT8236_MOTOR_TT_ENCODER = 3, /* TT 电机, 带编码器 */
  AT8236_MOTOR_TT         = 4  /* TT 电机, 不带编码器 */
} AT8236_MotorType_e;

/**
  * @brief  函数返回码
  */
typedef enum
{
  AT8236_OK         = 0, /* 成功(已交给 DMA 发送) */
  AT8236_ERR_PARAM  = 1, /* 参数错误/未初始化 */
  AT8236_ERR_UART   = 2, /* 串口端口初始化失败(见 uart_init_ret) */
  AT8236_ERR_BUSY   = 3, /* 上一帧 DMA 尚未发完, 本帧未发送 */
  AT8236_ERR_RANGE  = 4  /* 配置值超出协议范围 */
} AT8236_Result_e;

/**
  * @brief  驱动板配置(Init 时整份拷贝进对象)
  */
typedef struct
{
  const char         *name;              /* 调试用名字, 如 "AT8236" */
  UART_HandleTypeDef *huart;             /* 底层串口句柄(本工程 &huart2) */

  AT8236_MotorType_e  motor_type;        /* 电机类型, 默认 310 */
  uint16_t            encoder_lines;     /* 编码器每转相位线数, JCA25-370 为 11 */
  uint16_t            gear_ratio;        /* 减速比, JCA25-370-103K 为 103 */
  float               wheel_diameter_mm; /* 轮径 mm, 默认 67.0 */
  uint16_t            deadzone;          /* PWM 死区 0~3600, 默认 1600 */

  float               pid_kp;            /* 闭环 PID, 仅 ConfigPID 时下发 */
  float               pid_ki;
  float               pid_kd;

  uint32_t            command_timeout_ms;/* 速度/PWM 指令看门狗(ms), 0=关闭 */
} AT8236_Config_t;

/**
  * @brief  运行统计(中断上下文写入, 任务上下文读取)
  */
typedef struct
{
  volatile uint32_t rx_frames;    /* 完整帧数($...#) */
  volatile uint32_t total_frames; /* $MAll 帧数 */
  volatile uint32_t delta_frames; /* $MTEP 帧数 */
  volatile uint32_t speed_frames; /* $MSPD 帧数 */
  volatile uint32_t battery_frames;/* $Battery 帧数 */
  volatile uint32_t ack_frames;   /* 配置指令回显(指令+OK)帧数 */
  volatile uint32_t parse_errors; /* 字段数/格式不符的帧数 */
  volatile uint32_t overflow;     /* 超长帧/缓冲溢出丢弃次数 */
  volatile uint32_t tx_ok;        /* 成功交给 DMA 的指令数 */
  volatile uint32_t tx_busy;      /* DMA 忙而跳过的指令数 */
  volatile uint32_t sat_count;    /* 速度/PWM 被限幅的次数 */
  volatile uint32_t timeout_stops;/* 指令看门狗触发停车次数 */
} AT8236_Stats_t;

typedef struct AT8236_Object AT8236_Object;

/**
  * @brief  AT8236 驱动板对象
  */
struct AT8236_Object
{
  AT8236_Config_t cfg;                 /* 配置拷贝 */
  UART_PortObject uart;                /* 内嵌串口端口对象 */
  uint8_t         rx_buffer[AT8236_RX_BUFFER_SIZE]; /* DMA 接收缓冲 */

  /* 接收组帧(串口中断上下文) */
  char     frame[AT8236_FRAME_MAX];    /* $ 与 # 之间的内容 */
  uint16_t frame_len;
  uint8_t  receiving;
  uint8_t  overflow;

  char     tx_buffer[AT8236_TX_BUFFER_SIZE]; /* DMA 发送缓冲, 发送期间不可覆盖 */

  /* 驱动板上报(中断写, 任务读) */
  volatile int32_t encoder_total[AT8236_MOTOR_NUM]; /* 累计计数 */
  volatile int32_t encoder_delta[AT8236_MOTOR_NUM]; /* 10ms 计数 */
  volatile float   speed_mmps[AT8236_MOTOR_NUM];    /* 速度 mm/s */
  volatile float   battery_v;                       /* 电池电压 V */
  char             last_response[AT8236_FRAME_MAX]; /* 最近一条非数据帧原文 */
  volatile uint32_t last_rx_ms;                     /* 最近一帧时间(HAL_GetTick) */
  volatile uint32_t frame_seq;                      /* 帧序号, 用于在线判断 */

  /* 上报开关缓存 */
  uint8_t upload_total;
  uint8_t upload_delta;
  uint8_t upload_speed;

  /* 运行指令看门狗 */
  uint8_t  command_active;            /* 1=速度/PWM 指令正在保持 */
  uint8_t  command_is_pwm;            /* 1=PWM 指令, 0=速度指令 */
  uint32_t last_command_ms;

  uint8_t initialized;                /* Init 成功标志 */
  uint8_t uart_init_ret;              /* UART_Port_Init 原始返回码 */
  AT8236_Stats_t stats;
};

/* Exported functions ---------------------------------------------------------*/

/**
  * 初始化
  */
uint8_t AT8236_Init(AT8236_Object *dev, const AT8236_Config_t *cfg);
void    AT8236_DefaultConfig(AT8236_Config_t *cfg, UART_HandleTypeDef *huart, const char *name);
void    AT8236_Task(AT8236_Object *dev, uint32_t now_ms);

/**
  * 配置(写入驱动板 flash; 连续调用时每条间隔 >=100ms)
  */
uint8_t AT8236_ConfigMotorType(AT8236_Object *dev, AT8236_MotorType_e type);
uint8_t AT8236_ConfigEncoderLines(AT8236_Object *dev, uint16_t lines);
uint8_t AT8236_ConfigGearRatio(AT8236_Object *dev, uint16_t ratio);
uint8_t AT8236_ConfigWheelDiameter(AT8236_Object *dev, float diameter_mm);
uint8_t AT8236_ConfigDeadzone(AT8236_Object *dev, uint16_t deadzone);
uint8_t AT8236_ConfigPID(AT8236_Object *dev, float kp, float ki, float kd);
uint8_t AT8236_SetUpload(AT8236_Object *dev, uint8_t encoder_total,
                         uint8_t encoder_delta, uint8_t speed);
uint8_t AT8236_QueryBattery(AT8236_Object *dev);
uint8_t AT8236_QueryFlash(AT8236_Object *dev);
uint8_t AT8236_ResetFactory(AT8236_Object *dev);

/**
  * 运行控制(驱动板不返回应答; 保持上次指令直到下一条)
  */
uint8_t AT8236_SetSpeed(AT8236_Object *dev, int16_t m1, int16_t m2, int16_t m3, int16_t m4);
uint8_t AT8236_SetSpeedArray(AT8236_Object *dev, const int16_t speed[AT8236_MOTOR_NUM]);
uint8_t AT8236_SetPwm(AT8236_Object *dev, int16_t m1, int16_t m2, int16_t m3, int16_t m4);
uint8_t AT8236_SetPwmArray(AT8236_Object *dev, const int16_t pwm[AT8236_MOTOR_NUM]);
uint8_t AT8236_Stop(AT8236_Object *dev);   /* $spd:0#   PID 锁住轮子 */
uint8_t AT8236_Coast(AT8236_Object *dev);  /* $pwm:0#   滑行 */

/**
  * 反馈
  */
int32_t AT8236_GetEncoderTotal(const AT8236_Object *dev, uint8_t index);
int32_t AT8236_GetEncoderDelta(const AT8236_Object *dev, uint8_t index);
float   AT8236_GetSpeedMmps(const AT8236_Object *dev, uint8_t index);
float   AT8236_GetBatteryVoltage(const AT8236_Object *dev);
uint8_t AT8236_IsOnline(const AT8236_Object *dev, uint32_t now_ms, uint32_t timeout_ms);
const char *AT8236_GetLastResponse(const AT8236_Object *dev);

#ifdef __cplusplus
}
#endif

#endif /* __AT8236_H__ */
