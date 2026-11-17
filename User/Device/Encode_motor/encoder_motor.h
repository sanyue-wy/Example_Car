/**
 * @file    encoder_motor.h
 * @brief   编码器电机设备驱动(接口层)
 *          方向 GPIO + PWM 调速 + 编码器测速 + 速度环 PID
 * @version 1.0
 * @date    2026-11-17
 */

#ifndef __ENCODER_MOTOR_H__
#define __ENCODER_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "tim.h"
#include "drv_pwm.h"
#include "drv_encoder.h"
#include "pid.h"

/* Exported types ------------------------------------------------------------*/

/**
  ******************************************************************************
  * 建模思路与 drv_pwm/drv_encoder 一致: 一个编码器电机 = 一个
  * EncoderMotor_Object 对象。数据(硬件绑定/编码器/PWM/PID/运行量) + 行为
  * (开环调速/闭环调速/周期任务) 封装在一起, 业务层不出现 CCR、CNT、GPIO。
  *
  * 分层:
  *   EncoderMotor(设备层) -> drv_pwm(输出) + drv_encoder(反馈) + pid(算法)
  *   编码器对象与 PWM 通道对象内嵌在电机对象中, 由驱动层注册表登记 ——
  *   电机对象必须静态分配, 不能在栈上创建或运行中搬移。
  *
  * 硬件约束(无法封装掉, 迁移时注意):
  * 1. CubeMX 需配置: 编码器定时器 Encoder_Interface(本工程 TIM2 TI12 四倍频),
  *    PWM 通道(本工程 TIM1_CH1, 20kHz), 两个方向引脚为 GPIO 推挽输出
  *    (H 桥 IN1/IN2, 本工程 PB0/PB1);
  * 2. 方向真值表: 正转 IN1=1/IN2=0, 反转 IN1=0/IN2=1, 停止两脚都 0(滑行);
  *    两脚都置 1 是 H 桥刹车, 本驱动不使用;
  * 3. 同一定时器所有通道共用一个 PWM 频率: 不要把本电机的 PWM 定时器
  *    同时交给舵机驱动(50Hz), 否则后 Init 的一方会因频率冲突失败;
  * 4. 转速是"每个 Task 周期采一次编码器再差分"得到的, Task 周期必须与
  *    cfg.control_period_s 一致(建议 5~10ms); 两次采样之间的计数变化需
  *    小于半个编码器量程(见 drv_encoder 说明);
  * 5. counts_per_rev 要含四倍频与减速比; 本工程 JCA25-370-103K 为
  *    11 线 × 4 倍频 × 103 减速比 = 4532 计数/输出轴圈;
  * 6. 闭环时若实测转速超过 2 × rps_limit, Task 直接停车并记 fault_count ——
  *    counts_per_rev 填错或编码器取反配错导致的飞车会被这层拦住;
  * 7. 该电机编码器标称供电 3.3~5V, 但 TIM2 的 PA0/PA1 不是 5V 容忍引脚,
  *    必须把编码器 VCC 接 3.3V; 线色对应见 encoder_motor_debug.c 文件头。
  ******************************************************************************
  */

/**
  * @brief  控制模式
  */
typedef enum
{
  ENCODER_MOTOR_MODE_OPEN  = 0, /* 开环: 直接执行有符号占空比命令 */
  ENCODER_MOTOR_MODE_SPEED = 1, /* 闭环: 目标转速 -> 速度环 PID -> 占空比 */
} EncoderMotor_Mode_e;

/**
  * @brief  电机硬件与参数配置(Init 时整份拷贝进对象)
  */
typedef struct
{
  const char        *name;             /* 调试用名字, 如 "MOTOR1" */
  TIM_HandleTypeDef *enc_htim;         /* 编码器定时器句柄(Encoder_Interface) */
  TIM_HandleTypeDef *pwm_htim;         /* PWM 定时器句柄 */
  uint32_t           pwm_channel;      /* PWM 通道: TIM_CHANNEL_1 ~ TIM_CHANNEL_4 */
  GPIO_TypeDef      *dir_port;         /* 方向引脚端口 */
  uint16_t           dir_pin_fwd;      /* 正转方向引脚(H 桥 IN1) */
  uint16_t           dir_pin_rev;      /* 反转方向引脚(H 桥 IN2) */
  uint32_t           pwm_freq_hz;      /* PWM 频率(Hz), 建议 10~20kHz */
  uint32_t           counts_per_rev;   /* 输出轴一圈计数值(含倍频/减速比) */
  uint8_t            enc_invert;       /* 编码器方向取反: 0 正常, 1 取反 */
  uint8_t            dir_invert;       /* 方向引脚对调: 0 正常, 1 对调 */
  float              rps_limit;        /* 闭环目标限幅与飞车保护基准(转/秒), > 0 */
  float              speed_kp;         /* 速度环 Kp(‰占空比 / rps) */
  float              speed_ki;         /* 速度环 Ki */
  float              speed_kd;         /* 速度环 Kd */
  float              max_duty;         /* 占空比限幅(‰), 0 < max_duty <= 1000 */
  float              control_period_s; /* Task 调用周期(秒), 建议 0.005~0.01 */
} EncoderMotor_Config_t;

/**
  * @brief  运行统计(任务上下文写入, 业务/调试层读取)
  */
typedef struct
{
  volatile uint32_t task_count;  /* Task 调用次数 */
  volatile uint32_t stop_count;  /* Stop 次数 */
  volatile uint32_t sat_count;   /* 闭环输出达到限幅的次数 */
  volatile uint32_t fault_count; /* 超速保护触发次数 */
} EncoderMotor_Stats_t;

typedef struct EncoderMotor_Object EncoderMotor_Object;

/**
  * @brief  编码器电机对象
  */
struct EncoderMotor_Object
{
  EncoderMotor_Config_t cfg;     /* 配置拷贝 */
  ENCODER_Object        encoder; /* 编码器驱动对象(内嵌) */
  PWM_ChannelObject     pwm;     /* PWM 驱动对象(内嵌) */
  PID_Controller_t      pid;     /* 速度环 */

  EncoderMotor_Mode_e mode;      /* 当前控制模式 */
  float               target_rps;/* 闭环目标(已限幅) */
  float               rps;       /* 最近一次 Task 测得的输出轴转速(转/秒) */
  float               tps;       /* 最近一次 Task 测得的计数率(计数/秒) */
  int16_t             command_duty; /* 开环命令(‰, 带符号) */
  int16_t             duty;      /* 当前实际输出(‰, 带符号) */
  uint32_t            dt_ms;     /* Task 周期(毫秒), 由 control_period_s 导出 */
  uint8_t             initialized;
  uint8_t             last_enc_ret; /* Init 时编码器驱动返回码(诊断用) */
  uint8_t             last_pwm_ret; /* Init 时 PWM 驱动返回码(诊断用) */
  EncoderMotor_Stats_t stats;
};

/* Exported functions ---------------------------------------------------------*/

uint8_t  EncoderMotor_Init(EncoderMotor_Object *motor, const EncoderMotor_Config_t *cfg);
void     EncoderMotor_Task(EncoderMotor_Object *motor);
void     EncoderMotor_SetMode(EncoderMotor_Object *motor, EncoderMotor_Mode_e mode);
void     EncoderMotor_SetSpeedRps(EncoderMotor_Object *motor, float rps);
void     EncoderMotor_SetDuty(EncoderMotor_Object *motor, int16_t duty_permille);
void     EncoderMotor_Stop(EncoderMotor_Object *motor);
float    EncoderMotor_GetSpeedRps(const EncoderMotor_Object *motor);
float    EncoderMotor_GetTargetRps(const EncoderMotor_Object *motor);
int16_t  EncoderMotor_GetDuty(const EncoderMotor_Object *motor);
int32_t  EncoderMotor_GetCount(const EncoderMotor_Object *motor);
EncoderMotor_Mode_e EncoderMotor_GetMode(const EncoderMotor_Object *motor);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_MOTOR_H__ */