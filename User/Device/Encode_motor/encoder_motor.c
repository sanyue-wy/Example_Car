/**
 * @file    encoder_motor.c
 * @brief   编码器电机设备驱动(实现层)
 * @version 1.0
 * @date    2026-11-17
 */

/* Includes ------------------------------------------------------------------*/
#include "encoder_motor.h"

/* Private macros ------------------------------------------------------------*/

#define ENCODER_MOTOR_DUTY_MAX       1000      /* 占空比绝对上限(‰) */
#define ENCODER_MOTOR_FAULT_FACTOR   2.0f      /* 闭环飞车保护: 2 × rps_limit */

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  ******************************************************************************
  * 与 drv_pwm / drv_encoder 同构, 并做了如下取舍:
  * 1. 一个电机封装为一个 EncoderMotor_Object, 内部组合驱动层对象, 不重复实现
  *    PWM/编码器细节;
  * 2. 方向用两个 GPIO(H 桥 IN1/IN2)+ 单路 PWM 的常见接法; 负占空比通过
  *    交换 IN1/IN2 实现, PWM 本身始终送正占空比;
  * 3. 开环/闭环共用一条输出通道: 闭环用 pid 的输出(‰)直接当占空比, 限幅
  *    max_duty; 切模式时先把输出清零, 避免带着旧占空比进入新环;
  * 4. 闭环运行加一道飞车保护: |rps| > 2 × rps_limit 时立即停车并记
  *    fault_count —— 编码器配置错误时不会一路满占空比转下去。
  ******************************************************************************
  */

/* Private function declarations ---------------------------------------------*/

static void EncoderMotor_ApplyDuty(EncoderMotor_Object *motor, int16_t duty);

/* Private functions ----------------------------------------------------------*/

/**
  * @brief  把有符号占空比落到方向引脚与 PWM 比较值上
  * @param  motor 电机对象
  * @param  duty  有符号占空比(‰), 正转 > 0, 反转 < 0, 0 滑行
  * @note   先定方向再给占空比, 避免用旧方向叠加新占空比
  */
static void EncoderMotor_ApplyDuty(EncoderMotor_Object *motor, int16_t duty)
{
  uint16_t pin_fwd;
  uint16_t pin_rev;
  uint16_t magnitude;

  if (motor == NULL || motor->cfg.dir_port == NULL)
  {
    return;
  }

  pin_fwd = motor->cfg.dir_pin_fwd;
  pin_rev = motor->cfg.dir_pin_rev;
  if (motor->cfg.dir_invert != 0U)
  {
    uint16_t tmp = pin_fwd;
    pin_fwd = pin_rev;
    pin_rev = tmp;
  }

  /* 双重限幅: 绝对 1000‰ + 配置的 max_duty */
  if (duty > ENCODER_MOTOR_DUTY_MAX)
  {
    duty = ENCODER_MOTOR_DUTY_MAX;
  }
  if (duty < -ENCODER_MOTOR_DUTY_MAX)
  {
    duty = -ENCODER_MOTOR_DUTY_MAX;
  }
  if ((float)duty > motor->cfg.max_duty)
  {
    duty = (int16_t)motor->cfg.max_duty;
  }
  if ((float)duty < -motor->cfg.max_duty)
  {
    duty = (int16_t)(-motor->cfg.max_duty);
  }

  if (duty > 0)
  {
    HAL_GPIO_WritePin(motor->cfg.dir_port, pin_fwd, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->cfg.dir_port, pin_rev, GPIO_PIN_RESET);
    magnitude = (uint16_t)duty;
  }
  else if (duty < 0)
  {
    HAL_GPIO_WritePin(motor->cfg.dir_port, pin_fwd, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->cfg.dir_port, pin_rev, GPIO_PIN_SET);
    magnitude = (uint16_t)(-duty);
  }
  else
  {
    /* 两脚都低 = 滑行停车 */
    HAL_GPIO_WritePin(motor->cfg.dir_port, pin_fwd, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->cfg.dir_port, pin_rev, GPIO_PIN_RESET);
    magnitude = 0U;
  }

  (void)PWM_Channel_SetDuty(&motor->pwm, magnitude);
  motor->duty = duty;
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  初始化编码器电机: 绑定编码器/PWM/方向引脚并配置速度环
  * @param  motor 电机对象实例, 由调用方静态分配
  * @param  cfg   硬件绑定与参数配置
  * @retval 0 成功; 1 参数错误; 2 编码器驱动接入失败; 3 PWM 驱动接入失败
  * @note   上电默认: 开环模式、占空比 0、方向引脚全低, 不会自己转;
  *         重复 Init 同一对象视为原位重配, 计数/统计从零开始。
  */
uint8_t EncoderMotor_Init(EncoderMotor_Object *motor, const EncoderMotor_Config_t *cfg)
{
  uint8_t ret;

  if (motor == NULL || cfg == NULL || cfg->name == NULL ||
      cfg->enc_htim == NULL || cfg->pwm_htim == NULL || cfg->dir_port == NULL ||
      cfg->dir_pin_fwd == 0U || cfg->dir_pin_rev == 0U ||
      cfg->pwm_freq_hz == 0U || cfg->counts_per_rev == 0U ||
      cfg->enc_invert > 1U || cfg->dir_invert > 1U ||
      !(cfg->rps_limit > 0.0f) ||
      !(cfg->max_duty > 0.0f) || cfg->max_duty > (float)ENCODER_MOTOR_DUTY_MAX ||
      !(cfg->control_period_s >= 0.001f) || cfg->control_period_s > 0.1f ||
      (cfg->pwm_channel != TIM_CHANNEL_1 && cfg->pwm_channel != TIM_CHANNEL_2 &&
       cfg->pwm_channel != TIM_CHANNEL_3 && cfg->pwm_channel != TIM_CHANNEL_4))
  {
    return 1;
  }

  motor->initialized = 0U;
  motor->cfg = *cfg;
  motor->dt_ms = (uint32_t)(cfg->control_period_s * 1000.0f + 0.5f);
  if (motor->dt_ms == 0U)
  {
    return 1;
  }

  /* 编码器: 反馈通道 */
  ret = ENCODER_Init(&motor->encoder, cfg->enc_htim, cfg->name,
                     cfg->counts_per_rev, cfg->enc_invert);
  motor->last_enc_ret = ret;
  if (ret != 0U)
  {
    return 2;
  }

  /* PWM: 速度输出通道, 初始占空比 0 */
  ret = PWM_Channel_Init(&motor->pwm, cfg->pwm_htim, cfg->pwm_channel, cfg->name,
                         cfg->pwm_freq_hz, 0U);
  motor->last_pwm_ret = ret;
  if (ret != 0U)
  {
    return 3;
  }

  /* 速度环: 输出单位就是占空比千分比 */
  PID_Init(&motor->pid, cfg->speed_kp, cfg->speed_ki, cfg->speed_kd,
           cfg->max_duty, cfg->max_duty, cfg->control_period_s);
  motor->pid.Improve = PID_IMPROVE_INTEGRAL_LIMIT;

  /* 运行态归零 */
  motor->mode = ENCODER_MOTOR_MODE_OPEN;
  motor->target_rps = 0.0f;
  motor->rps = 0.0f;
  motor->tps = 0.0f;
  motor->command_duty = 0;
  motor->duty = 0;
  motor->stats.task_count = 0U;
  motor->stats.stop_count = 0U;
  motor->stats.sat_count = 0U;
  motor->stats.fault_count = 0U;

  motor->initialized = 1U;
  EncoderMotor_ApplyDuty(motor, 0); /* 上电即滑行 */

  return 0;
}

/**
  * @brief  电机周期任务: 采编码器 -> 算转速 -> 闭环 PID/开环命令 -> 输出
  * @param  motor 电机对象
  * @note   必须以 cfg.control_period_s 的周期调用(建议 5~10ms);
  *         闭环时若 |rps| 超过 2 × rps_limit 会立即停车并记 fault_count。
  */
void EncoderMotor_Task(EncoderMotor_Object *motor)
{
  int32_t delta;
  int16_t out;

  if (motor == NULL || motor->initialized == 0U)
  {
    return;
  }

  ENCODER_Update(&motor->encoder);
  delta = ENCODER_GetDelta(&motor->encoder);
  motor->tps = (float)delta / motor->cfg.control_period_s;
  motor->rps = ENCODER_GetSpeedRps(&motor->encoder, motor->dt_ms);
  motor->stats.task_count++;

  if (motor->mode == ENCODER_MOTOR_MODE_SPEED)
  {
    float raw;

    /* 飞车保护: 只在闭环生效, 开环由调用方自己负责 */
    if (FLOAT_ABS(motor->rps) > (motor->cfg.rps_limit * ENCODER_MOTOR_FAULT_FACTOR))
    {
      motor->stats.fault_count++;
      EncoderMotor_Stop(motor);
      return;
    }

    raw = PID_Calculate(&motor->pid, motor->rps, motor->target_rps);
    if (FLOAT_ABS(raw) >= motor->cfg.max_duty)
    {
      motor->stats.sat_count++;
    }
    out = (int16_t)((raw >= 0.0f) ? (raw + 0.5f) : (raw - 0.5f));
  }
  else
  {
    out = motor->command_duty;
  }

  EncoderMotor_ApplyDuty(motor, out);
}

/**
  * @brief  切换控制模式
  * @param  motor 电机对象
  * @param  mode  ENCODER_MOTOR_MODE_OPEN / ENCODER_MOTOR_MODE_SPEED
  * @note   切换时输出清零并清 PID 历史, 目标/命令也复位
  */
void EncoderMotor_SetMode(EncoderMotor_Object *motor, EncoderMotor_Mode_e mode)
{
  if (motor == NULL || motor->initialized == 0U ||
      (mode != ENCODER_MOTOR_MODE_OPEN && mode != ENCODER_MOTOR_MODE_SPEED))
  {
    return;
  }
  if (motor->mode == mode)
  {
    return;
  }

  motor->mode = mode;
  motor->target_rps = 0.0f;
  motor->command_duty = 0;
  PID_Clear(&motor->pid);
  EncoderMotor_ApplyDuty(motor, 0);
}

/**
  * @brief  设置闭环目标转速(转/秒, 带符号)
  * @param  motor 电机对象
  * @param  rps   目标转速, 自动限幅到 ±rps_limit; 调用会切到闭环模式
  * @note   闭环需持续调用 Task 才会输出; 未 Init 时调用无效
  */
void EncoderMotor_SetSpeedRps(EncoderMotor_Object *motor, float rps)
{
  if (motor == NULL || motor->initialized == 0U)
  {
    return;
  }

  if (motor->mode != ENCODER_MOTOR_MODE_SPEED)
  {
    EncoderMotor_SetMode(motor, ENCODER_MOTOR_MODE_SPEED);
  }

  if (rps > motor->cfg.rps_limit)
  {
    rps = motor->cfg.rps_limit;
  }
  else if (rps < -motor->cfg.rps_limit)
  {
    rps = -motor->cfg.rps_limit;
  }
  motor->target_rps = rps;
}

/**
  * @brief  设置开环占空比(‰, 带符号)
  * @param  motor         电机对象
  * @param  duty_permille 正转 > 0, 反转 < 0, 范围 ±max_duty, 越界自动限幅
  * @note   调用会切到开环模式, 并立即更新输出(不必等下一次 Task)
  */
void EncoderMotor_SetDuty(EncoderMotor_Object *motor, int16_t duty_permille)
{
  if (motor == NULL || motor->initialized == 0U)
  {
    return;
  }

  if (motor->mode != ENCODER_MOTOR_MODE_OPEN)
  {
    EncoderMotor_SetMode(motor, ENCODER_MOTOR_MODE_OPEN);
  }

  if ((float)duty_permille > motor->cfg.max_duty)
  {
    duty_permille = (int16_t)motor->cfg.max_duty;
  }
  else if ((float)duty_permille < -motor->cfg.max_duty)
  {
    duty_permille = (int16_t)(-motor->cfg.max_duty);
  }

  motor->command_duty = duty_permille;
  EncoderMotor_ApplyDuty(motor, duty_permille);
}

/**
  * @brief  停车: 目标清零、开环、输出 0(滑行)
  * @param  motor 电机对象
  */
void EncoderMotor_Stop(EncoderMotor_Object *motor)
{
  if (motor == NULL || motor->initialized == 0U)
  {
    return;
  }

  motor->mode = ENCODER_MOTOR_MODE_OPEN;
  motor->target_rps = 0.0f;
  motor->command_duty = 0;
  PID_Clear(&motor->pid);
  EncoderMotor_ApplyDuty(motor, 0);
  motor->stats.stop_count++;
}

/**
  * @brief  读取最近一次 Task 测得的转速
  * @param  motor 电机对象
  * @retval 输出轴转速(转/秒, 带符号); 未 Init/未 Task 时返回 0
  */
float EncoderMotor_GetSpeedRps(const EncoderMotor_Object *motor)
{
  if (motor == NULL || motor->initialized == 0U)
  {
    return 0.0f;
  }
  return motor->rps;
}

/**
  * @brief  读取闭环目标转速
  * @param  motor 电机对象
  * @retval 目标转速(转/秒, 已限幅)
  */
float EncoderMotor_GetTargetRps(const EncoderMotor_Object *motor)
{
  if (motor == NULL || motor->initialized == 0U)
  {
    return 0.0f;
  }
  return motor->target_rps;
}

/**
  * @brief  读取当前输出占空比
  * @param  motor 电机对象
  * @retval 有符号占空比(‰)
  */
int16_t EncoderMotor_GetDuty(const EncoderMotor_Object *motor)
{
  if (motor == NULL || motor->initialized == 0U)
  {
    return 0;
  }
  return motor->duty;
}

/**
  * @brief  读取累计计数
  * @param  motor 电机对象
  * @retval 自 Init/Reset 以来的有符号计数值
  */
int32_t EncoderMotor_GetCount(const EncoderMotor_Object *motor)
{
  if (motor == NULL || motor->initialized == 0U)
  {
    return 0;
  }
  return ENCODER_GetCount(&motor->encoder);
}

/**
  * @brief  读取当前模式
  * @param  motor 电机对象
  * @retval 当前控制模式; 未 Init 时按开环返回
  */
EncoderMotor_Mode_e EncoderMotor_GetMode(const EncoderMotor_Object *motor)
{
  if (motor == NULL || motor->initialized == 0U)
  {
    return ENCODER_MOTOR_MODE_OPEN;
  }
  return motor->mode;
}