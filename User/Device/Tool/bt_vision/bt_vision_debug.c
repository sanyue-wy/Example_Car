/**
 * @file    bt_vision_debug.c
 * @brief   bt_vision 蓝牙调试上位机(实现层)
 *          基于 btdbg 协议库, 串口绑定 USART2(PA2/PA3, HC-05), 115200 8N1
 * @note    接收: 单字节中断 -> btdbg_rx_byte (无锁环形缓冲, 中断安全)
 *          发送: 主循环内阻塞发送, 返回即发完, 库可安全复用其发送缓冲
 *          参数: params_car.c 的 25 项表, 上位机 SET 后立即生效
 */

#include "debug_config.h"

#if bt_vision_debug == 1

#include "bt_vision_debug.h"
#include "btdbg.h"
#include "params_car.h"
#include "main.h"
#include "usart.h"
#include "drv_uart.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/
#define BT_VISION_CALIB_SAMPLES    500u     /* 零偏校准采样点数(静止) */
#define BT_VISION_CALIB_TIMEOUT_MS 3000u    /* 校准超时: 无陀螺仪数据时上报失败 */
#define BT_VISION_TX_TIMEOUT_MS    20u      /* 阻塞发送超时, 最长 64B 帧约 5.6ms */
#define BT_VISION_STATS_PERIOD_MS  5000u    /* USART1 统计打印周期 */

/* Private variables ---------------------------------------------------------*/

/* btdbg_init 保存的是 cfg 指针, 必须为静态生命周期 */
static btdbg_config_t s_cfg;
static uint8_t        s_rx_byte;            /* 单字节中断接收暂存 */

/* 遥测缓存: 业务层 Update, Run 里按上报频率统一发送 */
static float   s_motor[5];                  /* 目标位置/实际位置/目标速度/实际速度/输出 */
static float   s_gyro[3];                   /* 原始角速度 gx/gy/gz (°/s, 未减零偏) */
static float   s_car[3];                    /* 速度/距离/航向 */
static uint8_t s_car_valid;                 /* 收到过 CAR 遥测才发送该通道 */

/* 陀螺仪零偏校准 */
static volatile uint8_t s_calib_req;
static uint8_t  s_calib_seen;               /* 校准期间是否收到过陀螺仪数据 */
static uint32_t s_calib_tick;
static uint16_t s_calib_n;
static float    s_calib_sum[3];

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  协议库发送回调(主循环上下文)
  * @note   阻塞发送, 返回时数据已交到 USART2 移位寄存器并发送完毕,
  *         库可安全复用 g_tx_enc; 发送失败只丢这一帧, 不影响后续
  */
static void BtVision_Write(const uint8_t *data, uint16_t len)
{
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)data, len, BT_VISION_TX_TIMEOUT_MS);
}

/**
  * @brief  毫秒时基
  */
static uint32_t BtVision_Millis(void)
{
    return HAL_GetTick();
}

/**
  * @brief  上位机发起零偏校准: 只置标志, 采样在主循环 Run 里做
  */
static uint8_t BtVision_StartCalib(void)
{
    s_calib_n        = 0;
    s_calib_seen     = 0;
    s_calib_sum[0]   = 0.0f;
    s_calib_sum[1]   = 0.0f;
    s_calib_sum[2]   = 0.0f;
    s_calib_tick     = HAL_GetTick();
    s_calib_req      = 1;
    return BTDBG_OK;
}

/* Exported functions ---------------------------------------------------------*/

/**
  * @brief  初始化: 绑定 USART2, 填参数表/UID, 启动单字节中断接收
  * @note   必须在 MX_USART2_UART_Init() 之后调用; 发送/接收互斥由协议层保证
  */
void BtVision_Debug_Init(void)
{
    s_car_valid      = 0;
    s_calib_req      = 0;
    memset(s_motor, 0, sizeof(s_motor));
    memset(s_gyro, 0, sizeof(s_gyro));
    memset(s_car, 0, sizeof(s_car));

    s_cfg.write       = BtVision_Write;
    s_cfg.millis      = BtVision_Millis;
    s_cfg.params      = g_car_params;
    s_cfg.param_count = g_car_param_count;
    memcpy(s_cfg.uid, (const void *)(uintptr_t)UID_BASE, sizeof(s_cfg.uid));
    s_cfg.fw_ver      = 1u;

    s_cfg.on_param_changed = NULL;          /* SET 成功后无需额外动作 */
    /* TODO: 持久化参数写 Flash 后把下面两个回调补上(建议双页交替 + CRC).
     * 现在为 NULL: 上位机 SAVE 会收到 0x03(不支持持久化), 不会假装成功. */
    s_cfg.save_param      = NULL;
    s_cfg.save_pset_ver   = NULL;
    s_cfg.pset_ver        = 0u;

    s_cfg.start_gyro_calib = BtVision_StartCalib;
    s_cfg.report_hz_var    = &g_report_hz;  /* SET_RATE 与 REPORT_HZ 共用变量 */

    btdbg_init(&s_cfg);

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);

    UART_Port_Printf(&uart1_port,
                     "[bt_vision] init ok | USART2 115200 (HC-05) | params=%u | report=%uHz\r\n",
                     (unsigned)g_car_param_count, (unsigned)btdbg_report_hz());
}

/**
  * @brief  HAL 接收完成回调(USART2 中断上下文)
  * @note   每收到 1 字节喂给协议库后立刻重新挂接收;
  *         btdbg_rx_byte 是单生产者无锁入环形缓冲, 可在中断里调
  * @warning 本工程只允许这一处定义 HAL_UART_RxCpltCallback
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    btdbg_rx_byte(s_rx_byte);
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
}

/**
  * @brief  主循环周期调用
  * @param  now HAL_GetTick() 时间戳
  * @note   顺序: 解析命令 -> 校准采样 -> 遥测上报 -> 接收兜底 -> 统计打印
  */
void BtVision_Debug_Run(uint32_t now)
{
    static uint32_t s_last_print = 0;

    btdbg_poll();

    /* ---- 零偏校准: 静止采 500 点取均值, 写回持久化参数并上报完成 ---- */
    if (s_calib_req)
    {
        s_calib_sum[0] += s_gyro[0];
        s_calib_sum[1] += s_gyro[1];
        s_calib_sum[2] += s_gyro[2];

        if (++s_calib_n >= BT_VISION_CALIB_SAMPLES)
        {
            g_gyro_bias_x = s_calib_sum[0] / (float)BT_VISION_CALIB_SAMPLES;
            g_gyro_bias_y = s_calib_sum[1] / (float)BT_VISION_CALIB_SAMPLES;
            g_gyro_bias_z = s_calib_sum[2] / (float)BT_VISION_CALIB_SAMPLES;
            btdbg_gyro_calib_done(BTDBG_OK, g_gyro_bias_x, g_gyro_bias_y, g_gyro_bias_z);
            s_calib_req = 0;
            UART_Port_Printf(&uart1_port,
                             "[bt_vision] gyro calib ok | bias=(%.3f, %.3f, %.3f)\r\n",
                             (double)g_gyro_bias_x, (double)g_gyro_bias_y, (double)g_gyro_bias_z);
        }
        else if ((s_calib_seen == 0U) &&
                 ((uint32_t)(now - s_calib_tick) >= BT_VISION_CALIB_TIMEOUT_MS))
        {
            /* 业务层一直没喂陀螺仪数据(UpdateGyro 未调用), 不能让上位机干等;
             * 有数据时只按采样点数结束, 不因主循环慢而误判超时 */
            btdbg_gyro_calib_done(BTDBG_E_BUSY, 0.0f, 0.0f, 0.0f);
            s_calib_req = 0;
            UART_Port_Printf(&uart1_port, "[bt_vision] [FAIL] gyro calib timeout (no data)\r\n");
        }
    }

    /* ---- 遥测上报: 3 条独立通道帧, 上位机按通道号分别画图 ---- */
    if (btdbg_telemetry_due())
    {
        btdbg_send_telemetry(BTDBG_CH_MOTOR, s_motor, 5u);
        btdbg_send_telemetry(BTDBG_CH_GYRO,  s_gyro,  3u);
        if (s_car_valid)
        {
            btdbg_send_telemetry(BTDBG_CH_CAR, s_car, 3u);
        }
    }

    /* ---- 接收兜底: 噪声/溢出会让 HAL 停掉接收, 这里重新挂载 ---- */
    if (huart2.RxState == HAL_UART_STATE_READY)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1u);
    }

    /* ---- 诊断: USART1 每 5s 打印一次协议统计 ---- */
    if ((uint32_t)(now - s_last_print) >= BT_VISION_STATS_PERIOD_MS)
    {
        const btdbg_stats_t *st = btdbg_stats();
        s_last_print = now;
        UART_Port_Printf(&uart1_port,
                         "[bt_vision] ok | state=%u rate=%uHz | frames=%lu crc=%lu cobs=%lu "
                         "ovf=%lu drop=%lu | tick_ms=%lu\r\n",
                         (unsigned)btdbg_get_state(), (unsigned)btdbg_report_hz(),
                         (unsigned long)st->rx_frames, (unsigned long)st->crc_errors,
                         (unsigned long)st->cobs_errors, (unsigned long)st->overflows,
                         (unsigned long)st->ring_drops, (unsigned long)HAL_GetTick());
    }
}

/**
  * @brief  喂入电机遥测(主循环上下文, 建议每控制周期一次)
  * @param  tgt_pos 目标位置, pos 实际位置, tgt_spd 目标速度, spd 实际速度, out 输出
  */
void BtVision_Debug_UpdateMotor(float tgt_pos, float pos, float tgt_spd, float spd, float out)
{
    s_motor[0] = tgt_pos;
    s_motor[1] = pos;
    s_motor[2] = tgt_spd;
    s_motor[3] = spd;
    s_motor[4] = out;
}

/**
  * @brief  喂入陀螺仪原始角速度(°/s, 未减零偏; 校准取这里的均值)
  */
void BtVision_Debug_UpdateGyro(float gx, float gy, float gz)
{
    s_gyro[0] = gx;
    s_gyro[1] = gy;
    s_gyro[2] = gz;
    if (s_calib_req)
    {
        s_calib_seen = 1;
    }
}

/**
  * @brief  喂入整车遥测: 速度(m/s) / 距离(m) / 航向(°)
  */
void BtVision_Debug_UpdateCar(float spd, float dist, float yaw)
{
    s_car[0]   = spd;
    s_car[1]   = dist;
    s_car[2]   = yaw;
    s_car_valid = 1;
}

/**
  * @brief  切换设备状态(上位机状态灯 + 参数写权限判断)
  * @param  state  BTDBG_ST_IDLE/RUNNING/CALIB/FAULT...
  * @param  reason 故障原因码, 正常切换填 0
  */
void BtVision_Debug_SetState(btdbg_state_t state, uint8_t reason)
{
    btdbg_set_state(state, reason);
}

#endif /* bt_vision_debug == 1 */