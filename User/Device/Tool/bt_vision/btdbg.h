/*
 * btdbg - 蓝牙调试上位机 下位机协议库 (L1 + L2, 协议 0x0100)
 * C99, 无 malloc, 无操作系统依赖, 单实例.
 *
 * 接入三步:
 *   1. 定义参数表 btdbg_param_t[] 并填 btdbg_config_t, 调 btdbg_init().
 *   2. 串口接收中断里每收到 1 字节调 btdbg_rx_byte(b).
 *   3. 主循环里调 btdbg_poll(); 需要发遥测时:
 *        if (btdbg_telemetry_due()) btdbg_send_telemetry(ch, vals, n);
 *
 * 线程模型: btdbg_rx_byte 可在中断中调用 (单生产者无锁环形缓冲);
 * 其余函数只能在同一个上下文 (主循环或同一任务) 中调用.
 */
#ifndef BTDBG_H
#define BTDBG_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BTDBG_PROTO_VER      0x0100u

/* ---- 可调配置 (编译前可用 -D 覆盖) ---- */
#ifndef BTDBG_RX_RING_SIZE
#define BTDBG_RX_RING_SIZE   512u      /* 必须是 2 的幂 */
#endif
#define BTDBG_MAX_FRAME      256u      /* L1: 编码前最大帧长 (含 CRC) */
#define BTDBG_MAX_ID_LEN     16u
#define BTDBG_MAX_TELEM_VALS 13u       /* 遥测帧 <= 64 字节 */

/* ---- 设备状态 (R30) ---- */
typedef enum {
    BTDBG_ST_IDLE     = 1,  /* 已连接空闲 */
    BTDBG_ST_SELFTEST = 2,  /* 自检中 */
    BTDBG_ST_RUNNING  = 3,  /* 运行中 */
    BTDBG_ST_CALIB    = 4,  /* 标定中 */
    BTDBG_ST_SLEEP    = 5,  /* 待机休眠 */
    BTDBG_ST_FAULT    = 6   /* 故障 */
} btdbg_state_t;

/* 允许写入的状态掩码 */
#define BTDBG_W(st)          (1u << (st))
#define BTDBG_W_IDLE         BTDBG_W(BTDBG_ST_IDLE)
#define BTDBG_W_IDLE_RUN     (BTDBG_W(BTDBG_ST_IDLE) | BTDBG_W(BTDBG_ST_RUNNING))

/* ---- 结果码 ---- */
enum {
    BTDBG_OK            = 0x00,
    BTDBG_E_RANGE       = 0x01,  /* 值越界 */
    BTDBG_E_UNSUPPORTED = 0x02,  /* 参数不支持 */
    BTDBG_E_NOPERSIST   = 0x03,  /* 不支持持久化 */
    BTDBG_E_STATE       = 0x04,  /* 状态不允许 */
    BTDBG_E_TYPE        = 0x05,  /* 类型不匹配 */
    BTDBG_IN_PROGRESS   = 0x06,  /* 进行中 */
    BTDBG_E_FORMAT      = 0x07,  /* 帧格式错误 */
    BTDBG_E_UNKNOWN_CMD = 0x08,  /* 未知命令 */
    BTDBG_E_BUSY        = 0x09   /* 设备忙 */
};

/* ---- 数据类型 (与上位机 type 码一致) ---- */
typedef enum {
    BTDBG_I8 = 1, BTDBG_I16 = 2, BTDBG_I32 = 3,
    BTDBG_U8 = 4, BTDBG_U16 = 5, BTDBG_U32 = 6,
    BTDBG_F32 = 7
} btdbg_type_t;

/* 持久化属性 */
#define BTDBG_RAM      0u
#define BTDBG_PERSIST  1u
#define BTDBG_RO       2u

typedef struct {
    const char *id;        /* ASCII 短名, [A-Za-z0-9_], <=16 */
    void       *ptr;       /* 指向你的变量 */
    uint8_t     type;      /* btdbg_type_t */
    uint8_t     persist;   /* BTDBG_RAM / BTDBG_PERSIST / BTDBG_RO */
    uint8_t     wmask;     /* 允许写入的状态, 如 BTDBG_W_IDLE_RUN */
    float       min, max;  /* 闭区间 */
} btdbg_param_t;

/* 参数表声明宏 */
#define BTDBG_PARAM(id_, var_, type_, persist_, wmask_, min_, max_) \
    { (id_), (void *)&(var_), (type_), (persist_), (wmask_), (float)(min_), (float)(max_) }

/* 遥测通道 */
#define BTDBG_CH_MOTOR  0x01  /* 目标位置, 实际位置, 目标速度, 实际速度, 输出 */
#define BTDBG_CH_GYRO   0x02  /* gx, gy, gz */
#define BTDBG_CH_CAR    0x03  /* 速度, 距离, 航向 */

typedef struct {
    /* ---- 必填 ---- */
    void     (*write)(const uint8_t *data, uint16_t len); /* 发送; 返回前须已拷走数据或发完 */
    uint32_t (*millis)(void);                             /* 毫秒时基 */
    const btdbg_param_t *params;
    uint16_t  param_count;
    uint8_t   uid[12];                                    /* 芯片 96 位 UID 原样 */
    uint16_t  fw_ver;

    /* ---- 可选 (NULL 即不支持) ---- */
    void    (*on_param_changed)(const btdbg_param_t *p);  /* SET 成功后回调 */
    uint8_t (*save_param)(const btdbg_param_t *p);        /* 写 Flash, 返回结果码 */
    uint8_t (*start_gyro_calib)(void);                    /* 返回 BTDBG_OK 表示已开始 */
    void    (*save_pset_ver)(uint32_t ver);               /* 持久化参数集版本 */
    uint32_t  pset_ver;                                   /* 上电时从 Flash 读出的版本, 0=未写过 */
    uint8_t  *report_hz_var;  /* 可选: 指向参数表里的 REPORT_HZ 变量, 使 SET_RATE 与参数写入共用同一变量;
                                 NULL 则库内部保存. 初值不在 1..100 时置 50 */
} btdbg_config_t;

typedef struct {
    uint32_t rx_frames, crc_errors, cobs_errors, overflows, ring_drops;
} btdbg_stats_t;

void     btdbg_init(const btdbg_config_t *cfg);
void     btdbg_rx_byte(uint8_t b);            /* 可在中断中调用 */
void     btdbg_poll(void);                    /* 主循环调用 */

void          btdbg_set_state(btdbg_state_t st, uint8_t reason);  /* 状态变化即上报 0x40 */
btdbg_state_t btdbg_get_state(void);

int      btdbg_telemetry_due(void);           /* 到上报时刻返回 1 */
void     btdbg_send_telemetry(uint8_t ch, const float *vals, uint8_t n);

/* 零偏校准结束时调用: result=BTDBG_OK 成功. 自动恢复校准前状态 */
void     btdbg_gyro_calib_done(uint8_t result, float bx, float by, float bz);

uint8_t  btdbg_report_hz(void);
const btdbg_stats_t *btdbg_stats(void);

/* 底层工具 (测试用, 也可单独使用) */
uint16_t btdbg_crc16(const uint8_t *d, size_t n);                 /* CRC-16/CCITT-FALSE */
size_t   btdbg_cobs_encode(const uint8_t *in, size_t n, uint8_t *out);
size_t   btdbg_cobs_decode(const uint8_t *in, size_t n, uint8_t *out); /* 0 = 非法 */

#ifdef __cplusplus
}
#endif
#endif /* BTDBG_H */
