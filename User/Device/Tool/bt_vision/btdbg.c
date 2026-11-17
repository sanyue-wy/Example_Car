/* btdbg.c - 见 btdbg.h */
#include "btdbg.h"
#include <string.h>

/* ---------- 命令码 ---------- */
#define CMD_HELLO      0x01
#define CMD_PING       0x02
#define CMD_GET        0x10
#define CMD_SET        0x11
#define CMD_LIST       0x12
#define CMD_SAVE       0x13
#define CMD_SET_RATE   0x20
#define CMD_PSET_VER   0x21
#define CMD_GYRO_CALIB 0x30
#define EVT_STATE      0x40
#define EVT_TELEM      0x41
#define EVT_HEARTBEAT  0x42
#define EVT_CALIB_DONE 0x43

#define DEDUP_MS       2000u
#define HEARTBEAT_MS   1000u
#define ENC_BUF        (BTDBG_MAX_FRAME + BTDBG_MAX_FRAME / 254u + 2u)

#if (BTDBG_RX_RING_SIZE & (BTDBG_RX_RING_SIZE - 1u)) != 0
#error "BTDBG_RX_RING_SIZE must be a power of 2"
#endif

/* ---------- 状态 ---------- */
typedef struct {
    uint8_t  valid, seq, len;
    uint32_t t;
    uint8_t  resp[40];
} dedup_t;

static const btdbg_config_t *g_cfg;
static volatile uint16_t g_rx_head, g_rx_tail;
static uint8_t  g_ring[BTDBG_RX_RING_SIZE];
static uint8_t  g_acc[ENC_BUF];       /* 正在累积的编码帧 */
static uint16_t g_acc_len;
static uint8_t  g_acc_discard;
static uint8_t  g_dec[BTDBG_MAX_FRAME + 4];
static uint8_t  g_tx_raw[BTDBG_MAX_FRAME];
static uint8_t  g_tx_enc[ENC_BUF + 1];

static btdbg_state_t g_state = BTDBG_ST_IDLE;
static btdbg_state_t g_state_before_calib = BTDBG_ST_IDLE;
static uint8_t  g_rate_store = 50;
static uint8_t *g_rate = &g_rate_store;
static uint32_t g_pset_ver;
static uint32_t g_last_telem, g_last_hb;
static dedup_t  g_dedup_save, g_dedup_calib;
static btdbg_stats_t g_stats;

/* ---------- 工具 ---------- */
uint16_t btdbg_crc16(const uint8_t *d, size_t n)
{
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

size_t btdbg_cobs_encode(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t r = 0, code_pos = 0, w = 1;
    uint8_t code = 1;
    while (r < n) {
        if (in[r] == 0) {
            out[code_pos] = code; code = 1; code_pos = w++;
        } else {
            out[w++] = in[r]; code++;
            if (code == 0xFF) { out[code_pos] = code; code = 1; code_pos = w++; }
        }
        r++;
    }
    out[code_pos] = code;
    return w;
}

size_t btdbg_cobs_decode(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t r = 0, w = 0;
    while (r < n) {
        uint8_t code = in[r++];
        if (code == 0) return 0;
        for (uint8_t i = 1; i < code; i++) {
            if (r >= n || in[r] == 0) return 0;
            out[w++] = in[r++];
        }
        if (code != 0xFF && r < n) out[w++] = 0;
    }
    return w;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void put_u32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t get_u32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static void put_f32(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); put_u32(p, u); }

static uint32_t now(void) { return g_cfg->millis(); }

/* 发送一帧: raw[0..n) 为 cmd+seq+payload */
static void send_raw(const uint8_t *raw, uint16_t n)
{
    if (n + 2u > BTDBG_MAX_FRAME) return;
    if (raw != g_tx_raw) memmove(g_tx_raw, raw, n);
    put_u16(g_tx_raw + n, btdbg_crc16(g_tx_raw, n));
    size_t e = btdbg_cobs_encode(g_tx_raw, n + 2u, g_tx_enc);
    g_tx_enc[e++] = 0x00;
    g_cfg->write(g_tx_enc, (uint16_t)e);
}

/* ---------- 参数 ---------- */
static const btdbg_param_t *find_param(const uint8_t *id, uint8_t len)
{
    for (uint16_t i = 0; i < g_cfg->param_count; i++) {
        const btdbg_param_t *p = &g_cfg->params[i];
        if (strlen(p->id) == len && memcmp(p->id, id, len) == 0) return p;
    }
    return NULL;
}

/* 读变量 -> 4 字节大端 (窄类型扩展) */
static uint32_t param_raw(const btdbg_param_t *p)
{
    switch (p->type) {
    case BTDBG_I8:  return (uint32_t)(int32_t)*(int8_t *)p->ptr;
    case BTDBG_I16: return (uint32_t)(int32_t)*(int16_t *)p->ptr;
    case BTDBG_I32: return (uint32_t)*(int32_t *)p->ptr;
    case BTDBG_U8:  return *(uint8_t *)p->ptr;
    case BTDBG_U16: return *(uint16_t *)p->ptr;
    case BTDBG_U32: return *(uint32_t *)p->ptr;
    default: { uint32_t u; memcpy(&u, p->ptr, 4); return u; }
    }
}

/* 校验并写入; 返回结果码 */
static uint8_t param_write(const btdbg_param_t *p, uint32_t raw)
{
    double v;
    switch (p->type) {
    case BTDBG_I8: case BTDBG_I16: case BTDBG_I32: v = (double)(int32_t)raw; break;
    case BTDBG_U8: case BTDBG_U16: case BTDBG_U32: v = (double)raw; break;
    default: { float f; memcpy(&f, &raw, 4); if (f != f) return BTDBG_E_RANGE; v = f; }
    }
    if (v < (double)p->min || v > (double)p->max) return BTDBG_E_RANGE;
    /* 窄类型额外检查截断 */
    if ((p->type == BTDBG_I8 && (v < -128 || v > 127)) ||
        (p->type == BTDBG_I16 && (v < -32768 || v > 32767)) ||
        (p->type == BTDBG_U8 && v > 255) ||
        (p->type == BTDBG_U16 && v > 65535)) return BTDBG_E_RANGE;

    switch (p->type) {
    case BTDBG_I8:  *(int8_t *)p->ptr   = (int8_t)(int32_t)raw; break;
    case BTDBG_I16: *(int16_t *)p->ptr  = (int16_t)(int32_t)raw; break;
    case BTDBG_I32: *(int32_t *)p->ptr  = (int32_t)raw; break;
    case BTDBG_U8:  *(uint8_t *)p->ptr  = (uint8_t)raw; break;
    case BTDBG_U16: *(uint16_t *)p->ptr = (uint16_t)raw; break;
    case BTDBG_U32: *(uint32_t *)p->ptr = raw; break;
    default: memcpy(p->ptr, &raw, 4); break;
    }
    return BTDBG_OK;
}

static uint8_t state_allows(const btdbg_param_t *p)
{
    return (uint8_t)((p->wmask >> (unsigned)g_state) & 1u);
}

/* 在 out 写入 id_len,id,type,value; 返回长度 */
static uint16_t put_param(uint8_t *out, const btdbg_param_t *p)
{
    uint8_t l = (uint8_t)strlen(p->id);
    out[0] = l;
    memcpy(out + 1, p->id, l);
    out[1 + l] = p->type;
    put_u32(out + 2 + l, param_raw(p));
    return (uint16_t)(6u + l);
}

/* 解析 id; 返回消耗字节数, 0=格式错 */
static uint8_t parse_id(const uint8_t *pl, uint16_t n, const uint8_t **id, uint8_t *len)
{
    if (n < 1) return 0;
    uint8_t l = pl[0];
    if (l == 0 || l > BTDBG_MAX_ID_LEN || n < 1u + l) return 0;
    for (uint8_t i = 0; i < l; i++) {
        uint8_t c = pl[1 + i];
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return 0;
    }
    *id = pl + 1; *len = l;
    return (uint8_t)(1u + l);
}

/* ---------- 应答 ---------- */
static uint8_t g_resp[BTDBG_MAX_FRAME];

static uint16_t resp_begin(uint8_t cmd, uint8_t seq, uint8_t rc)
{
    g_resp[0] = (uint8_t)(cmd | 0x80u); g_resp[1] = seq; g_resp[2] = rc;
    return 3;
}

static void dedup_store(dedup_t *d, uint8_t seq, uint16_t n)
{
    if (n > sizeof d->resp) { d->valid = 0; return; }
    d->valid = 1; d->seq = seq; d->len = (uint8_t)n; d->t = now();
    memcpy(d->resp, g_resp, n);
}

static int dedup_hit(dedup_t *d, uint8_t seq)
{
    if (d->valid && d->seq == seq && (uint32_t)(now() - d->t) < DEDUP_MS) {
        send_raw(d->resp, d->len);   /* 不重复执行, 回缓存 */
        return 1;
    }
    return 0;
}

static void handle(const uint8_t *f, uint16_t n)
{
    uint8_t cmd = f[0], seq = f[1];
    const uint8_t *pl = f + 2;
    uint16_t pn = (uint16_t)(n - 2u), r;
    const uint8_t *id; uint8_t idl, used;
    const btdbg_param_t *p;

    if (cmd & 0x80u) return;                       /* 设备不处理应答帧 */
    if (seq == 0) { r = resp_begin(cmd, seq, BTDBG_E_FORMAT); send_raw(g_resp, r); return; }

    switch (cmd) {
    case CMD_HELLO:
        r = resp_begin(cmd, seq, BTDBG_OK);
        put_u16(g_resp + r, BTDBG_PROTO_VER); r += 2;
        g_resp[r++] = (uint8_t)g_state;
        memcpy(g_resp + r, g_cfg->uid, 12); r += 12;
        put_u16(g_resp + r, g_cfg->fw_ver); r += 2;
        put_u32(g_resp + r, g_pset_ver); r += 4;
        g_resp[r++] = *g_rate;
        break;

    case CMD_PING:
        r = resp_begin(cmd, seq, BTDBG_OK);
        g_resp[r++] = (uint8_t)g_state;
        break;

    case CMD_GET:
        if (!(used = parse_id(pl, pn, &id, &idl)) || used != pn) { r = resp_begin(cmd, seq, BTDBG_E_FORMAT); break; }
        if (!(p = find_param(id, idl))) { r = resp_begin(cmd, seq, BTDBG_E_UNSUPPORTED); break; }
        r = resp_begin(cmd, seq, BTDBG_OK);
        r += put_param(g_resp + r, p);
        break;

    case CMD_SET: {
        uint8_t rc;
        if (!(used = parse_id(pl, pn, &id, &idl)) || pn != used + 5u) { r = resp_begin(cmd, seq, BTDBG_E_FORMAT); break; }
        if (!(p = find_param(id, idl)) || p->persist == BTDBG_RO) { r = resp_begin(cmd, seq, BTDBG_E_UNSUPPORTED); break; }
        if (pl[used] != p->type) { r = resp_begin(cmd, seq, BTDBG_E_TYPE); break; }
        if (!state_allows(p)) { r = resp_begin(cmd, seq, BTDBG_E_STATE); break; }
        rc = param_write(p, get_u32(pl + used + 1));
        if (rc == BTDBG_OK && g_cfg->on_param_changed) g_cfg->on_param_changed(p);
        r = resp_begin(cmd, seq, rc);
        if (rc == BTDBG_OK) r += put_param(g_resp + r, p);   /* 回显实际生效值 */
        break;
    }

    case CMD_LIST: {
        uint16_t idx;
        if (pn != 2) { r = resp_begin(cmd, seq, BTDBG_E_FORMAT); break; }
        idx = get_u16(pl);
        if (idx >= g_cfg->param_count) { r = resp_begin(cmd, seq, BTDBG_E_UNSUPPORTED); break; }
        r = resp_begin(cmd, seq, BTDBG_OK);
        put_u16(g_resp + r, g_cfg->param_count); r += 2;
        put_u16(g_resp + r, idx); r += 2;
        r += put_param(g_resp + r, &g_cfg->params[idx]);
        break;
    }

    case CMD_SAVE: {
        uint8_t rc;
        if (dedup_hit(&g_dedup_save, seq)) return;
        if (!(used = parse_id(pl, pn, &id, &idl)) || used != pn) { r = resp_begin(cmd, seq, BTDBG_E_FORMAT); break; }
        if (!(p = find_param(id, idl))) rc = BTDBG_E_UNSUPPORTED;
        else if (p->persist != BTDBG_PERSIST || !g_cfg->save_param) rc = BTDBG_E_NOPERSIST;
        else if (!state_allows(p)) rc = BTDBG_E_STATE;
        else rc = g_cfg->save_param(p);
        r = resp_begin(cmd, seq, rc);
        if (rc == BTDBG_OK) { memcpy(g_resp + r, pl, used); r += used; }
        dedup_store(&g_dedup_save, seq, r);
        break;
    }

    case CMD_SET_RATE:
        if (pn != 1) { r = resp_begin(cmd, seq, BTDBG_E_FORMAT); break; }
        if (pl[0] < 1 || pl[0] > 100) { r = resp_begin(cmd, seq, BTDBG_E_RANGE); break; }
        *g_rate = pl[0];
        r = resp_begin(cmd, seq, BTDBG_OK);
        g_resp[r++] = *g_rate;
        break;

    case CMD_PSET_VER:
        if (pn != 4) { r = resp_begin(cmd, seq, BTDBG_E_FORMAT); break; }
        g_pset_ver = get_u32(pl);
        if (g_cfg->save_pset_ver) g_cfg->save_pset_ver(g_pset_ver);
        r = resp_begin(cmd, seq, BTDBG_OK);
        put_u32(g_resp + r, g_pset_ver); r += 4;
        break;

    case CMD_GYRO_CALIB: {
        uint8_t rc;
        if (dedup_hit(&g_dedup_calib, seq)) return;
        if (pn != 0) rc = BTDBG_E_FORMAT;
        else if (!g_cfg->start_gyro_calib) rc = BTDBG_E_UNKNOWN_CMD;
        else if (g_state != BTDBG_ST_IDLE) rc = BTDBG_E_STATE;
        else {
            rc = g_cfg->start_gyro_calib();
            if (rc == BTDBG_OK) rc = BTDBG_IN_PROGRESS;
        }
        r = resp_begin(cmd, seq, rc);
        dedup_store(&g_dedup_calib, seq, r);
        send_raw(g_resp, r);
        if (rc == BTDBG_IN_PROGRESS) {             /* 先应答, 再上报状态 */
            g_state_before_calib = g_state;
            btdbg_set_state(BTDBG_ST_CALIB, 0);
        }
        return;
    }

    default:
        r = resp_begin(cmd, seq, BTDBG_E_UNKNOWN_CMD);
        break;
    }
    send_raw(g_resp, r);
}

static void frame_done(void)
{
    size_t n;
    if (g_acc_len == 0) return;
    n = btdbg_cobs_decode(g_acc, g_acc_len, g_dec);
    if (n == 0) { g_stats.cobs_errors++; return; }
    if (n < 4 || n > BTDBG_MAX_FRAME) { g_stats.overflows++; return; }
    if (btdbg_crc16(g_dec, n - 2) != get_u16(g_dec + n - 2)) { g_stats.crc_errors++; return; }
    g_stats.rx_frames++;
    handle(g_dec, (uint16_t)(n - 2));
}

/* ---------- 公共接口 ---------- */
void btdbg_init(const btdbg_config_t *cfg)
{
    g_cfg = cfg;
    g_rx_head = g_rx_tail = 0;
    g_acc_len = 0; g_acc_discard = 0;
    g_state = BTDBG_ST_IDLE;
    g_rate = cfg->report_hz_var ? cfg->report_hz_var : &g_rate_store;
    if (*g_rate < 1 || *g_rate > 100) *g_rate = 50;
    g_pset_ver = cfg->pset_ver;
    g_last_telem = g_last_hb = cfg->millis();
    memset(&g_dedup_save, 0, sizeof g_dedup_save);
    memset(&g_dedup_calib, 0, sizeof g_dedup_calib);
    memset(&g_stats, 0, sizeof g_stats);
}

void btdbg_rx_byte(uint8_t b)
{
    uint16_t h = g_rx_head, nh = (uint16_t)((h + 1u) & (BTDBG_RX_RING_SIZE - 1u));
    if (nh == g_rx_tail) { g_stats.ring_drops++; return; }   /* 满: 丢字节, CRC 会兜住 */
    g_ring[h] = b;
    g_rx_head = nh;
}

void btdbg_poll(void)
{
    while (g_rx_tail != g_rx_head) {
        uint8_t b = g_ring[g_rx_tail];
        g_rx_tail = (uint16_t)((g_rx_tail + 1u) & (BTDBG_RX_RING_SIZE - 1u));
        if (b == 0x00) {
            if (!g_acc_discard) frame_done();
            g_acc_len = 0; g_acc_discard = 0;          /* 重新对齐 */
        } else if (!g_acc_discard) {
            if (g_acc_len >= sizeof g_acc) { g_acc_discard = 1; g_stats.overflows++; }
            else g_acc[g_acc_len++] = b;
        }
    }
    if ((uint32_t)(now() - g_last_hb) >= HEARTBEAT_MS) {
        uint8_t f[3] = { EVT_HEARTBEAT, 0, (uint8_t)g_state };
        g_last_hb = now();
        send_raw(f, 3);
    }
}

void btdbg_set_state(btdbg_state_t st, uint8_t reason)
{
    uint8_t f[4];
    if (st == g_state) return;
    g_state = st;
    f[0] = EVT_STATE; f[1] = 0; f[2] = (uint8_t)st; f[3] = reason;
    send_raw(f, 4);
}

btdbg_state_t btdbg_get_state(void) { return g_state; }
uint8_t btdbg_report_hz(void) { return *g_rate; }
const btdbg_stats_t *btdbg_stats(void) { return &g_stats; }

int btdbg_telemetry_due(void)
{
    uint8_t hz = (*g_rate >= 1 && *g_rate <= 100) ? *g_rate : 50;
    uint32_t period = 1000u / hz, t = now();
    if ((uint32_t)(t - g_last_telem) >= period) {
        /* 追赶但不累积欠账 */
        g_last_telem = ((uint32_t)(t - g_last_telem) >= 2u * period) ? t : g_last_telem + period;
        return 1;
    }
    return 0;
}

void btdbg_send_telemetry(uint8_t ch, const float *vals, uint8_t n)
{
    uint8_t f[8 + 4 * BTDBG_MAX_TELEM_VALS];
    uint16_t k = 0;
    if (n > BTDBG_MAX_TELEM_VALS) n = BTDBG_MAX_TELEM_VALS;
    f[k++] = EVT_TELEM; f[k++] = 0; f[k++] = ch;
    put_u32(f + k, now()); k += 4;
    f[k++] = n;
    for (uint8_t i = 0; i < n; i++) { put_f32(f + k, vals[i]); k += 4; }
    send_raw(f, k);
}

void btdbg_gyro_calib_done(uint8_t result, float bx, float by, float bz)
{
    uint8_t f[15];
    f[0] = EVT_CALIB_DONE; f[1] = 0; f[2] = result;
    put_f32(f + 3, bx); put_f32(f + 7, by); put_f32(f + 11, bz);
    send_raw(f, 15);
    btdbg_set_state(g_state_before_calib, 0);
}
