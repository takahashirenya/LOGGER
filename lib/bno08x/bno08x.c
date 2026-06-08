#include "bno08x.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#define BNO_MAX_PACKET_SIZE 512
#define BNO_PACKET_HEADER_LEN 4

#define BNO_FEATURE_ENABLE_TIMEOUT_MS 2000
#define BNO_PACKET_TIMEOUT_MS 2000
#define BNO_INIT_TIMEOUT_MS 2000

#define BNO_Q_POINT_14_SCALAR (1.0f / 16384.0f)
#define BNO_Q_POINT_12_SCALAR (1.0f / 4096.0f)
#define BNO_Q_POINT_9_SCALAR  (1.0f / 512.0f)
#define BNO_Q_POINT_8_SCALAR  (1.0f / 256.0f)
#define BNO_Q_POINT_4_SCALAR  (1.0f / 16.0f)

typedef struct {
    uint8_t channel;
    uint8_t seq;
    uint16_t packet_len;
    uint16_t data_len;
} bno_packet_header_t;

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    bool initialized;

    uint8_t seq_send[6];
    uint8_t cmd_seq;

    uint8_t rxbuf[BNO_MAX_PACKET_SIZE];
    uint8_t txbuf[BNO_MAX_PACKET_SIZE];

    bool feature_enabled[256];

    bool has_accel;
    bool has_lin_accel;
    bool has_gyro;
    bool has_quat;

    bno08x_vec3_t accel;
    bno08x_vec3_t lin_accel;
    bno08x_vec3_t gyro;
    bno08x_quat_t quat;
} bno08x_ctx_t;

static bno08x_ctx_t g_bno;

static inline int16_t le_i16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint16_t le_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t le_u32(const uint8_t *p) {
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline void write_le_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void write_le_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint16_t report_length_from_id(uint8_t report_id) {
    switch (report_id) {
        case BNO_REPORT_ACCELEROMETER: return 10;
        case BNO_REPORT_GYROSCOPE: return 10;
        case BNO_REPORT_MAGNETOMETER: return 10;
        case BNO_REPORT_LINEAR_ACCELERATION: return 10;
        case BNO_REPORT_ROTATION_VECTOR: return 14;
        case BNO_REPORT_GRAVITY: return 10;
        case BNO_REPORT_GAME_ROTATION_VECTOR: return 12;
        case BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR: return 14;
        case BNO_COMMAND_RESPONSE: return 16;
        case BNO_SHTP_REPORT_ID_RESPONSE: return 16;
        case BNO_GET_FEATURE_RESPONSE: return 17;
        case 0xFA: return 5;
        case 0xFB: return 5;
        default: return 0;
    }
}

static bool read_header(bno_packet_header_t *hdr) {
    uint8_t raw[4];
    int rc = i2c_read_blocking(g_bno.i2c, g_bno.addr, raw, 4, false);
    if (rc != 4) {
        return false;
    }

    uint16_t packet_len = le_u16(raw) & 0x7FFF;
    hdr->packet_len = packet_len;
    hdr->channel = raw[2];
    hdr->seq = raw[3];
    hdr->data_len = (packet_len >= 4) ? (uint16_t)(packet_len - 4) : 0;
    return true;
}

static bool data_ready(void) {
    bno_packet_header_t hdr;
    if (!read_header(&hdr)) {
        return false;
    }

    if (hdr.channel > 5) {
        return false;
    }

    if (hdr.packet_len == 0 || hdr.packet_len == 0x7FFF || hdr.packet_len == 0xFFFF) {
        return false;
    }

    return hdr.data_len > 0;
}

static bool read_packet(uint8_t *channel, uint8_t *payload, uint16_t *payload_len) {
    bno_packet_header_t hdr;
    if (!read_header(&hdr)) {
        return false;
    }

    if (hdr.packet_len == 0 || hdr.packet_len > BNO_MAX_PACKET_SIZE) {
        return false;
    }

    int rc = i2c_read_blocking(g_bno.i2c, g_bno.addr, g_bno.rxbuf, hdr.packet_len, false);
    if (rc != (int)hdr.packet_len) {
        return false;
    }

    *channel = g_bno.rxbuf[2];
    *payload_len = hdr.data_len;
    memcpy(payload, &g_bno.rxbuf[4], hdr.data_len);
    return true;
}

static bool send_packet(uint8_t channel, const uint8_t *data, uint16_t data_len) {
    if (data_len + 4 > BNO_MAX_PACKET_SIZE) {
        return false;
    }

    uint16_t packet_len = (uint16_t)(data_len + 4);
    write_le_u16(&g_bno.txbuf[0], packet_len);
    g_bno.txbuf[2] = channel;
    g_bno.txbuf[3] = g_bno.seq_send[channel];
    memcpy(&g_bno.txbuf[4], data, data_len);

    int rc = i2c_write_blocking(g_bno.i2c, g_bno.addr, g_bno.txbuf, packet_len, false);
    if (rc != (int)packet_len) {
        return false;
    }

    g_bno.seq_send[channel]++;
    return true;
}

static void parse_vec3_report(const uint8_t *r, float scalar, bno08x_vec3_t *out) {
    out->x = (float)le_i16(&r[4]) * scalar;
    out->y = (float)le_i16(&r[6]) * scalar;
    out->z = (float)le_i16(&r[8]) * scalar;
}

static void parse_quat_report(const uint8_t *r, float scalar, bno08x_quat_t *out) {
    float x = (float)le_i16(&r[4]) * scalar;
    float y = (float)le_i16(&r[6]) * scalar;
    float z = (float)le_i16(&r[8]) * scalar;
    float w = (float)le_i16(&r[10]) * scalar;

    out->w = w;
    out->x = x;
    out->y = y;
    out->z = z;
}

static void handle_control_report(uint8_t report_id, const uint8_t *r, uint16_t len) {
    (void)len;

    if (report_id == BNO_GET_FEATURE_RESPONSE) {
        uint8_t feature_id = r[1];
        g_bno.feature_enabled[feature_id] = true;
    }
}

static void handle_sensor_report(uint8_t report_id, const uint8_t *r, uint16_t len) {
    (void)len;

    switch (report_id) {
        case BNO_REPORT_ACCELEROMETER:
            parse_vec3_report(r, BNO_Q_POINT_8_SCALAR, &g_bno.accel);
            g_bno.has_accel = true;
            break;

        case BNO_REPORT_LINEAR_ACCELERATION:
            parse_vec3_report(r, BNO_Q_POINT_8_SCALAR, &g_bno.lin_accel);
            g_bno.has_lin_accel = true;
            break;

        case BNO_REPORT_GYROSCOPE:
            parse_vec3_report(r, BNO_Q_POINT_9_SCALAR, &g_bno.gyro);
            g_bno.has_gyro = true;
            break;

        case BNO_REPORT_ROTATION_VECTOR:
            parse_quat_report(r, BNO_Q_POINT_14_SCALAR, &g_bno.quat);
            g_bno.has_quat = true;
            break;

        default:
            break;
    }
}

static void process_payload(const uint8_t *payload, uint16_t payload_len) {
    uint16_t offset = 0;

    while (offset < payload_len) {
        uint8_t report_id = payload[offset];
        uint16_t rlen = report_length_from_id(report_id);

        if (rlen == 0 || offset + rlen > payload_len) {
            return;
        }

        const uint8_t *r = &payload[offset];

        if (report_id >= 0xF0) {
            handle_control_report(report_id, r, rlen);
        } else {
            handle_sensor_report(report_id, r, rlen);
        }

        offset += rlen;
    }
}

static bool wait_for_product_id(void) {
    uint8_t req[2] = { BNO_SHTP_REPORT_ID_REQUEST, 0x00 };

    if (!send_packet(BNO_CHANNEL_CONTROL, req, sizeof(req))) {
        return false;
    }

    absolute_time_t until = make_timeout_time_ms(BNO_INIT_TIMEOUT_MS);
    while (!time_reached(until)) {
        if (!data_ready()) {
            sleep_ms(2);
            continue;
        }

        uint8_t channel = 0;
        uint16_t len = 0;
        uint8_t payload[BNO_MAX_PACKET_SIZE];

        if (!read_packet(&channel, payload, &len)) {
            sleep_ms(2);
            continue;
        }

        if (channel == BNO_CHANNEL_CONTROL && len >= 16 && payload[0] == BNO_SHTP_REPORT_ID_RESPONSE) {
            (void)le_u32(&payload[4]);
            return true;
        }

        process_payload(payload, len);
    }

    return false;
}

static bool soft_reset(void) {
    uint8_t cmd = 1;

    if (!send_packet(BNO_CHANNEL_EXECUTABLE, &cmd, 1)) {
        return false;
    }
    sleep_ms(500);

    if (!send_packet(BNO_CHANNEL_EXECUTABLE, &cmd, 1)) {
        return false;
    }
    sleep_ms(500);

    for (int i = 0; i < 3; i++) {
        if (data_ready()) {
            uint8_t channel = 0;
            uint16_t len = 0;
            uint8_t payload[BNO_MAX_PACKET_SIZE];
            if (read_packet(&channel, payload, &len)) {
                process_payload(payload, len);
            }
        }
        sleep_ms(20);
    }

    return true;
}

bool bno08x_init(i2c_inst_t *i2c, uint8_t addr) {
    memset(&g_bno, 0, sizeof(g_bno));
    g_bno.i2c = i2c;

    if (addr == 0) {
        uint8_t addrs[2] = { BNO08X_ADDR_0, BNO08X_ADDR_1 };
        bool found = false;

        for (int i = 0; i < 2; i++) {
            uint8_t dummy[4];
            int rc = i2c_read_blocking(i2c, addrs[i], dummy, 4, false);
            if (rc == 4) {
                g_bno.addr = addrs[i];
                found = true;
                break;
            }
        }

        if (!found) {
            return false;
        }
    } else {
        g_bno.addr = addr;
    }

    for (int tries = 0; tries < 3; tries++) {
        if (!soft_reset()) {
            sleep_ms(100);
            continue;
        }

        if (wait_for_product_id()) {
            g_bno.initialized = true;
            return true;
        }
    }

    return false;
}

uint8_t bno08x_get_address(void) {
    return g_bno.addr;
}

bool bno08x_enable_feature(uint8_t feature_id, uint32_t report_interval_us) {
    if (!g_bno.initialized) {
        return false;
    }

    uint8_t pkt[17];
    memset(pkt, 0, sizeof(pkt));

    pkt[0] = BNO_SET_FEATURE_COMMAND;
    pkt[1] = feature_id;
    write_le_u32(&pkt[5], report_interval_us);
    write_le_u32(&pkt[9], 0);
    write_le_u32(&pkt[13], 0);

    if (!send_packet(BNO_CHANNEL_CONTROL, pkt, sizeof(pkt))) {
        return false;
    }

    absolute_time_t until = make_timeout_time_ms(BNO_FEATURE_ENABLE_TIMEOUT_MS);
    while (!time_reached(until)) {
        bno08x_update();
        if (g_bno.feature_enabled[feature_id]) {
            return true;
        }
        sleep_ms(2);
    }

    return false;
}

bool bno08x_update(void) {
    if (!g_bno.initialized) {
        return false;
    }

    bool any = false;

    for (int i = 0; i < 10; i++) {
        if (!data_ready()) {
            break;
        }

        uint8_t channel = 0;
        uint16_t len = 0;
        uint8_t payload[BNO_MAX_PACKET_SIZE];

        if (!read_packet(&channel, payload, &len)) {
            break;
        }

        if (channel == BNO_CHANNEL_CONTROL ||
            channel == BNO_CHANNEL_INPUT_REPORTS ||
            channel == BNO_CHANNEL_WAKE_INPUT_REPORTS ||
            channel == BNO_CHANNEL_GYRO) {
            process_payload(payload, len);
            any = true;
        }
    }

    return any;
}

bool bno08x_read_accel(float *x, float *y, float *z) {
    bno08x_update();
    if (!g_bno.has_accel) return false;
    if (x) *x = g_bno.accel.x;
    if (y) *y = g_bno.accel.y;
    if (z) *z = g_bno.accel.z;
    return true;
}

bool bno08x_read_linear_accel(float *x, float *y, float *z) {
    bno08x_update();
    if (!g_bno.has_lin_accel) return false;
    if (x) *x = g_bno.lin_accel.x;
    if (y) *y = g_bno.lin_accel.y;
    if (z) *z = g_bno.lin_accel.z;
    return true;
}

bool bno08x_read_gyro(float *x, float *y, float *z) {
    bno08x_update();
    if (!g_bno.has_gyro) return false;
    if (x) *x = g_bno.gyro.x;
    if (y) *y = g_bno.gyro.y;
    if (z) *z = g_bno.gyro.z;
    return true;
}

bool bno08x_read_quaternion(float *w, float *x, float *y, float *z) {
    bno08x_update();
    if (!g_bno.has_quat) return false;
    if (w) *w = g_bno.quat.w;
    if (x) *x = g_bno.quat.x;
    if (y) *y = g_bno.quat.y;
    if (z) *z = g_bno.quat.z;
    return true;
}

bool bno08x_tare(uint8_t axis_mask, uint8_t outputs) {
    if (!g_bno.initialized) {
        return false;
    }

    uint8_t pkt[12];
    memset(pkt, 0, sizeof(pkt));

    pkt[0] = BNO_COMMAND_REQUEST;
    pkt[1] = g_bno.cmd_seq++;
    pkt[2] = BNO_ME_TARE;
    pkt[3] = 0x00;
    pkt[4] = axis_mask;
    pkt[5] = outputs;

    return send_packet(BNO_CHANNEL_CONTROL, pkt, sizeof(pkt));
}

bno08x_quat_t bno08x_quat_conjugate(bno08x_quat_t q) {
    bno08x_quat_t r = { q.w, -q.x, -q.y, -q.z };
    return r;
}

bno08x_quat_t bno08x_quat_multiply(bno08x_quat_t a, bno08x_quat_t b) {
    bno08x_quat_t r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

bno08x_quat_t bno08x_apply_reference(bno08x_quat_t reference, bno08x_quat_t current) {
    bno08x_quat_t ref_conj = bno08x_quat_conjugate(reference);
    return bno08x_quat_multiply(ref_conj, current);
}

bno08x_euler_t bno08x_quat_to_euler_deg(bno08x_quat_t q) {
    bno08x_euler_t e;

    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.roll_deg = atan2f(sinr_cosp, cosr_cosp) * 180.0f / (float)M_PI;

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f) {
        e.pitch_deg = copysignf(90.0f, sinp);
    } else {
        e.pitch_deg = asinf(sinp) * 180.0f / (float)M_PI;
    }

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.yaw_deg = atan2f(siny_cosp, cosy_cosp) * 180.0f / (float)M_PI;

    return e;
}