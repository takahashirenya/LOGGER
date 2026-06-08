#ifndef BNO08X_H
#define BNO08X_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BNO08X_ADDR_0 0x4A
#define BNO08X_ADDR_1 0x4B

// Channels
#define BNO_CHANNEL_COMMAND               0x00
#define BNO_CHANNEL_EXECUTABLE            0x01
#define BNO_CHANNEL_CONTROL               0x02
#define BNO_CHANNEL_INPUT_REPORTS         0x03
#define BNO_CHANNEL_WAKE_INPUT_REPORTS    0x04
#define BNO_CHANNEL_GYRO                  0x05

// Control report IDs
#define BNO_COMMAND_RESPONSE              0xF1
#define BNO_COMMAND_REQUEST               0xF2
#define BNO_SHTP_REPORT_ID_RESPONSE       0xF8
#define BNO_SHTP_REPORT_ID_REQUEST        0xF9
#define BNO_GET_FEATURE_RESPONSE          0xFC
#define BNO_SET_FEATURE_COMMAND           0xFD

// Motion engine commands
#define BNO_ME_TARE                       0x03

// Sensor report IDs
#define BNO_REPORT_ACCELEROMETER          0x01
#define BNO_REPORT_GYROSCOPE              0x02
#define BNO_REPORT_MAGNETOMETER           0x03
#define BNO_REPORT_LINEAR_ACCELERATION    0x04
#define BNO_REPORT_ROTATION_VECTOR        0x05
#define BNO_REPORT_GRAVITY                0x06
#define BNO_REPORT_GAME_ROTATION_VECTOR   0x08
#define BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR 0x09

#define BNO_DEFAULT_REPORT_INTERVAL_US    50000u

typedef struct {
    float x;
    float y;
    float z;
} bno08x_vec3_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} bno08x_quat_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} bno08x_euler_t;

bool bno08x_init(i2c_inst_t *i2c, uint8_t addr);
uint8_t bno08x_get_address(void);

bool bno08x_enable_feature(uint8_t feature_id, uint32_t report_interval_us);
bool bno08x_update(void);

bool bno08x_read_accel(float *x, float *y, float *z);
bool bno08x_read_linear_accel(float *x, float *y, float *z);
bool bno08x_read_gyro(float *x, float *y, float *z);
bool bno08x_read_quaternion(float *w, float *x, float *y, float *z);

bool bno08x_tare(uint8_t axis_mask, uint8_t outputs);

bno08x_quat_t bno08x_quat_conjugate(bno08x_quat_t q);
bno08x_quat_t bno08x_quat_multiply(bno08x_quat_t a, bno08x_quat_t b);
bno08x_quat_t bno08x_apply_reference(bno08x_quat_t reference, bno08x_quat_t current);
bno08x_euler_t bno08x_quat_to_euler_deg(bno08x_quat_t q);

#ifdef __cplusplus
}
#endif

#endif