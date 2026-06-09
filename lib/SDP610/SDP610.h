#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct i2c_inst i2c_inst_t;

#define SDP610_DEFAULT_ADDR 0x40
#define SDP610_DEFAULT_SCALE_FACTOR 240.0f
#define SDP610_DEFAULT_TIMEOUT 10
#define SDP610_START_CONTINUOUS_READ 0xF1

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        i2c_inst_t *i2c;
        uint8_t addr;
        bool use_sensor;
        bool initialized;
        bool connected;
        uint8_t timeout;
        float scale_factor;
        float airspeed;
    } sdp610_t;

    void sdp610_init_struct(sdp610_t *dev, i2c_inst_t *i2c, uint8_t addr, bool use_sensor, uint8_t timeout);
    bool sdp610_init(sdp610_t *dev);
    bool sdp610_read_raw(sdp610_t *dev, int16_t *raw);
    bool sdp610_read_diff_pressure(sdp610_t *dev, float *pressure_pa);
    bool sdp610_read_airspeed(sdp610_t *dev, float temp_c, float magic_number, float *airspeed_mps);

#ifdef __cplusplus
}
#endif
