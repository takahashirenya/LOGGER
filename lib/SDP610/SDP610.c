#include "SDP610.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

void sdp610_init_struct(sdp610_t *dev, i2c_inst_t *i2c, uint8_t addr, bool use_sensor, uint8_t timeout)
{
    dev->i2c = i2c;
    dev->addr = addr;
    dev->use_sensor = use_sensor;
    dev->initialized = false;
    dev->connected = false;
    dev->timeout = timeout;
    dev->scale_factor = SDP610_DEFAULT_SCALE_FACTOR;
    dev->airspeed = 0.0f;
}

bool sdp610_init(sdp610_t *dev)
{
    if (!dev || !dev->i2c)
    {
        return false;
    }

    dev->initialized = false;
    dev->connected = false;

    if (!dev->use_sensor)
    {
        printf("SDP610: sensor disabled\r\n");
        return false;
    }

    uint8_t cmd = SDP610_START_CONTINUOUS_READ;
    for (uint8_t i = 0; i < dev->timeout; i++)
    {
        int rc = i2c_write_blocking(dev->i2c, dev->addr, &cmd, 1, false);
        if (rc == 1)
        {
            dev->initialized = true;
            dev->connected = true;
            return true;
        }

        printf("SDP610: init failed, addr 0x%02X not responding\r\n", dev->addr);
        sleep_ms(10);
    }

    return false;
}

bool sdp610_read_raw(sdp610_t *dev, int16_t *raw)
{
    if (!dev || !dev->i2c || !raw)
    {
        return false;
    }

    uint8_t data[2];
    int rc = i2c_read_blocking(dev->i2c, dev->addr, data, 2, false);
    if (rc != 2)
    {
        printf("SDP610: raw read failed\r\n");
        dev->connected = false;
        return false;
    }

    *raw = (int16_t)((uint16_t)data[0] << 8 | data[1]);
    dev->connected = true;
    return true;
}

bool sdp610_read_diff_pressure(sdp610_t *dev, float *pressure_pa)
{
    if (!dev || !pressure_pa || dev->scale_factor == 0.0f)
    {
        return false;
    }

    int16_t raw = 0;
    if (!sdp610_read_raw(dev, &raw))
    {
        return false;
    }

    *pressure_pa = (float)raw / dev->scale_factor;
    return true;
}

bool sdp610_read_airspeed(sdp610_t *dev, float temp_c, float magic_number, float *airspeed_mps)
{
    if (!dev || !airspeed_mps)
    {
        return false;
    }

    dev->connected = false;
    if (!dev->initialized)
    {
        return false;
    }

    float dp = 0.0f;
    if (!sdp610_read_diff_pressure(dev, &dp))
    {
        return false;
    }

    float rho = 1.251f - temp_c * 0.004f;
    if (rho <= 0.0f)
    {
        printf("SDP610: air density is zero or negative\r\n");
        return false;
    }

    float corrected_dp = dp * 0.75f;
    if (corrected_dp < 0.0f)
    {
        return false;
    }

    dev->airspeed = magic_number * sqrtf(2.0f * corrected_dp / rho);
    *airspeed_mps = dev->airspeed;
    dev->connected = true;
    return true;
}
