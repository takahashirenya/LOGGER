#include "bmp581.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

static inline bool i2c_ok(int rc) { return rc >= 0; }

static bool bmp_read_reg(bmp581_t *dev, uint8_t reg, uint8_t *data, size_t n)
{
    int rc = i2c_write_blocking(dev->i2c, dev->addr, &reg, 1, true);
    if (rc != 1)
        return false;
    rc = i2c_read_blocking(dev->i2c, dev->addr, data, n, false);
    return rc == (int)n;
}

static bool bmp_write_reg(bmp581_t *dev, uint8_t reg, const uint8_t *data, size_t n)
{
    uint8_t buf[1 + 8];
    if (n > 8)
        return false;
    buf[0] = reg;
    for (size_t i = 0; i < n; ++i)
        buf[1 + i] = data[i];
    int rc = i2c_write_blocking(dev->i2c, dev->addr, buf, 1 + n, false);
    return i2c_ok(rc);
}

// ---- OSRsetting：osr_t(bit2-0), osr_p(bit5-3), press_en(bit6) ----
bool bmp581_set_osr(bmp581_t *dev, uint8_t osr_t, uint8_t osr_p, bool press_en)
{
    if (osr_t > 7 || osr_p > 7)
        return false;
    uint8_t v = (press_en ? (1u << 6) : 0) | ((osr_p & 0x7) << 3) | (osr_t & 0x7);
    return bmp_write_reg(dev, BMP581_REG_OSR_CONFIG, &v, 1);
}

// ---- ODR setting：odr_code(bit6-2), pwr_mode(bit1-0) ----
bool bmp581_set_odr_mode(bmp581_t *dev, uint8_t odr_code, uint8_t pwr_mode)
{
    if (odr_code > 0x1F || pwr_mode > 3)
        return false;
    uint8_t v = (odr_code << 2) | (pwr_mode & 0x03);
    return bmp_write_reg(dev, BMP581_REG_ODR_CONFIG, &v, 1);
}

// ---- initialization ----
bool bmp581_init(bmp581_t *dev)
{

    // check device
    uint8_t chip_id = 0;
    if (!bmp_read_reg(dev, BMP581_REG_CHIP_ID, &chip_id, 1))
    {
        return false;
    }
    if (chip_id != BMP581_CHIP_ID)
    {
        return false;
    }

    // reset device
    uint8_t rst = 0xB6;
    if (!bmp_write_reg(dev, BMP581_REG_CMD, &rst, 1))
    {
        return false;
    }
    sleep_ms(5);

    // I/F mode check
    uint8_t chip_status = 0;
    if (!bmp_read_reg(dev, BMP581_REG_CHIP_STATUS, &chip_status, 1))
    {
        return false;
    }
    uint8_t hif = chip_status & 0x03;
    if (hif == 0x01 || hif == 0x02)
    {
        return false;
    }

    // NVM check
    uint8_t status = 0;
    if (!bmp_read_reg(dev, BMP581_REG_STATUS, &status, 1))
    {
        return false;
    }
    if ((status & 0x06) != 0x02)
    {
        return false;
    }

    // default setting
    if (!bmp581_set_osr(dev, 0b010, 0b010, true))
    {
        return false;
    }

    if (!bmp581_set_odr_mode(dev, BMP581_ODR_CODE_100_299_HZ, BMP581_PWR_MODE_NORMAL))
    {
        return false;
    }

    // INT_SOURCE setting (data ready)
    uint8_t int_src = 0x01; // bit0 = drdy_data_reg_en
    bmp_write_reg(dev, BMP581_REG_INT_SOURCE, &int_src, 1);

    return true;
}

bool bmp581_read_temp_press_raw(bmp581_t *dev, float *temp_c, float *press_pa)
{
    uint8_t b[6];
    if (!bmp_read_reg(dev, BMP581_REG_TEMP_XLSB, b, 6))
        return false;
    // temperature 24bit (MSB..XLSB = b[2],b[1],b[0])
    int32_t t24 = ((int32_t)b[2] << 16) | ((int32_t)b[1] << 8) | b[0];
    if (t24 & 0x800000)
        t24 |= 0xFF000000;
    *temp_c = (float)t24 / 65536.0f;

    // pressure 24bit (MSB..XLSB = b[5],b[4],b[3])
    int32_t p24 = ((int32_t)b[5] << 16) | ((int32_t)b[4] << 8) | b[3];
    if (p24 & 0x800000)
        p24 |= 0xFF000000;
    *press_pa = (float)p24 / 64.0f;

    return true;
}

bool bmp581_data_ready(bmp581_t *dev)
{
    uint8_t status;
    if (!bmp_read_reg(dev, BMP581_REG_INT_STATUS, &status, 1))
    {
        return false;
    }
    return (status & BMP581_DATA_READY_BIT) != 0;
}

// ---- main ----
bool bmp581_main(bmp581_t *dev, float *temp_c, float *press_pa)
{
    const int max_wait_ms = 100; // max wait 100ms
    int waited_ms = 0;
    while (!bmp581_data_ready(dev))
    {
        sleep_ms(10);
        waited_ms += 10;
        if (waited_ms >= max_wait_ms)
        {
            return false;
        }
    }

    return bmp581_read_temp_press_raw(dev, temp_c, press_pa);
}
