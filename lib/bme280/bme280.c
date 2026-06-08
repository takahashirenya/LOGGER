#include "bme280.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

static bool bme_write_reg(bme280_t *dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    int rc = i2c_write_blocking(dev->i2c, dev->addr, buf, 2, false);
    return rc == 2;
}

static bool bme_read_regs(bme280_t *dev, uint8_t reg, uint8_t *buf, size_t len) {
    int rc = i2c_write_blocking(dev->i2c, dev->addr, &reg, 1, true);
    if (rc != 1) return false;
    rc = i2c_read_blocking(dev->i2c, dev->addr, buf, len, false);
    return rc == (int)len;
}

static uint8_t read_u8(bme280_t *dev, uint8_t reg) {
    uint8_t v = 0;
    bme_read_regs(dev, reg, &v, 1);
    return v;
}

static uint16_t read_u16_le(bme280_t *dev, uint8_t reg) {
    uint8_t b[2];
    bme_read_regs(dev, reg, b, 2);
    return (uint16_t)(b[1] << 8 | b[0]);
}

static int16_t read_s16_le(bme280_t *dev, uint8_t reg) {
    return (int16_t)read_u16_le(dev, reg);
}

static int16_t convert_signed_12(uint16_t v) {
    if (v & 0x800) {
        return (int16_t)(v | 0xF000);
    }
    return (int16_t)v;
}

static bool bme280_load_calibration(bme280_t *dev) {
    dev->calib.dig_T1 = read_u16_le(dev, 0x88);
    dev->calib.dig_T2 = read_s16_le(dev, 0x8A);
    dev->calib.dig_T3 = read_s16_le(dev, 0x8C);

    dev->calib.dig_P1 = read_u16_le(dev, 0x8E);
    dev->calib.dig_P2 = read_s16_le(dev, 0x90);
    dev->calib.dig_P3 = read_s16_le(dev, 0x92);
    dev->calib.dig_P4 = read_s16_le(dev, 0x94);
    dev->calib.dig_P5 = read_s16_le(dev, 0x96);
    dev->calib.dig_P6 = read_s16_le(dev, 0x98);
    dev->calib.dig_P7 = read_s16_le(dev, 0x9A);
    dev->calib.dig_P8 = read_s16_le(dev, 0x9C);
    dev->calib.dig_P9 = read_s16_le(dev, 0x9E);

    dev->calib.dig_H1 = read_u8(dev, 0xA1);
    dev->calib.dig_H2 = read_s16_le(dev, 0xE1);
    dev->calib.dig_H3 = read_u8(dev, 0xE3);

    uint8_t e4 = read_u8(dev, 0xE4);
    uint8_t e5 = read_u8(dev, 0xE5);
    uint8_t e6 = read_u8(dev, 0xE6);

    dev->calib.dig_H4 = convert_signed_12((uint16_t)((e4 << 4) | (e5 & 0x0F)));
    dev->calib.dig_H5 = convert_signed_12((uint16_t)(((e5 >> 4) & 0x0F) | (e6 << 4)));
    dev->calib.dig_H6 = (int8_t)read_u8(dev, 0xE7);

    return true;
}

static float compensate_temperature(bme280_t *dev, int32_t adc_T) {
    float v1 = ((adc_T / 16384.0f) - (dev->calib.dig_T1 / 1024.0f)) * dev->calib.dig_T2;
    float v2 = (((adc_T / 131072.0f) - (dev->calib.dig_T1 / 8192.0f)) *
                ((adc_T / 131072.0f) - (dev->calib.dig_T1 / 8192.0f))) * dev->calib.dig_T3;
    dev->t_fine = (int32_t)(v1 + v2);
    return (v1 + v2) / 5120.0f;
}

static float compensate_humidity(bme280_t *dev, int32_t adc_H) {
    float h = dev->t_fine - 76800.0f;
    if (h == 0.0f) return 0.0f;

    h = (adc_H - (dev->calib.dig_H4 * 64.0f + (dev->calib.dig_H5 / 16384.0f) * h)) *
        ((dev->calib.dig_H2 / 65536.0f) *
         (1.0f + (dev->calib.dig_H6 / 67108864.0f) * h *
         (1.0f + (dev->calib.dig_H3 / 67108864.0f) * h)));

    h = h * (1.0f - dev->calib.dig_H1 * h / 524288.0f);

    if (h > 100.0f) h = 100.0f;
    if (h < 0.0f) h = 0.0f;
    return h;
}

static float compensate_pressure(bme280_t *dev, int32_t adc_P) {
    float v1, v2, p;

    v1 = (dev->t_fine / 2.0f) - 64000.0f;
    v2 = (((v1 / 4.0f) * (v1 / 4.0f)) / 2048.0f) * dev->calib.dig_P6;
    v2 = v2 + ((v1 * dev->calib.dig_P5) * 2.0f);
    v2 = (v2 / 4.0f) + (dev->calib.dig_P4 * 65536.0f);
    v1 = (((dev->calib.dig_P3 * (((v1 / 4.0f) * (v1 / 4.0f)) / 8192.0f)) / 8.0f)
        + ((dev->calib.dig_P2 * v1) / 2.0f)) / 262144.0f;
    v1 = ((32768.0f + v1) * dev->calib.dig_P1) / 32768.0f;

    if (v1 == 0.0f) return 0.0f;

    p = ((1048576.0f - adc_P) - (v2 / 4096.0f)) * 3125.0f;
    if (p < 0x80000000) {
        p = (p * 2.0f) / v1;
    } else {
        p = (p / v1) * 2.0f;
    }

    v1 = (dev->calib.dig_P9 * (((p / 8.0f) * (p / 8.0f)) / 8192.0f)) / 4096.0f;
    v2 = ((p / 4.0f) * dev->calib.dig_P8) / 8192.0f;
    p = p + ((v1 + v2 + dev->calib.dig_P7) / 16.0f);

    return p / 100.0f; // hPa
}

bool bme280_init(bme280_t *dev) {
    dev->flag_init = false;
    dev->flag_conn = false;
    dev->tmp = 0.0f;
    dev->hum = 0.0f;
    dev->pres = 0.0f;

    if (!dev->flag_use) {
        return false;
    }

    uint8_t chip_id = 0;
    if (!bme_read_regs(dev, 0xD0, &chip_id, 1)) {
        return false;
    }
    if (chip_id != BME280_CHIP_ID) {
        return false;
    }

    for (int i = 0; i < dev->time_out; i++) {
        if (bme_write_reg(dev, 0xF2, 0x01) &&   // hum x1
            bme_write_reg(dev, 0xF4, 0x27)) {   // temp x1, press x1, normal mode
            bme280_load_calibration(dev);
            dev->flag_init = true;
            return true;
        }
        sleep_ms(10);
    }

    return false;
}

bool bme280_read_data(bme280_t *dev, float *temp_c, float *hum_pct, float *press_hpa) {
    uint8_t block[8];

    dev->flag_conn = false;
    if (!dev->flag_init) return false;
    if (!bme_read_regs(dev, 0xF7, block, 8)) return false;

    int32_t adc_P = (int32_t)((block[0] << 12) | (block[1] << 4) | (block[2] >> 4));
    int32_t adc_T = (int32_t)((block[3] << 12) | (block[4] << 4) | (block[5] >> 4));
    int32_t adc_H = (int32_t)((block[6] << 8) | block[7]);

    *temp_c   = compensate_temperature(dev, adc_T);
    *hum_pct  = compensate_humidity(dev, adc_H);
    *press_hpa = compensate_pressure(dev, adc_P);

    dev->tmp  = *temp_c;
    dev->hum  = *hum_pct;
    dev->pres = *press_hpa;
    dev->flag_conn = true;

    return true;
}

bool bme280_main(bme280_t *dev, float *temp_c, float *hum_pct, float *press_hpa) {
    return bme280_read_data(dev, temp_c, hum_pct, press_hpa);
}