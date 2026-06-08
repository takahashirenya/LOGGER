#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct i2c_inst i2c_inst_t;

#define BME280_I2C_ADDR_PRIMARY 0x76
#define BME280_I2C_ADDR_SECONDARY 0x77
#define BME280_CHIP_ID 0x60

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_t;

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;

    bool flag_use;
    bool flag_init;
    bool flag_conn;
    int  time_out;

    float tmp;
    float hum;
    float pres;

    bme280_calib_t calib;
    int32_t t_fine;
} bme280_t;

#ifdef __cplusplus
extern "C" {
#endif

bool bme280_init(bme280_t *dev);
bool bme280_read_data(bme280_t *dev, float *temp_c, float *hum_pct, float *press_hpa);
bool bme280_main(bme280_t *dev, float *temp_c, float *hum_pct, float *press_hpa);

#ifdef __cplusplus
}
#endif