#pragma once // include guard

#include <stdint.h>
#include <stdbool.h>

typedef struct i2c_inst i2c_inst_t;

// --- Registers ---
#define BMP581_REG_CMD 0x7E
#define BMP581_REG_OSR_EFF 0x38
#define BMP581_REG_ODR_CONFIG 0x37
#define BMP581_REG_OSR_CONFIG 0x36
#define BMP581_REG_OOR_CONFIG 0x35
#define BMP581_REG_OOR_RANGE 0x34
#define BMP581_REG_OOR_THR_P_MSB 0x33
#define BMP581_REG_OOR_THR_P_LSB 0x32
#define BMP581_REG_DSP_IIR 0x31
#define BMP581_REG_DSP_CONFIG 0x30
#define BMP581_REG_NVM_DATA_MSB 0x2D
#define BMP581_REG_NVM_DATA_LSB 0x2C
#define BMP581_REG_NVM_ADDR 0x2B
#define BMP581_REG_FIFO_DATA 0x29
#define BMP581_REG_STATUS 0x28
#define BMP581_REG_INT_STATUS 0x27
#define BMP581_REG_PRESS_XLSB 0x20
#define BMP581_REG_TEMP_XLSB 0x1D
#define BMP581_REG_FIFO_SEL 0x18
#define BMP581_REG_FIFO_COUNT 0x17
#define BMP581_REG_INT_SOURCE 0x15
#define BMP581_REG_INT_CONFIG 0x14
#define BMP581_REG_DRIVE_CONFIG 0x13
#define BMP581_REG_CHIP_STATUS 0x11
#define BMP581_REG_REV_ID 0x02
#define BMP581_REG_CHIP_ID 0x01

#define BMP581_CHIP_ID 0x50
#define BMP581_I2C_ADDR 0x47     // SDO pin is GND
#define BMP581_I2C_ADDR_ALT 0x46 // SDO pin is VDDIO

// Oversampling settings

#define BMP581_OSR_T_1 0x00
#define BMP581_OSR_T_2 0x01
#define BMP581_OSR_T_4 0x02
#define BMP581_OSR_T_8 0x03
#define BMP581_OSR_T_16 0x04
#define BMP581_OSR_T_32 0x05
#define BMP581_OSR_T_64 0x06
#define BMP581_OSR_T_128 0x07

#define BMP581_OSR_P_1 0x00
#define BMP581_OSR_P_2 0x01
#define BMP581_OSR_P_4 0x02
#define BMP581_OSR_P_8 0x03
#define BMP581_OSR_P_16 0x04
#define BMP581_OSR_P_32 0x05
#define BMP581_OSR_P_64 0x06
#define BMP581_OSR_P_128 0x07

// Output data rate settings (normal mode)
// ===== ODR codes (ODR_CONFIG[6:2]) =====
#define BMP581_ODR_CODE_240_000_HZ 0x00 // 240.000 Hz
#define BMP581_ODR_CODE_218_537_HZ 0x01 // 218.537 Hz
#define BMP581_ODR_CODE_199_111_HZ 0x02 // 199.111 Hz
#define BMP581_ODR_CODE_179_200_HZ 0x03 // 179.200 Hz
#define BMP581_ODR_CODE_160_000_HZ 0x04 // 160.000 Hz
#define BMP581_ODR_CODE_149_333_HZ 0x05 // 149.333 Hz
#define BMP581_ODR_CODE_140_000_HZ 0x06 // 140.000 Hz
#define BMP581_ODR_CODE_129_855_HZ 0x07 // 129.855 Hz
#define BMP581_ODR_CODE_120_000_HZ 0x08 // 120.000 Hz
#define BMP581_ODR_CODE_110_164_HZ 0x09 // 110.164 Hz
#define BMP581_ODR_CODE_100_299_HZ 0x0A // 100.299 Hz
#define BMP581_ODR_CODE_089_600_HZ 0x0B //  89.600 Hz
#define BMP581_ODR_CODE_080_000_HZ 0x0C //  80.000 Hz
#define BMP581_ODR_CODE_070_000_HZ 0x0D //  70.000 Hz
#define BMP581_ODR_CODE_060_000_HZ 0x0E //  60.000 Hz
#define BMP581_ODR_CODE_050_056_HZ 0x0F //  50.056 Hz
#define BMP581_ODR_CODE_045_025_HZ 0x10 //  45.025 Hz
#define BMP581_ODR_CODE_040_000_HZ 0x11 //  40.000 Hz
#define BMP581_ODR_CODE_035_000_HZ 0x12 //  35.000 Hz
#define BMP581_ODR_CODE_030_000_HZ 0x13 //  30.000 Hz
#define BMP581_ODR_CODE_025_005_HZ 0x14 //  25.005 Hz
#define BMP581_ODR_CODE_020_000_HZ 0x15 //  20.000 Hz
#define BMP581_ODR_CODE_015_000_HZ 0x16 //  15.000 Hz
#define BMP581_ODR_CODE_010_000_HZ 0x17 //  10.000 Hz
#define BMP581_ODR_CODE_005_000_HZ 0x18 //   5.000 Hz
#define BMP581_ODR_CODE_004_000_HZ 0x19 //   4.000 Hz
#define BMP581_ODR_CODE_003_000_HZ 0x1A //   3.000 Hz
#define BMP581_ODR_CODE_002_000_HZ 0x1B //   2.000 Hz
#define BMP581_ODR_CODE_001_000_HZ 0x1C //   1.000 Hz
#define BMP581_ODR_CODE_000_500_HZ 0x1D //   0.500 Hz
#define BMP581_ODR_CODE_000_250_HZ 0x1E //   0.250 Hz
#define BMP581_ODR_CODE_000_125_HZ 0x1F //   0.125 Hz

// Power mode settings (ODR_CONFIG[1:0])
// 0b00=sleep, 0b01=normal, 0b10=forced, 0b11=continuous
#define BMP581_PWR_MODE_SLEEP 0x00
#define BMP581_PWR_MODE_NORMAL 0x01
#define BMP581_PWR_MODE_FORCED 0x02
#define BMP581_PWR_MODE_CONTINUOUS 0x03

// intterrupt react
#define BMP581_DATA_READY_BIT 0x01
#define BMP581_RESET_CMD 0xB6
#define BMP581_WAIT_MS 5

#ifdef __cplusplus
extern "C" // for C++ compilers
{
#endif
    typedef struct
    {
        i2c_inst_t *i2c; /* example: i2c0 / i2c1 */
        uint8_t addr;    /* I2C address */
    } bmp581_t;

    // initialization
    bool bmp581_init(bmp581_t *dev);

    // settings
    bool bmp581_set_osr(bmp581_t *dev, uint8_t osr_t, uint8_t osr_p, bool press_en);
    bool bmp581_set_odr_mode(bmp581_t *dev, uint8_t odr_code, uint8_t pwr_mode);

    // Raw data read
    bool bmp581_read_temp_press_raw(bmp581_t *dev, float *temp_c, float *press_pa);

    // Data Ready flag check
    bool bmp581_data_ready(bmp581_t *dev);

    // Main function (init + read)
    bool bmp581_main(bmp581_t *dev, float *temp_c, float *press_pa);

#ifdef __cplusplus
}
#endif