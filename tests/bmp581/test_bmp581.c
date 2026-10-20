#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "hardware/i2c.h"
#include "bmp581.h"

static uint8_t regs[256], selected;
static int writes, fail_reg, fallback;
static unsigned elapsed_ms, ready_at;
static i2c_inst_t bus;
static bmp581_t dev = { .i2c = &bus, .addr = BMP581_I2C_ADDR };

static void reset_mock(void)
{
    memset(regs, 0, sizeof(regs));
    regs[BMP581_REG_CHIP_ID] = BMP581_CHIP_ID;
    regs[BMP581_REG_STATUS] = 0x02;
    writes = 0;
    fail_reg = -1;
    fallback = 0;
    elapsed_ms = ready_at = 0;
}

void sleep_ms(uint32_t ms) { elapsed_ms += ms; }

int i2c_write_blocking(i2c_inst_t *i2c, uint8_t addr,
                       const uint8_t *data, size_t n, bool nostop)
{
    (void)i2c; (void)addr; (void)nostop;
    selected = data[0];
    if (n == 1) return 1;
    writes++;
    // A short write must fail initialization, even if nonnegative.
    if (selected == fail_reg) return (int)n - 1;
    if (selected == BMP581_REG_CMD) {
        regs[BMP581_REG_OSR_CONFIG] = 0;
        regs[BMP581_REG_ODR_CONFIG] = 0;
    } else {
        memcpy(&regs[selected], &data[1], n - 1);
    }
    if (selected == BMP581_REG_ODR_CONFIG) {
        regs[BMP581_REG_OSR_EFF] = fallback ? 0x89 :
            (uint8_t)(0x80 | (regs[BMP581_REG_OSR_CONFIG] & 0x3f));
    }
    return (int)n;
}

int i2c_read_blocking(i2c_inst_t *i2c, uint8_t addr,
                      uint8_t *data, size_t n, bool nostop)
{
    (void)i2c; (void)addr; (void)nostop;
    if (selected == BMP581_REG_INT_STATUS) {
        *data = elapsed_ms >= ready_at ? BMP581_DATA_READY_BIT : 0;
    } else {
        memcpy(data, &regs[selected], n);
    }
    return (int)n;
}

int main(void)
{
    bmp581_config_t config = {
        .osr_t = BMP581_OSR_T_1, .osr_p = BMP581_OSR_P_16,
        .odr_code = BMP581_ODR_CODE_100_299_HZ,
        .pwr_mode = BMP581_PWR_MODE_CONTINUOUS,
    };
    reset_mock();
    assert(bmp581_init_with_config(&dev, &config));
    assert(regs[BMP581_REG_OSR_CONFIG] == 0x60);
    assert((regs[BMP581_REG_ODR_CONFIG] & 3) == 3);

    // Highest-resolution setting survives both initial startup and recovery.
    config.osr_t = BMP581_OSR_T_8;
    config.osr_p = BMP581_OSR_P_128;
    for (int i = 0; i < 2; i++) {
        assert(bmp581_init_with_config(&dev, &config));
        assert(regs[BMP581_REG_OSR_CONFIG] == 0x7b);
    }

    reset_mock();
    assert(bmp581_init(&dev));
    assert(regs[BMP581_REG_OSR_CONFIG] == 0x52);
    assert(regs[BMP581_REG_ODR_CONFIG] == 0x29);

    reset_mock();
    config.osr_p = 8;
    assert(!bmp581_init_with_config(&dev, &config));
    assert(!bmp581_init_with_config(&dev, NULL));
    assert(!bmp581_init_with_config(NULL, &config));
    assert(writes == 0);

    reset_mock();
    fail_reg = BMP581_REG_OSR_CONFIG;
    assert(!bmp581_init(&dev));
    reset_mock();
    fail_reg = BMP581_REG_INT_SOURCE;
    assert(!bmp581_init(&dev));
    reset_mock();
    fail_reg = BMP581_REG_ODR_CONFIG;
    assert(!bmp581_init(&dev));

    reset_mock();
    fallback = 1;
    assert(!bmp581_init(&dev));

    float temp, press;
    reset_mock();
    ready_at = 110;
    assert(bmp581_main(&dev, &temp, &press));
    assert(elapsed_ms == 110);
    reset_mock();
    ready_at = 1000;
    assert(!bmp581_main(&dev, &temp, &press));
    assert(elapsed_ms == 250);
    puts("BMP581 configuration and timeout tests passed");
    return 0;
}
