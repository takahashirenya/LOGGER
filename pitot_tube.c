#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "SDP610.h"
#include "sdcard.h"

#define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7

#define I2C_PORT i2c1
#define I2C_SDA 30
#define I2C_SCL 31
#define I2C_BAUDRATE (100 * 1000)

#define SENSOR_COUNT 3

static void pitot_tube_read_rowdata(sdp610_t sensors[SENSOR_COUNT], char *rowdata, size_t rowdata_len)
{
    uint32_t timestamp = to_ms_since_boot(get_absolute_time());
    int16_t raw_values[SENSOR_COUNT] = {0};
    bool raw_valid[SENSOR_COUNT] = {false};

    for (size_t i = 0; i < SENSOR_COUNT; i++) {
        raw_valid[i] = sdp610_read_raw(&sensors[i], &raw_values[i]);
    }

    char raw_text[SENSOR_COUNT][16];
    for (size_t i = 0; i < SENSOR_COUNT; i++) {
        if (raw_valid[i]) {
            snprintf(raw_text[i], sizeof(raw_text[i]), "%d", raw_values[i]);
        } else {
            snprintf(raw_text[i], sizeof(raw_text[i]), "NaN");
        }
    }

    snprintf(rowdata, rowdata_len, "%lu,%s,%s,%s",
             (unsigned long)timestamp,
             raw_text[0],
             raw_text[1],
             raw_text[2]);
}

int main()
{
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    sleep_ms(200);

    printf("pitot_tube boot OK\r\n");

    spi_init(SPI_PORT, 8000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    SDCardLogger *logger = sdcard_init(SPI_PORT, PIN_CS, 1, "log_");
    if (!logger || !logger->initialized) {
        printf("SD card init failed\r\n");
    } else {
        sdcard_make_file(logger);
        sdcard_write(logger, "timestamp,raw1,raw2,raw3");
        printf("SD card OK\r\n");
    }

    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    const uint8_t sensor_addresses[SENSOR_COUNT] = {0x40, 0x44, 0x4F};
    sdp610_t sensors[SENSOR_COUNT];

    for (size_t i = 0; i < SENSOR_COUNT; i++) {
        sdp610_init_struct(&sensors[i],
                           I2C_PORT,
                           sensor_addresses[i],
                           true,
                           SDP610_DEFAULT_TIMEOUT);

        if (!sdp610_init(&sensors[i])) {
            printf("SDP610 0x%02X init failed\r\n", sensor_addresses[i]);
        } else {
            printf("SDP610 0x%02X init OK\r\n", sensor_addresses[i]);
        }
    }

    while (true) {
        char rowdata[160];
        pitot_tube_read_rowdata(sensors, rowdata, sizeof(rowdata));

        printf("%s\r\n", rowdata);
        if (logger && logger->initialized) {
            sdcard_write(logger, rowdata);
        }

        sleep_ms(100);
    }
}
