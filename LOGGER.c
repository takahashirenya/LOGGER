#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "sdcard.h"
#include "bme280.h"
#include "bmp581.h"
#include "bno08x.h"
#include "mcp2515.h"

// SPI Defines (SD Card)
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_MOSI 19
#define PIN_SCK  18
#define PIN_CS   21

// I2C0 Defines (BMP581, BNO08x)
#define I2C0_PORT i2c0
#define I2C0_SDA 0
#define I2C0_SCL 1

// I2C1 Defines (BME280)
#define I2C1_PORT i2c1
#define I2C1_SDA 30
#define I2C1_SCL 31

// CAN Defines (SPI1)
#define CAN_SPI         spi1
#define CAN_SCK_PIN     10
#define CAN_MOSI_PIN    11
#define CAN_MISO_PIN    12
#define CAN_CS_PIN      9
#define CAN_INT_PIN     8

// UART defines
#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 4
#define UART_RX_PIN 5



int main()
{
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    sleep_ms(200);

    printf("BOOT OK\r\n");

    // ========== SPI0 Initialization (SD Card) ==========
    spi_init(SPI_PORT, 8000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // ========== I2C0 Initialization (BMP581, BNO08x) ==========
    i2c_init(I2C0_PORT, 400*1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);

    // ========== I2C1 Initialization (BME280) ==========
    i2c_init(I2C1_PORT, 100*1000);
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);

    // ========== UART Initialization ==========
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_puts(UART_ID, "LOGGER initialized\r\n");

    // ========== BME280 Initialization ==========
    printf("Initializing BME280...\r\n");
    bme280_t bme;
    bme.i2c = I2C1_PORT;
    bme.addr = BME280_I2C_ADDR_PRIMARY;
    bme.flag_use = true;
    bme.time_out = 10;

    if (!bme280_init(&bme)) {
        printf("BME280 init failed\r\n");
    } else {
        printf("BME280 init OK\r\n");
    }

    // ========== BMP581 Initialization ==========
    printf("Initializing BMP581...\r\n");
    bmp581_t bmp;
    bmp.i2c = I2C0_PORT;
    bmp.addr = BMP581_I2C_ADDR;

    if (!bmp581_init(&bmp)) {
        printf("BMP581 init failed\r\n");
    } else {
        printf("BMP581 init OK\r\n");
    }

    // ========== BNO08x Initialization ==========
    printf("BNO08x init start...\r\n");
    sleep_ms(2000);

    if (!bno08x_init(I2C0_PORT, 0)) {
        printf("BNO08x init FAILED\r\n");
    } else {
        printf("BNO08x init OK, addr=0x%02X\r\n", bno08x_get_address());

        if (!bno08x_enable_feature(BNO_REPORT_ACCELEROMETER, BNO_DEFAULT_REPORT_INTERVAL_US)) {
            printf("enable accel failed\r\n");
        }
        if (!bno08x_enable_feature(BNO_REPORT_LINEAR_ACCELERATION, BNO_DEFAULT_REPORT_INTERVAL_US)) {
            printf("enable linear accel failed\r\n");
        }
        if (!bno08x_enable_feature(BNO_REPORT_GYROSCOPE, BNO_DEFAULT_REPORT_INTERVAL_US)) {
            printf("enable gyro failed\r\n");
        }
        if (!bno08x_enable_feature(BNO_REPORT_ROTATION_VECTOR, BNO_DEFAULT_REPORT_INTERVAL_US)) {
            printf("enable rotation vector failed\r\n");
        }
        printf("BNO08x feature enable done\r\n");
    }

    // ========== CAN (MCP2515) Initialization ==========
    printf("Initializing CAN...\r\n");
    mcp2515_t can;
    mcp2515_init_struct(&can,
                        CAN_SPI,
                        CAN_SCK_PIN,
                        CAN_MOSI_PIN,
                        CAN_MISO_PIN,
                        CAN_CS_PIN,
                        CAN_INT_PIN,
                        1 * 1000 * 1000);

    mcp2515_error_t can_ret = mcp2515_begin(&can, MCP2515_BITRATE_500KBPS, MCP2515_CLOCK_8MHZ);
    if (can_ret != MCP2515_OK) {
        printf("CAN init error: %d\r\n", can_ret);
    } else {
        printf("CAN init OK\r\n");
    }

    // ========== SD Card Logger Initialization ==========
    printf("Initializing SD card...\r\n");
    SDCardLogger *logger = sdcard_init(SPI_PORT, PIN_CS, 1, "log_");
    
    if (!logger || !logger->initialized) {
        printf("Failed to initialize SD card\r\n");
    } else {
        printf("Creating log file...\r\n");
        sdcard_make_file(logger);
        printf("SD card OK\r\n");
    }

    // ========== Main Loop ==========
    int loop_count = 0;
    int can_tx_count = 0;

    bno08x_quat_t init_q = {
        .w = 0.9971924f,
        .x = -0.01263428f,
        .y = -0.07397461f,
        .z = 0.0003051758f
    };

    while (true) {
        loop_count++;

        printf("===== Loop %d =====\r\n", loop_count);

        // ========== BME280 Read ==========
        float bme_temp, bme_hum, bme_press;
        if (bme280_main(&bme, &bme_temp, &bme_hum, &bme_press)) {
            printf("BME280: Temp=%.2f C, Hum=%.2f %%, Press=%.2f hPa\r\n",
                   bme_temp, bme_hum, bme_press);
        } else {
            printf("BME280: read error\r\n");
        }

        // ========== BMP581 Read ==========
        float bmp_temp, bmp_press;
        if (bmp581_main(&bmp, &bmp_temp, &bmp_press)) {
            printf("BMP581: Temp=%.2f C, Press=%.2f Pa\r\n", bmp_temp, bmp_press);
        } else {
            printf("BMP581: read error\r\n");
        }

        // ========== BNO08x Read ==========
        float ax, ay, az;
        float lax, lay, laz;
        float gx, gy, gz;
        float w, x, y, z;

        bool ok_acc = bno08x_read_accel(&ax, &ay, &az);
        bool ok_lacc = bno08x_read_linear_accel(&lax, &lay, &laz);
        bool ok_gyro = bno08x_read_gyro(&gx, &gy, &gz);
        bool ok_quat = bno08x_read_quaternion(&w, &x, &y, &z);

        if (ok_acc) {
            printf("BNO08x ACC: %.3f %.3f %.3f\r\n", ax, ay, az);
        }
        if (ok_lacc) {
            printf("BNO08x LACC: %.3f %.3f %.3f\r\n", lax, lay, laz);
        }
        if (ok_gyro) {
            printf("BNO08x GYRO: %.3f %.3f %.3f\r\n", gx, gy, gz);
        }
        if (ok_quat) {
            printf("BNO08x QUAT: %.5f %.5f %.5f %.5f\r\n", w, x, y, z);
            bno08x_quat_t q_now = { .w = w, .x = x, .y = y, .z = z };
            bno08x_quat_t q_corr = bno08x_apply_reference(init_q, q_now);
            bno08x_euler_t e = bno08x_quat_to_euler_deg(q_corr);
            printf("BNO08x EULER: R=%.2f P=%.2f Y=%.2f\r\n", e.roll_deg, e.pitch_deg, e.yaw_deg);
        }

        // ========== CAN Send ==========
        can_tx_count++;
        can_frame_t tx_frame = {
            .id = 0x456,
            .dlc = 4,
            .data = {(uint8_t)loop_count, (uint8_t)(loop_count >> 8), can_tx_count, 0},
            .extended = false,
            .rtr = false,
        };

        mcp2515_error_t tx_err = mcp2515_send_message(&can, &tx_frame);
        if (tx_err == MCP2515_OK) {
            printf("CAN TX OK: ID=0x%03X, Data=[%02X %02X %02X %02X]\r\n",
                   tx_frame.id, tx_frame.data[0], tx_frame.data[1],
                   tx_frame.data[2], tx_frame.data[3]);
        } else if (tx_err == MCP2515_ALL_TX_BUSY) {
            printf("CAN TX busy\r\n");
        } else {
            printf("CAN TX error: %d\r\n", tx_err);
        }

        // ========== CAN Receive ==========
        if (mcp2515_check_receive(&can)) {
            can_frame_t rx_frame;
            mcp2515_error_t rx_err = mcp2515_read_message(&can, &rx_frame);
            if (rx_err == MCP2515_OK) {
                printf("CAN RX OK: ID=0x%03X, DLC=%u, Data=[", rx_frame.id, rx_frame.dlc);
                for (uint8_t i = 0; i < rx_frame.dlc; i++) {
                    printf("%02X ", rx_frame.data[i]);
                }
                printf("]\r\n");
            }
        }

        // ========== SD Card Log ==========
        if (logger && logger->initialized) {
            char log_buffer[256];
            snprintf(log_buffer, sizeof(log_buffer),
                     "Loop:%d BME280(T:%.2f H:%.2f P:%.2f) BMP581(T:%.2f P:%.2f) BNO08x(A:%.1f %.1f %.1f)\r\n",
                     loop_count, bme_temp, bme_hum, bme_press, bmp_temp, bmp_press, ax, ay, az);
            sdcard_write(logger, log_buffer);
        }

        printf("\r\n");
        sleep_ms(1000);
    }

    if (logger) {
        sdcard_deinit(logger);
    }
    return 0;
}
