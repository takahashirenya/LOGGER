#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "sdcard.h"
#include "bme280.h"
#include "bmp581.h"
#include "bno08x.h"
#include "SDP610.h"

// SPI Defines (SD Card)
#define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_MOSI 7
#define PIN_SCK  6
#define PIN_CS   5

// I2C0 Defines (BMP581, BNO08x)
#define I2C0_PORT i2c0
#define I2C0_SDA 0
#define I2C0_SCL 1

// I2C1 Defines (BME280, SDP610)
#define I2C1_PORT i2c1
#define I2C1_SDA 30
#define I2C1_SCL 31

// UART link to STEERING
#define STEERING_UART        uart0
#define STEERING_UART_TX_PIN 32
#define STEERING_UART_RX_PIN 33
#define STEERING_UART_BAUD   115200
#define AIRSPEED_DATA_SIZE   ((uint8_t)sizeof(float))
#define AIRSPEED_FRAME_SIZE  8U
#define PWM_FRAME_HEADER_1   0xA5U
#define PWM_FRAME_HEADER_2   0x5AU
#define PWM_DATA_SIZE        5U
#define PWM_FRAME_SIZE       9U

#define SAMPLE_PERIOD_MS     20U
#define STATUS_PERIOD_LOOPS  50
#define LOG_QUEUE_DEPTH      32U
#define LOG_LINE_SIZE        768U
#define LOG_BATCH_LINES      8U
#define LOG_BATCH_TIMEOUT_MS 200U
#define LOG_BATCH_SIZE       (LOG_LINE_SIZE * LOG_BATCH_LINES)

// GNSS SPI1 (u-blox, NMEA GNRMC only)
#define GNSS_SPI             spi1
#define GNSS_SPI_RX_PIN      8
#define GNSS_SPI_CS_PIN      9
#define GNSS_SPI_SCK_PIN     10
#define GNSS_SPI_TX_PIN      11
#define GNSS_SPI_BAUD        (1 * 1000 * 1000)
#define GNSS_READ_SIZE       100U
#define GNSS_POLL_MS         20U
#define GNSS_LINE_SIZE       128U

#define WATCHDOG_TIMEOUT_MS       10000U
#define IO_HEARTBEAT_TIMEOUT_MS    6000U
#define SENSOR_RETRY_INTERVAL_MS   1000U
#define GNSS_STALE_RETRY_MS        3000U

#define SDP610_SENSOR_COUNT 3
#define SDP610_INIT_TIMEOUT 100
#define PITOT_CALIBRATION_TEMPERATURE_C 23.9f
#define PITOT_CALIBRATION_KA 0.000259759f
#define PITOT_CALIBRATION_OFFSET_MPS 0.140523f

static const uint8_t sdp610_addresses[SDP610_SENSOR_COUNT] = {0x40, 0x44, 0x4F};

typedef struct {
    char line[LOG_LINE_SIZE];
} log_record_t;

static queue_t log_queue;
static SDCardLogger *sd_logger = NULL;
static volatile uint32_t log_drop_count = 0;
static char log_batch[LOG_BATCH_SIZE];

typedef struct {
    bool received;
    bool checksum_ok;
    bool valid;
    uint32_t last_ms;
    char rmc[GNSS_LINE_SIZE];
} gnss_data_t;

static critical_section_t gnss_lock;
static gnss_data_t gnss_latest;
static char gnss_line_buffer[GNSS_LINE_SIZE];
static size_t gnss_line_length = 0;
static volatile uint32_t io_heartbeat_ms = 0;

typedef struct {
    bool received;
    uint16_t lad_pwm_us;
    uint16_t ele_pwm_us;
    uint8_t hatch_gear;
    uint32_t last_ms;
    uint32_t rx_count;
} steering_pwm_data_t;

static steering_pwm_data_t steering_pwm_latest;
static uint8_t pwm_rx_frame[PWM_FRAME_SIZE];
static uint8_t pwm_rx_index = 0;

static void steering_pwm_uart_process(void)
{
    while (uart_is_readable(STEERING_UART)) {
        uint8_t byte = uart_getc(STEERING_UART);
        if (pwm_rx_index == 0 && byte != PWM_FRAME_HEADER_1) {
            continue;
        }
        if (pwm_rx_index == 1 && byte != PWM_FRAME_HEADER_2) {
            pwm_rx_index = (byte == PWM_FRAME_HEADER_1) ? 1 : 0;
            continue;
        }
        if (pwm_rx_index == 2 && byte != PWM_DATA_SIZE) {
            pwm_rx_index = (byte == PWM_FRAME_HEADER_1) ? 1 : 0;
            continue;
        }

        pwm_rx_frame[pwm_rx_index++] = byte;
        if (pwm_rx_index == PWM_FRAME_SIZE) {
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < PWM_FRAME_SIZE - 1; i++) {
                checksum ^= pwm_rx_frame[i];
            }
            if (checksum == pwm_rx_frame[PWM_FRAME_SIZE - 1]) {
                steering_pwm_latest.lad_pwm_us =
                    (uint16_t)pwm_rx_frame[3] | ((uint16_t)pwm_rx_frame[4] << 8);
                steering_pwm_latest.ele_pwm_us =
                    (uint16_t)pwm_rx_frame[5] | ((uint16_t)pwm_rx_frame[6] << 8);
                steering_pwm_latest.hatch_gear = pwm_rx_frame[7];
                steering_pwm_latest.last_ms = to_ms_since_boot(get_absolute_time());
                steering_pwm_latest.rx_count++;
                steering_pwm_latest.received = true;
            }
            pwm_rx_index = 0;
        }
    }
}

static void ubx_checksum(const uint8_t *data, size_t length, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < length; i++) {
        *ck_a = (uint8_t)(*ck_a + data[i]);
        *ck_b = (uint8_t)(*ck_b + *ck_a);
    }
}

static void gnss_send_cfg_msg(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    uint8_t command[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,
                           msg_class, msg_id, rate, 0, 0};
    ubx_checksum(&command[2], 7, &command[9], &command[10]);
    gpio_put(GNSS_SPI_CS_PIN, 0);
    busy_wait_us_32(5);
    spi_write_blocking(GNSS_SPI, command, sizeof(command));
    busy_wait_us_32(5);
    gpio_put(GNSS_SPI_CS_PIN, 1);
    sleep_ms(100);
}

static void gnss_set_25hz_rate(void)
{
    // UBX-CFG-VALSET, RAM layer:
    // CFG-RATE-MEAS=40 ms, CFG-RATE-NAV=1,
    // CFG-MSGOUT-NMEA_ID_RMC_SPI=1 (every navigation solution).
    uint8_t command[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x15, 0x00,
        0x00, 0x01, 0x00, 0x00,
        0x01, 0x00, 0x21, 0x30, 0x28, 0x00,
        0x02, 0x00, 0x21, 0x30, 0x01, 0x00,
        0xAF, 0x00, 0x91, 0x20, 0x01,
        0x00, 0x00,
    };
    ubx_checksum(&command[2], sizeof(command) - 4,
                 &command[sizeof(command) - 2], &command[sizeof(command) - 1]);
    gpio_put(GNSS_SPI_CS_PIN, 0);
    busy_wait_us_32(5);
    spi_write_blocking(GNSS_SPI, command, sizeof(command));
    busy_wait_us_32(5);
    gpio_put(GNSS_SPI_CS_PIN, 1);
    sleep_ms(100);
}

static void gnss_init(void)
{
    spi_init(GNSS_SPI, GNSS_SPI_BAUD);
    spi_set_format(GNSS_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(GNSS_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(GNSS_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(GNSS_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_init(GNSS_SPI_CS_PIN);
    gpio_set_dir(GNSS_SPI_CS_PIN, GPIO_OUT);
    gpio_put(GNSS_SPI_CS_PIN, 1);

    sleep_ms(2000);
    const uint8_t cfg[][3] = {
        {0xF0, 0x00, 0}, {0xF0, 0x01, 0}, {0xF0, 0x02, 0},
        {0xF0, 0x03, 0}, {0xF0, 0x05, 0}, {0xF0, 0x41, 0},
        {0xF0, 0x04, 1},
    };
    for (size_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
        gnss_send_cfg_msg(cfg[i][0], cfg[i][1], cfg[i][2]);
    }
    gnss_set_25hz_rate();
    printf("GNSS SPI ready (%u Hz, navigation 25 Hz)\r\n", GNSS_SPI_BAUD);
}

static bool nmea_checksum_ok(const char *line)
{
    if (!line || line[0] != '$') {
        return false;
    }
    const char *star = strchr(line, '*');
    if (!star || strlen(star + 1) < 2) {
        return false;
    }
    uint8_t checksum = 0;
    for (const char *p = line + 1; p < star; p++) {
        checksum ^= (uint8_t)*p;
    }
    char hex[3] = {star[1], star[2], '\0'};
    return checksum == (uint8_t)strtoul(hex, NULL, 16);
}

static void gnss_store_rmc(const char *line)
{
    bool checksum_ok = nmea_checksum_ok(line);
    const char *time_end = strchr(line, ',');
    const char *status = time_end ? strchr(time_end + 1, ',') : NULL;
    bool navigation_valid = checksum_ok && status && status[1] == 'A' && status[2] == ',';
    gnss_data_t data = {
        .received = true,
        .checksum_ok = checksum_ok,
        .valid = navigation_valid,
        .last_ms = to_ms_since_boot(get_absolute_time()),
    };
    size_t length = strnlen(line, sizeof(data.rmc) - 1);
    for (size_t i = 0; i < length; i++) {
        data.rmc[i] = (line[i] == ',') ? ';' : line[i];
    }
    data.rmc[length] = '\0';

    critical_section_enter_blocking(&gnss_lock);
    gnss_latest = data;
    critical_section_exit(&gnss_lock);
}

static void gnss_process_byte(uint8_t byte)
{
    if (byte == 0xFF || byte == 0x00 || byte == '\r') {
        return;
    }
    if (byte == '\n') {
        gnss_line_buffer[gnss_line_length] = '\0';
        if (strncmp(gnss_line_buffer, "$GNRMC", 6) == 0) {
            gnss_store_rmc(gnss_line_buffer);
        }
        gnss_line_length = 0;
        return;
    }
    if (byte == '$') {
        gnss_line_length = 0;
    }
    if (gnss_line_length < sizeof(gnss_line_buffer) - 1) {
        gnss_line_buffer[gnss_line_length++] = (char)byte;
    } else {
        gnss_line_length = 0;
    }
}

static void gnss_poll(void)
{
    uint8_t data[GNSS_READ_SIZE];
    gpio_put(GNSS_SPI_CS_PIN, 0);
    spi_read_blocking(GNSS_SPI, 0xFF, data, sizeof(data));
    gpio_put(GNSS_SPI_CS_PIN, 1);
    for (size_t i = 0; i < sizeof(data); i++) {
        gnss_process_byte(data[i]);
    }
}

static bool bno08x_start(void)
{
    if (!bno08x_init(I2C0_PORT, 0)) {
        return false;
    }

    bool ok = true;
    ok &= bno08x_enable_feature(BNO_REPORT_ACCELEROMETER, 20000U);
    ok &= bno08x_enable_feature(BNO_REPORT_LINEAR_ACCELERATION, 20000U);
    ok &= bno08x_enable_feature(BNO_REPORT_GYROSCOPE, 20000U);
    ok &= bno08x_enable_feature(BNO_REPORT_ROTATION_VECTOR, 20000U);
    return ok;
}

static void io_worker_core(void)
{
    log_record_t record;
    size_t batch_length = 0;
    uint32_t line_count = 0;
    absolute_time_t batch_deadline = get_absolute_time();
    absolute_time_t next_gnss_poll = get_absolute_time();
    uint32_t last_gnss_recovery_ms = to_ms_since_boot(get_absolute_time());

    while (true) {
        io_heartbeat_ms = to_ms_since_boot(get_absolute_time());
        if (time_reached(next_gnss_poll)) {
            gnss_poll();
            next_gnss_poll = make_timeout_time_ms(GNSS_POLL_MS);
        }

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        uint32_t gnss_last_ms;
        bool gnss_received;
        critical_section_enter_blocking(&gnss_lock);
        gnss_last_ms = gnss_latest.last_ms;
        gnss_received = gnss_latest.received;
        critical_section_exit(&gnss_lock);
        if ((!gnss_received || now_ms - gnss_last_ms > GNSS_STALE_RETRY_MS) &&
            now_ms - last_gnss_recovery_ms > GNSS_STALE_RETRY_MS) {
            gnss_set_25hz_rate();
            last_gnss_recovery_ms = now_ms;
        }

        while (line_count < LOG_BATCH_LINES && queue_try_remove(&log_queue, &record)) {
            size_t line_length = strnlen(record.line, sizeof(record.line));
            size_t separator = line_count > 0 ? 1 : 0;
            if (batch_length + separator + line_length >= sizeof(log_batch)) {
                log_drop_count++;
                continue;
            }
            if (separator) {
                log_batch[batch_length++] = '\n';
            }
            memcpy(&log_batch[batch_length], record.line, line_length);
            batch_length += line_length;
            line_count++;
            if (line_count == 1) {
                batch_deadline = make_timeout_time_ms(LOG_BATCH_TIMEOUT_MS);
            }
        }

        if (line_count > 0 &&
            (line_count >= LOG_BATCH_LINES || time_reached(batch_deadline))) {
            log_batch[batch_length] = '\0';
            if (sd_logger) {
                sdcard_write(sd_logger, log_batch);
            }
            io_heartbeat_ms = to_ms_since_boot(get_absolute_time());
            batch_length = 0;
            line_count = 0;
            batch_deadline = get_absolute_time();
        }

        sleep_ms(1);
    }
}

static bool pitot_calculate_speed(int16_t raw, float *speed_mps)
{
    if (!speed_mps) {
        return false;
    }
    if (raw < 0) {
        *speed_mps = 0.0f;
        return true;
    }

    const float pito_k = PITOT_CALIBRATION_KA *
                         sqrtf((PITOT_CALIBRATION_TEMPERATURE_C + 273.15f) * 287.0f);
    *speed_mps = pito_k * sqrtf((float)raw) + PITOT_CALIBRATION_OFFSET_MPS;
    return true;
}



int main()
{
    stdio_init_all();
    sleep_ms(500);

    // In standalone operation there is no USB host consuming stdout. Disable
    // the USB stdio driver so printf calls in this file and the sensor drivers
    // are discarded immediately and cannot affect the sampling loop.
    if (!stdio_usb_connected()) {
        stdio_set_driver_enabled(&stdio_usb, false);
    }

    printf("BOOT OK%s\r\n", watchdog_caused_reboot() ? " (watchdog recovery)" : "");

    // ========== SPI0 Initialization (SD Card) ==========
    spi_init(SPI_PORT, 100*1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
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
    i2c_init(I2C1_PORT, 400*1000);
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);

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
    printf("BNO08x init %s\r\n", bno08x_start() ? "OK" : "FAILED (will retry)");

    // ========== SDP610 Initialization ==========
    sdp610_t sdp610[SDP610_SENSOR_COUNT];
    for (size_t i = 0; i < SDP610_SENSOR_COUNT; i++) {
        sdp610_init_struct(&sdp610[i], I2C1_PORT, sdp610_addresses[i], true,
                           SDP610_INIT_TIMEOUT);
        if (sdp610_init(&sdp610[i])) {
            printf("SDP610 0x%02X init OK\r\n", sdp610_addresses[i]);
        } else {
            printf("SDP610 0x%02X init failed\r\n", sdp610_addresses[i]);
        }
    }

    // ========== UART link to STEERING initialization ==========
    uart_init(STEERING_UART, STEERING_UART_BAUD);
    uart_set_format(STEERING_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(STEERING_UART, true);
    gpio_set_function(STEERING_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(STEERING_UART_RX_PIN, GPIO_FUNC_UART);
    printf("STEERING UART ready (%u baud)\r\n", STEERING_UART_BAUD);

    // ========== SD Card Logger Initialization ==========
    printf("Initializing SD card...\r\n");
    SDCardLogger *logger = sdcard_init(SPI_PORT, PIN_CS, 16, "log_");
    
    if (!logger || !logger->initialized) {
        printf("Failed to initialize SD card\r\n");
    } else {
        printf("Creating log file...\r\n");
        sdcard_make_file(logger);
        sdcard_write(logger,
                     "timestamp_ms,loop,bme280_ok,bme280_temp_c,bme280_humidity_percent,bme280_pressure_hpa,"
                     "bmp581_ok,bmp581_temp_c,bmp581_pressure_pa,"
                     "accel_ok,accel_x_mps2,accel_y_mps2,accel_z_mps2,"
                     "linear_accel_ok,linear_accel_x_mps2,linear_accel_y_mps2,linear_accel_z_mps2,"
                     "gyro_ok,gyro_x_rps,gyro_y_rps,gyro_z_rps,"
                     "quat_ok,quat_w,quat_x,quat_y,quat_z,"
                     "euler_roll_deg,euler_pitch_deg,euler_yaw_deg,"
                     "sdp610_1_ok,sdp610_1_raw,sdp610_2_ok,sdp610_2_raw,"
                     "sdp610_3_ok,sdp610_3_raw,airspeed_valid,airspeed_mps,"
                     "uart_tx_count,uart_tx_status,log_drop_count,"
                     "airspeed_tx_mps,sample_processing_ms,"
                     "steering_pwm_received,steering_pwm_rx_count,steering_pwm_age_ms,"
                     "lad_pwm_us,ele_pwm_us,hatch_gear,"
                     "gnss_received,gnss_checksum_ok,gnss_valid,gnss_age_ms,gnss_gnrmc");
        printf("SD card OK\r\n");
        sd_logger = logger;
    }

    queue_init(&log_queue, sizeof(log_record_t), LOG_QUEUE_DEPTH);
    critical_section_init(&gnss_lock);
    gnss_init();
    io_heartbeat_ms = to_ms_since_boot(get_absolute_time());
    multicore_launch_core1(io_worker_core);
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    // ========== Main Loop ==========
    int loop_count = 0;
    int uart_tx_count = 0;
    uint32_t last_bme_retry_ms = 0;
    uint32_t last_bmp_retry_ms = 0;
    uint32_t last_bno_retry_ms = 0;
    uint32_t bno_failure_count = 0;

    bno08x_quat_t init_q = {
        .w = 0.9971924f,
        .x = -0.01263428f,
        .y = -0.07397461f,
        .z = 0.0003051758f
    };

    while (true) {
        uint32_t loop_started_ms = to_ms_since_boot(get_absolute_time());
        loop_count++;
        bool print_status = (loop_count % STATUS_PERIOD_LOOPS) == 1;

        steering_pwm_uart_process();
        steering_pwm_data_t steering_pwm = steering_pwm_latest;
        uint32_t steering_pwm_age_ms = steering_pwm.received
            ? loop_started_ms - steering_pwm.last_ms
            : 0;

        if (print_status) {
            printf("===== Loop %d =====\r\n", loop_count);
        }

        // ========== BME280 Read ==========
        float bme_temp = 0.0f;
        float bme_hum = 0.0f;
        float bme_press = 0.0f;
        bool ok_bme = bme280_main(&bme, &bme_temp, &bme_hum, &bme_press);
        if (!ok_bme && loop_started_ms - last_bme_retry_ms >= SENSOR_RETRY_INTERVAL_MS) {
            last_bme_retry_ms = loop_started_ms;
            if (bme280_init(&bme)) {
                printf("BME280 recovered\r\n");
            }
        }
        if (print_status && ok_bme) {
            printf("BME280: Temp=%.2f C, Hum=%.2f %%, Press=%.2f hPa\r\n",
                   bme_temp, bme_hum, bme_press);
        } else if (print_status) {
            printf("BME280: read error\r\n");
        }

        // ========== BMP581 Read ==========
        float bmp_temp = 0.0f;
        float bmp_press = 0.0f;
        bool ok_bmp = bmp581_main(&bmp, &bmp_temp, &bmp_press);
        if (!ok_bmp && loop_started_ms - last_bmp_retry_ms >= SENSOR_RETRY_INTERVAL_MS) {
            last_bmp_retry_ms = loop_started_ms;
            if (bmp581_init(&bmp)) {
                printf("BMP581 recovered\r\n");
            }
        }
        if (print_status && ok_bmp) {
            printf("BMP581: Temp=%.2f C, Press=%.2f Pa\r\n", bmp_temp, bmp_press);
        } else if (print_status) {
            printf("BMP581: read error\r\n");
        }

        // ========== BNO08x Read ==========
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;
        float lax = 0.0f;
        float lay = 0.0f;
        float laz = 0.0f;
        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;
        float w = 0.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float roll_deg = 0.0f;
        float pitch_deg = 0.0f;
        float yaw_deg = 0.0f;

        bool ok_acc = bno08x_read_accel(&ax, &ay, &az);
        bool ok_lacc = bno08x_read_linear_accel(&lax, &lay, &laz);
        bool ok_gyro = bno08x_read_gyro(&gx, &gy, &gz);
        bool ok_quat = bno08x_read_quaternion(&w, &x, &y, &z);

        if (ok_acc || ok_lacc || ok_gyro || ok_quat) {
            bno_failure_count = 0;
        } else {
            bno_failure_count++;
            if (bno_failure_count >= 50 &&
                loop_started_ms - last_bno_retry_ms >= SENSOR_RETRY_INTERVAL_MS) {
                last_bno_retry_ms = loop_started_ms;
                printf("BNO08x unresponsive; restarting...\r\n");
                if (bno08x_start()) {
                    printf("BNO08x recovered\r\n");
                }
                bno_failure_count = 0;
            }
        }

        if (print_status && ok_acc) {
            printf("BNO08x ACC: %.3f %.3f %.3f\r\n", ax, ay, az);
        }
        if (print_status && ok_lacc) {
            printf("BNO08x LACC: %.3f %.3f %.3f\r\n", lax, lay, laz);
        }
        if (print_status && ok_gyro) {
            printf("BNO08x GYRO: %.3f %.3f %.3f\r\n", gx, gy, gz);
        }
        if (ok_quat) {
            if (print_status) {
                printf("BNO08x QUAT: %.5f %.5f %.5f %.5f\r\n", w, x, y, z);
            }
            bno08x_quat_t q_now = { .w = w, .x = x, .y = y, .z = z };
            bno08x_quat_t q_corr = bno08x_apply_reference(init_q, q_now);
            bno08x_euler_t e = bno08x_quat_to_euler_deg(q_corr);
            roll_deg = e.roll_deg;
            pitch_deg = e.pitch_deg;
            yaw_deg = e.yaw_deg;
            if (print_status) {
                printf("BNO08x EULER: R=%.2f P=%.2f Y=%.2f\r\n", e.roll_deg, e.pitch_deg, e.yaw_deg);
            }
        }

        // ========== SDP610 Read / Airspeed ==========
        int16_t sdp_raw[SDP610_SENSOR_COUNT] = {0};
        bool sdp_ok[SDP610_SENSOR_COUNT] = {false};
        for (size_t i = 0; i < SDP610_SENSOR_COUNT; i++) {
            if (!sdp610[i].initialized) {
                sdp610_init(&sdp610[i]);
            }
            if (sdp610[i].initialized) {
                sdp_ok[i] = sdp610_read_raw(&sdp610[i], &sdp_raw[i]);
                if (!sdp_ok[i]) {
                    sdp610[i].initialized = false;
                }
            }
        }

        float airspeed_mps = 0.0f;
        bool airspeed_valid = sdp_ok[0] && pitot_calculate_speed(sdp_raw[0], &airspeed_mps);
        if (print_status) {
            printf("SDP610: raw=[%d %d %d], speed=%s",
                   sdp_raw[0], sdp_raw[1], sdp_raw[2], airspeed_valid ? "" : "invalid");
            if (airspeed_valid) {
                printf("%.3f m/s", airspeed_mps);
            }
            printf("\r\n");
        }

        // ========== UART Send (header, length, airspeed float, XOR checksum) ==========
        int uart_tx_status = 0;
        float transmitted_speed_mps = 0.0f;
        if (airspeed_valid) {
            transmitted_speed_mps = airspeed_mps < 1.0f ? 1.0f : airspeed_mps;
            uint8_t frame[AIRSPEED_FRAME_SIZE] = {0xAA, 0x55, AIRSPEED_DATA_SIZE};
            memcpy(&frame[3], &transmitted_speed_mps, AIRSPEED_DATA_SIZE);
            for (uint8_t i = 0; i < AIRSPEED_FRAME_SIZE - 1; i++) {
                frame[AIRSPEED_FRAME_SIZE - 1] ^= frame[i];
            }
            uart_write_blocking(STEERING_UART, frame, AIRSPEED_FRAME_SIZE);
            uart_tx_status = AIRSPEED_FRAME_SIZE;
            uart_tx_count++;
            if (print_status) {
                printf("UART TX OK: speed=%.3f m/s, Frame=[%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
                       transmitted_speed_mps, frame[0], frame[1], frame[2], frame[3],
                       frame[4], frame[5], frame[6], frame[7]);
            }
        } else {
            uart_tx_status = -1;
            if (print_status) {
                printf("UART TX skipped: invalid speed\r\n");
            }
        }

        gnss_data_t gnss;
        critical_section_enter_blocking(&gnss_lock);
        gnss = gnss_latest;
        critical_section_exit(&gnss_lock);
        uint32_t gnss_age_ms = gnss.received
            ? to_ms_since_boot(get_absolute_time()) - gnss.last_ms
            : 0;
        if (print_status) {
            printf("GNSS: received=%d checksum=%d valid=%d age=%lu ms RMC=%s\r\n",
                   gnss.received, gnss.checksum_ok, gnss.valid,
                   (unsigned long)gnss_age_ms,
                   gnss.valid ? gnss.rmc : "none");
            printf("STEERING PWM: received=%d age=%lu ms lad=%u us ele=%u us hatch_gear=%u\r\n",
                   steering_pwm.received, (unsigned long)steering_pwm_age_ms,
                   steering_pwm.lad_pwm_us, steering_pwm.ele_pwm_us,
                   steering_pwm.hatch_gear);
        }

        // ========== SD Card Log ==========
        if (logger && logger->initialized) {
            uint32_t sample_processing_ms =
                to_ms_since_boot(get_absolute_time()) - loop_started_ms;
            log_record_t record;
            snprintf(record.line, sizeof(record.line),
                     "%lu,%d,%d,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%d,%.3f,%.3f,%.3f,"
                     "%d,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f,%d,%.5f,%.5f,%.5f,%.5f,"
                     "%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%.3f,%d,%d,%lu,%.3f,%lu,"
                     "%d,%lu,%lu,%u,%u,%u,%d,%d,%d,%lu,%s",
                     (unsigned long)to_ms_since_boot(get_absolute_time()),
                     loop_count,
                     ok_bme, bme_temp, bme_hum, bme_press,
                     ok_bmp, bmp_temp, bmp_press,
                     ok_acc, ax, ay, az,
                     ok_lacc, lax, lay, laz,
                     ok_gyro, gx, gy, gz,
                     ok_quat, w, x, y, z,
                     roll_deg, pitch_deg, yaw_deg,
                     sdp_ok[0], sdp_raw[0], sdp_ok[1], sdp_raw[1],
                     sdp_ok[2], sdp_raw[2], airspeed_valid, airspeed_mps,
                     uart_tx_count, uart_tx_status, (unsigned long)log_drop_count,
                     transmitted_speed_mps, (unsigned long)sample_processing_ms,
                     steering_pwm.received, (unsigned long)steering_pwm.rx_count,
                     (unsigned long)steering_pwm_age_ms,
                     steering_pwm.lad_pwm_us, steering_pwm.ele_pwm_us,
                     steering_pwm.hatch_gear,
                     gnss.received, gnss.checksum_ok, gnss.valid,
                     (unsigned long)gnss_age_ms,
                     gnss.valid ? gnss.rmc : "");
            if (!queue_try_add(&log_queue, &record)) {
                log_drop_count++;
            }
        }

        uint32_t loop_elapsed_ms = to_ms_since_boot(get_absolute_time()) - loop_started_ms;
        if (print_status) {
            printf("loop=%lu ms, log_drops=%lu, queued=%u\r\n\r\n",
                   (unsigned long)loop_elapsed_ms,
                   (unsigned long)log_drop_count,
                   queue_get_level(&log_queue));
        }
        if (loop_elapsed_ms < SAMPLE_PERIOD_MS) {
            sleep_ms(SAMPLE_PERIOD_MS - loop_elapsed_ms);
        }

        // Core 0 feeds the watchdog only while core 1 is also making progress.
        // A hang in either core therefore causes an automatic reboot and restart.
        uint32_t watchdog_now_ms = to_ms_since_boot(get_absolute_time());
        if (watchdog_now_ms - io_heartbeat_ms <= IO_HEARTBEAT_TIMEOUT_MS) {
            watchdog_update();
        }
    }

    if (logger) {
        sdcard_deinit(logger);
    }
    return 0;
}
