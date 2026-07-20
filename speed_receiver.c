#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

// SPI1 slave pins. The master's MOSI/TX (GPIO39) connects to this RX (GPIO36).
#define PICO_SPI          spi1
#define PICO_SPI_RX_PIN   36
#define PICO_SPI_CS_PIN   37
#define PICO_SPI_SCK_PIN  38
#define PICO_SPI_TX_PIN   39
#define PICO_SPI_BAUD     (1 * 1000 * 1000)

int main(void)
{
    stdio_init_all();

    spi_init(PICO_SPI, PICO_SPI_BAUD);
    spi_set_slave(PICO_SPI, true);
    spi_set_format(PICO_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PICO_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_SPI_CS_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_SPI_TX_PIN, GPIO_FUNC_SPI);

    printf("Pico SPI speed receiver ready\r\n");

    while (true) {
        uint8_t speed_data[sizeof(float)];
        int received = spi_read_blocking(PICO_SPI, 0, speed_data, sizeof(speed_data));
        if (received == (int)sizeof(speed_data)) {
            float speed_mps;
            memcpy(&speed_mps, speed_data, sizeof(speed_mps));
            printf("speed=%.3f m/s\r\n", speed_mps);
        }
    }
}
