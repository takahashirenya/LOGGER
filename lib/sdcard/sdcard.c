#include "sdcard.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Manual SPI initialization (128 clock cycles with CS low)
static void manual_spi_init(spi_inst_t *spi, uint cs_pin) {
    gpio_put(cs_pin, 0);  // CS LOW
    sleep_us(10);
    
    for (int i = 0; i < 16; i++) {
        uint8_t dummy = 0xFF;
        spi_write_blocking(spi, &dummy, 1);
    }
    
    gpio_put(cs_pin, 1);  // CS HIGH
    sleep_ms(1);
}

// Initialize SPI with specified baudrate
static void init_spi(spi_inst_t *spi, uint baudrate) {
    spi_set_baudrate(spi, baudrate);
}

// Send command to SD card and get response
static int8_t sd_cmd(BlockDev *bd, uint8_t cmd, uint32_t arg, uint8_t crc, 
                     uint8_t final_bytes, bool release, bool skip1) {
    uint8_t response;
    
    gpio_put(bd->cs_pin, 0);  // CS LOW
    
    // Build and send command
    bd->cmdbuf[0] = 0x40 | cmd;
    bd->cmdbuf[1] = (arg >> 24) & 0xFF;
    bd->cmdbuf[2] = (arg >> 16) & 0xFF;
    bd->cmdbuf[3] = (arg >> 8) & 0xFF;
    bd->cmdbuf[4] = arg & 0xFF;
    bd->cmdbuf[5] = crc;
    spi_write_blocking(bd->spi, bd->cmdbuf, 6);
    
    if (skip1) {
        uint8_t dummy = 0xFF;
        spi_write_blocking(bd->spi, &dummy, 1);
    }
    
    // Wait for response
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        uint8_t dummy = 0xFF;
        spi_read_blocking(bd->spi, dummy, &response, 1);
        
        if (!(response & 0x80)) {
            // Read final bytes
            for (int j = 0; j < final_bytes; j++) {
                spi_read_blocking(bd->spi, 0xFF, bd->tokenbuf, 1);
            }
            
            if (release) {
                uint8_t dummy = 0xFF;
                spi_write_blocking(bd->spi, &dummy, 1);
            }
            gpio_put(bd->cs_pin, 1);  // CS HIGH
            return response;
        }
    }
    
    // Timeout
    uint8_t dummy = 0xFF;
    spi_write_blocking(bd->spi, &dummy, 1);
    gpio_put(bd->cs_pin, 1);  // CS HIGH
    return -1;
}

// Initialize SD Card v1
static bool init_card_v1(BlockDev *bd) {
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        sd_cmd(bd, 55, 0, 0, 0, true, false);
        if (sd_cmd(bd, 41, 0, 0, 0, true, false) == 0) {
            bd->cdv = 512;
            printf("[SDCard] v1 card\n");
            return true;
        }
    }
    return false;
}

// Initialize SD Card v2
static bool init_card_v2(BlockDev *bd) {
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        sleep_ms(50);
        sd_cmd(bd, 58, 0, 0, 4, true, false);
        sd_cmd(bd, 55, 0, 0, 0, true, false);
        if (sd_cmd(bd, 41, 0x40000000, 0, 0, true, false) == 0) {
            sd_cmd(bd, 58, 0, 0, 4, true, false);
            bd->cdv = 1;
            printf("[SDCard] v2 card\n");
            return true;
        }
    }
    return false;
}

// Initialize SD Card
static bool init_card(BlockDev *bd) {
    manual_spi_init(bd->spi, bd->cs_pin);
    init_spi(bd->spi, 100000);  // Low baudrate for init
    gpio_put(bd->cs_pin, 1);
    
    // CMD0: reset card
    for (int i = 0; i < 5; i++) {
        if (sd_cmd(bd, 0, 0, 0x95, 0, true, false) == R1_IDLE_STATE) {
            break;
        }
    }
    
    // CMD8: determine card version
    int8_t r = sd_cmd(bd, 8, 0x01AA, 0x87, 4, true, false);
    
    if (r == R1_IDLE_STATE) {
        if (!init_card_v2(bd)) {
            printf("SD Card v2 init failed\n");
            return false;
        }
    } else if (r == (R1_IDLE_STATE | R1_ILLEGAL_COMMAND)) {
        if (!init_card_v1(bd)) {
            printf("SD Card v1 init failed\n");
            return false;
        }
    } else {
        printf("Could not determine SD card version\n");
        return false;
    }
    
    // CMD16: set block length to 512 bytes
    if (sd_cmd(bd, 16, BLOCK_SIZE, 0, 0, true, false) != 0) {
        printf("Can't set block size\n");
        return false;
    }
    
    // Set high data rate
    init_spi(bd->spi, 8000000);
    
    // Clock card 100+ cycles with CS high
    for (int i = 0; i < 16; i++) {
        uint8_t dummy = 0xFF;
        spi_write_blocking(bd->spi, &dummy, 1);
    }
    
    return true;
}

// Initialize block device
BlockDev *block_dev_init(spi_inst_t *spi, uint cs_pin, uint32_t gbyte) {
    BlockDev *bd = malloc(sizeof(BlockDev));
    if (!bd) return NULL;
    
    bd->spi = spi;
    bd->cs_pin = cs_pin;
    bd->sectors = gbyte * 1024 * 1024 * 2;
    
    // Initialize CS pin (just set it to output, don't call gpio_init on SPI pins)
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);
    
    if (!init_card(bd)) {
        free(bd);
        return NULL;
    }
    
    bd->initialized = true;
    return bd;
}

// Deinitialize block device
void block_dev_deinit(BlockDev *bd) {
    if (bd) {
        free(bd);
    }
}

// Read blocks from SD card
int block_dev_readblocks(BlockDev *bd, uint32_t block_num, uint8_t *buf) {
    if (!bd || !bd->initialized) return -1;
    
    // CMD17: read single block
    gpio_put(bd->cs_pin, 0);
    sd_cmd(bd, 17, block_num * bd->cdv, 0, 0, false, false);
    
    // Wait for data token
    for (int i = 0; i < CMD_TIMEOUT; i++) {
        uint8_t dummy = 0xFF;
        spi_read_blocking(bd->spi, dummy, bd->tokenbuf, 1);
        if (bd->tokenbuf[0] == TOKEN_DATA) {
            break;
        }
    }
    
    // Read data
    spi_read_blocking(bd->spi, 0xFF, buf, BLOCK_SIZE);
    
    // Read and ignore checksum
    uint8_t dummy = 0xFF;
    spi_read_blocking(bd->spi, dummy, bd->tokenbuf, 1);
    spi_read_blocking(bd->spi, dummy, bd->tokenbuf, 1);
    
    gpio_put(bd->cs_pin, 1);
    return 0;
}

// Write blocks to SD card
int block_dev_writeblocks(BlockDev *bd, uint32_t block_num, uint8_t *buf) {
    if (!bd || !bd->initialized) return -1;
    
    // CMD24: write single block
    gpio_put(bd->cs_pin, 0);
    sd_cmd(bd, 24, block_num * bd->cdv, 0, 0, true, false);
    
    // Send data token
    uint8_t token = TOKEN_DATA;
    spi_write_blocking(bd->spi, &token, 1);
    
    // Send data
    spi_write_blocking(bd->spi, buf, BLOCK_SIZE);
    
    // Send dummy checksum
    uint8_t dummy = 0xFF;
    spi_write_blocking(bd->spi, &dummy, 1);
    spi_write_blocking(bd->spi, &dummy, 1);
    
    // Check response
    spi_read_blocking(bd->spi, 0xFF, bd->tokenbuf, 1);
    if ((bd->tokenbuf[0] & 0x1F) != 0x05) {
        spi_write_blocking(bd->spi, &dummy, 1);
        gpio_put(bd->cs_pin, 1);
        return -1;
    }
    
    // Wait for write to finish
    do {
        spi_read_blocking(bd->spi, 0xFF, bd->tokenbuf, 1);
    } while (bd->tokenbuf[0] == 0);
    
    spi_write_blocking(bd->spi, &dummy, 1);
    gpio_put(bd->cs_pin, 1);
    return 0;
}

// Initialize SD card logger
SDCardLogger *sdcard_init(spi_inst_t *spi, uint cs_pin, uint32_t gbyte, const char *filename_prefix) {
    SDCardLogger *logger = malloc(sizeof(SDCardLogger));
    if (!logger) return NULL;
    
    // Initialize block device
    BlockDev *bd = block_dev_init(spi, cs_pin, gbyte);
    if (!bd) {
        free(logger);
        return NULL;
    }
    
    logger->block_dev = *bd;
    free(bd);
    
    strncpy(logger->filename_prefix, filename_prefix, sizeof(logger->filename_prefix) - 1);
    logger->filename_prefix[sizeof(logger->filename_prefix) - 1] = '\0';
    logger->file = NULL;
    logger->file_open = false;
    logger->initialized = true;
    logger->write_counter = 0;
    
    return logger;
}

// Deinitialize SD card logger
void sdcard_deinit(SDCardLogger *logger) {
    if (logger) {
        if (logger->file) {
            fclose(logger->file);
        }
        free(logger);
    }
}

// Create new log file
void sdcard_make_file(SDCardLogger *logger) {
    if (!logger || !logger->initialized) {
        printf("SDcard: INIT ERROR\n");
        return;
    }
    
    for (int i = 1; i < 100; i++) {
        snprintf(logger->filepath, sizeof(logger->filepath), "/sd/log_%d.txt", i);
        
        logger->file = fopen(logger->filepath, "r");
        if (logger->file) {
            fclose(logger->file);
            continue;
        }
        
        logger->file = fopen(logger->filepath, "w");
        if (logger->file) {
            printf("%s\n", logger->filepath);
            logger->file_open = true;
            return;
        }
    }
    
    printf("SDcard: Could not create file\n");
}

// Write data to log file
void sdcard_write(SDCardLogger *logger, const char *data) {
    if (!logger || !logger->initialized) {
        printf("SDcard: USE/INIT failed\n");
        return;
    }
    
    if (!logger->file_open) {
        printf("SDcard: File not open\n");
        return;
    }
    
    if (fprintf(logger->file, "%s\n", data) < 0) {
        printf("SDcard: Write error\n");
        if (logger->file) {
            fclose(logger->file);
            logger->file = fopen(logger->filepath, "a");
        }
        return;
    }
    
    // Flush every 10 writes
    logger->write_counter++;
    if (logger->write_counter % 10 == 0) {
        fflush(logger->file);
    }
}

// Set filename prefix
void sdcard_set_filename_prefix(SDCardLogger *logger, const char *prefix) {
    if (logger) {
        strncpy(logger->filename_prefix, prefix, sizeof(logger->filename_prefix) - 1);
        logger->filename_prefix[sizeof(logger->filename_prefix) - 1] = '\0';
    }
}
