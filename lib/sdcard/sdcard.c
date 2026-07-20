#include "sdcard.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "diskio.h"

#define SDCARD_DRIVE "0:"
#define SD_INIT_BAUDRATE 100000
#define SD_DATA_BAUDRATE 1000000
#define DATA_TOKEN_TIMEOUT_MS 500
#define WRITE_BUSY_TIMEOUT_MS 1000

static BlockDev *active_block_dev = NULL;

// Send idle clocks with CS high so the card enters SPI mode before CMD0.
static void manual_spi_init(spi_inst_t *spi, uint cs_pin) {
    gpio_put(cs_pin, 1);  // CS HIGH
    sleep_us(10);
    
    for (int i = 0; i < 16; i++) {
        uint8_t dummy = 0xFF;
        spi_write_blocking(spi, &dummy, 1);
    }
    
    sleep_ms(1);
}

// Initialize SPI with specified baudrate
static void init_spi(spi_inst_t *spi, uint baudrate) {
    spi_set_baudrate(spi, baudrate);
}

static void sd_release(BlockDev *bd) {
    uint8_t dummy = 0xFF;
    gpio_put(bd->cs_pin, 1);  // CS HIGH
    spi_write_blocking(bd->spi, &dummy, 1);
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
                sd_release(bd);
            }
            return response;
        }
    }
    
    // Timeout
    sd_release(bd);
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
    int8_t last_cmd58 = -1;
    int8_t last_cmd55 = -1;
    int8_t last_acmd41 = -1;

    for (int i = 0; i < CMD_TIMEOUT; i++) {
        sleep_ms(50);
        last_cmd58 = sd_cmd(bd, 58, 0, 0, 4, true, false);
        last_cmd55 = sd_cmd(bd, 55, 0, 0, 0, true, false);
        last_acmd41 = sd_cmd(bd, 41, 0x40000000, 0, 0, true, false);
        if (last_acmd41 == 0) {
            sd_cmd(bd, 58, 0, 0, 4, true, false);
            bd->cdv = 1;
            printf("[SDCard] v2 card\n");
            return true;
        }
    }

    printf("[SDCard] v2 timeout: CMD58=%d CMD55=%d ACMD41=%d\n",
           last_cmd58, last_cmd55, last_acmd41);
    return false;
}

// Initialize SD Card
static bool init_card(BlockDev *bd) {
    init_spi(bd->spi, SD_INIT_BAUDRATE);  // Low baudrate for init
    manual_spi_init(bd->spi, bd->cs_pin);
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
    init_spi(bd->spi, SD_DATA_BAUDRATE);
    
    // Clock card 100+ cycles with CS high
    gpio_put(bd->cs_pin, 1);
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

// Read one block from SD card
int block_dev_readblocks(BlockDev *bd, uint32_t block_num, uint8_t *buf) {
    if (!bd || !bd->initialized) return -1;
    
    // CMD17: read single block
    gpio_put(bd->cs_pin, 0);
    int8_t r = sd_cmd(bd, 17, block_num * bd->cdv, 0, 0, false, false);
    if (r != 0) {
        printf("[SDCard] CMD17 failed: block=%lu r=%d\n",
               (unsigned long)block_num, r);
        sd_release(bd);
        return -1;
    }
    
    // Wait for data token
    bool token_found = false;
    absolute_time_t deadline = make_timeout_time_ms(DATA_TOKEN_TIMEOUT_MS);
    do {
        uint8_t dummy = 0xFF;
        spi_read_blocking(bd->spi, dummy, bd->tokenbuf, 1);
        if (bd->tokenbuf[0] == TOKEN_DATA) {
            token_found = true;
            break;
        }
    } while (!time_reached(deadline));
    if (!token_found) {
        printf("[SDCard] read token timeout: block=%lu last=0x%02X\n",
               (unsigned long)block_num, bd->tokenbuf[0]);
        sd_release(bd);
        return -1;
    }
    
    // Read data
    spi_read_blocking(bd->spi, 0xFF, buf, BLOCK_SIZE);
    
    // Read and ignore checksum
    uint8_t dummy = 0xFF;
    spi_read_blocking(bd->spi, dummy, bd->tokenbuf, 1);
    spi_read_blocking(bd->spi, dummy, bd->tokenbuf, 1);
    
    sd_release(bd);
    return 0;
}

// Write one block to SD card
int block_dev_writeblocks(BlockDev *bd, uint32_t block_num, const uint8_t *buf) {
    if (!bd || !bd->initialized) return -1;
    
    // CMD24: write single block
    gpio_put(bd->cs_pin, 0);
    if (sd_cmd(bd, 24, block_num * bd->cdv, 0, 0, false, false) != 0) {
        sd_release(bd);
        return -1;
    }
    
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
        sd_release(bd);
        return -1;
    }
    
    // Wait for write to finish
    absolute_time_t deadline = make_timeout_time_ms(WRITE_BUSY_TIMEOUT_MS);
    do {
        spi_read_blocking(bd->spi, 0xFF, bd->tokenbuf, 1);
        if (time_reached(deadline)) {
            printf("[SDCard] write busy timeout: block=%lu\n",
                   (unsigned long)block_num);
            sd_release(bd);
            return -1;
        }
    } while (bd->tokenbuf[0] == 0);
    
    sd_release(bd);
    return 0;
}

DSTATUS disk_initialize(BYTE pdrv) {
    if (pdrv != 0 || !active_block_dev || !active_block_dev->initialized) {
        return STA_NOINIT;
    }
    return 0;
}

DSTATUS disk_status(BYTE pdrv) {
    if (pdrv != 0 || !active_block_dev || !active_block_dev->initialized) {
        return STA_NOINIT;
    }
    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || !active_block_dev || !buff || count == 0) {
        return RES_PARERR;
    }

    for (UINT i = 0; i < count; i++) {
        if (block_dev_readblocks(active_block_dev, (uint32_t)(sector + i), buff + i * BLOCK_SIZE) != 0) {
            return RES_ERROR;
        }
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || !active_block_dev || !buff || count == 0) {
        return RES_PARERR;
    }

    for (UINT i = 0; i < count; i++) {
        if (block_dev_writeblocks(active_block_dev, (uint32_t)(sector + i), buff + i * BLOCK_SIZE) != 0) {
            return RES_ERROR;
        }
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv != 0 || !active_block_dev) {
        return RES_PARERR;
    }

    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_COUNT:
        if (!buff) {
            return RES_PARERR;
        }
        *(LBA_t *)buff = active_block_dev->sectors;
        return RES_OK;
    case GET_SECTOR_SIZE:
        if (!buff) {
            return RES_PARERR;
        }
        *(WORD *)buff = BLOCK_SIZE;
        return RES_OK;
    case GET_BLOCK_SIZE:
        if (!buff) {
            return RES_PARERR;
        }
        *(DWORD *)buff = 1;
        return RES_OK;
    default:
        return RES_PARERR;
    }
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
    active_block_dev = &logger->block_dev;

    FRESULT fr = f_mount(&logger->fs, SDCARD_DRIVE, 1);
    if (fr != FR_OK) {
        printf("SDcard: mount failed: %d\n", fr);
        if (fr == FR_NO_FILESYSTEM) {
            printf("SDcard: no valid FAT volume. Format the card as FAT32, not exFAT.\n");
        }
        free(logger);
        active_block_dev = NULL;
        return NULL;
    }
    
    strncpy(logger->filename_prefix, filename_prefix, sizeof(logger->filename_prefix) - 1);
    logger->filename_prefix[sizeof(logger->filename_prefix) - 1] = '\0';
    memset(&logger->file, 0, sizeof(logger->file));
    logger->file_open = false;
    logger->initialized = true;
    logger->write_counter = 0;
    
    return logger;
}

// Deinitialize SD card logger
void sdcard_deinit(SDCardLogger *logger) {
    if (logger) {
        if (logger->file_open) {
            f_close(&logger->file);
        }
        f_mount(NULL, SDCARD_DRIVE, 0);
        active_block_dev = NULL;
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
        snprintf(logger->filepath, sizeof(logger->filepath), SDCARD_DRIVE "/%s%d.csv", logger->filename_prefix, i);

        FRESULT fr = f_open(&logger->file, logger->filepath, FA_WRITE | FA_CREATE_NEW);
        if (fr == FR_EXIST) {
            continue;
        }
        if (fr == FR_OK) {
            printf("%s\n", logger->filepath);
            logger->file_open = true;
            return;
        }
        printf("SDcard: f_open failed: %d\n", fr);
        break;
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
        FRESULT reopen_result = f_open(&logger->file, logger->filepath,
                                       FA_WRITE | FA_OPEN_APPEND);
        if (reopen_result != FR_OK) {
            printf("SDcard: Reopen failed: %d\n", reopen_result);
            return;
        }
        logger->file_open = true;
        printf("SDcard: File reopened\n");
    }
    
    UINT written = 0;
    FRESULT fr = f_write(&logger->file, data, strlen(data), &written);
    if (fr == FR_OK) {
        fr = f_write(&logger->file, "\n", 1, &written);
    }
    if (fr != FR_OK) {
        printf("SDcard: Write error: %d\n", fr);
        f_close(&logger->file);
        if (f_open(&logger->file, logger->filepath, FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
            logger->file_open = true;
        } else {
            logger->file_open = false;
        }
        return;
    }
    
    // Keep each logger loop on the card even if power is removed.
    logger->write_counter++;
    fr = f_sync(&logger->file);
    if (fr != FR_OK) {
        printf("SDcard: Sync error: %d\n", fr);
        f_close(&logger->file);
        logger->file_open = false;
    }
}

// Set filename prefix
void sdcard_set_filename_prefix(SDCardLogger *logger, const char *prefix) {
    if (logger) {
        strncpy(logger->filename_prefix, prefix, sizeof(logger->filename_prefix) - 1);
        logger->filename_prefix[sizeof(logger->filename_prefix) - 1] = '\0';
    }
}
