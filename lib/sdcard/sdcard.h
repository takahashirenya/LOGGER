#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "hardware/spi.h"

// SD Card Command Timeout
#define CMD_TIMEOUT 100

// SD Card Response Flags
#define R1_IDLE_STATE       (1 << 0)
#define R1_ILLEGAL_COMMAND  (1 << 2)

// SD Card Tokens
#define TOKEN_CMD25         0xFC
#define TOKEN_STOP_TRAN     0xFD
#define TOKEN_DATA          0xFE

// SD Card Block Size
#define BLOCK_SIZE          512

typedef struct {
    spi_inst_t *spi;
    uint cs_pin;
    uint8_t cmdbuf[6];
    uint8_t dummybuf[BLOCK_SIZE];
    uint8_t tokenbuf[1];
    uint32_t sectors;
    uint32_t cdv;
    bool initialized;
} BlockDev;

typedef struct {
    BlockDev block_dev;
    char filename_prefix[256];
    FILE *file;
    char filepath[512];
    bool file_open;
    bool initialized;
    uint32_t write_counter;
} SDCardLogger;

// Function prototypes
BlockDev *block_dev_init(spi_inst_t *spi, uint cs_pin, uint32_t gbyte);
void block_dev_deinit(BlockDev *bd);
int block_dev_readblocks(BlockDev *bd, uint32_t block_num, uint8_t *buf);
int block_dev_writeblocks(BlockDev *bd, uint32_t block_num, uint8_t *buf);

SDCardLogger *sdcard_init(spi_inst_t *spi, uint cs_pin, uint32_t gbyte, const char *filename_prefix);
void sdcard_deinit(SDCardLogger *logger);
void sdcard_make_file(SDCardLogger *logger);
void sdcard_write(SDCardLogger *logger, const char *data);
void sdcard_set_filename_prefix(SDCardLogger *logger, const char *prefix);

#endif // SDCARD_H
