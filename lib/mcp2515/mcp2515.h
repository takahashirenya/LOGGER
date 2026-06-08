#ifndef MCP2515_H
#define MCP2515_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MCP2515_MAX_DATA_LEN 8

// ===== MCP2515 instructions =====
#define MCP2515_INSTR_WRITE        0x02
#define MCP2515_INSTR_READ         0x03
#define MCP2515_INSTR_BIT_MODIFY   0x05
#define MCP2515_INSTR_READ_STATUS  0xA0
#define MCP2515_INSTR_RESET        0xC0

// ===== MCP2515 registers =====
#define MCP_RXF0SIDH   0x00
#define MCP_RXF0SIDL   0x01
#define MCP_RXF0EID8   0x02
#define MCP_RXF0EID0   0x03
#define MCP_RXF1SIDH   0x04
#define MCP_RXF1SIDL   0x05
#define MCP_RXF1EID8   0x06
#define MCP_RXF1EID0   0x07
#define MCP_CANSTAT    0x0E
#define MCP_CANCTRL    0x0F
#define MCP_RXF2SIDH   0x08
#define MCP_RXF2SIDL   0x09
#define MCP_RXF2EID8   0x0A
#define MCP_RXF2EID0   0x0B
#define MCP_RXF3SIDH   0x10
#define MCP_RXF3SIDL   0x11
#define MCP_RXF3EID8   0x12
#define MCP_RXF3EID0   0x13
#define MCP_RXF4SIDH   0x14
#define MCP_RXF4SIDL   0x15
#define MCP_RXF4EID8   0x16
#define MCP_RXF4EID0   0x17
#define MCP_RXF5SIDH   0x18
#define MCP_RXF5SIDL   0x19
#define MCP_RXF5EID8   0x1A
#define MCP_RXF5EID0   0x1B
#define MCP_RXM0SIDH   0x20
#define MCP_RXM0SIDL   0x21
#define MCP_RXM0EID8   0x22
#define MCP_RXM0EID0   0x23
#define MCP_RXM1SIDH   0x24
#define MCP_RXM1SIDL   0x25
#define MCP_RXM1EID8   0x26
#define MCP_RXM1EID0   0x27
#define MCP_CNF3       0x28
#define MCP_CNF2       0x29
#define MCP_CNF1       0x2A
#define MCP_CANINTE    0x2B
#define MCP_CANINTF    0x2C
#define MCP_EFLG       0x2D
#define MCP_TXB0CTRL   0x30
#define MCP_TXB0SIDH   0x31
#define MCP_TXB0SIDL   0x32
#define MCP_TXB0EID8   0x33
#define MCP_TXB0EID0   0x34
#define MCP_TXB0DLC    0x35
#define MCP_TXB0DATA   0x36
#define MCP_TXB1CTRL   0x40
#define MCP_TXB1SIDH   0x41
#define MCP_TXB1SIDL   0x42
#define MCP_TXB1EID8   0x43
#define MCP_TXB1EID0   0x44
#define MCP_TXB1DLC    0x45
#define MCP_TXB1DATA   0x46
#define MCP_TXB2CTRL   0x50
#define MCP_TXB2SIDH   0x51
#define MCP_TXB2SIDL   0x52
#define MCP_TXB2EID8   0x53
#define MCP_TXB2EID0   0x54
#define MCP_TXB2DLC    0x55
#define MCP_TXB2DATA   0x56
#define MCP_RXB0CTRL   0x60
#define MCP_RXB0SIDH   0x61
#define MCP_RXB0SIDL   0x62
#define MCP_RXB0EID8   0x63
#define MCP_RXB0EID0   0x64
#define MCP_RXB0DLC    0x65
#define MCP_RXB0DATA   0x66
#define MCP_RXB1CTRL   0x70
#define MCP_RXB1SIDH   0x71
#define MCP_RXB1SIDL   0x72
#define MCP_RXB1EID8   0x73
#define MCP_RXB1EID0   0x74
#define MCP_RXB1DLC    0x75
#define MCP_RXB1DATA   0x76

// ===== Bit fields =====
#define CANCTRL_REQOP_MASK    0xE0
#define CANCTRL_MODE_NORMAL   0x00
#define CANCTRL_MODE_SLEEP    0x20
#define CANCTRL_MODE_LOOPBACK 0x40
#define CANCTRL_MODE_LISTEN   0x60
#define CANCTRL_MODE_CONFIG   0x80

#define CANSTAT_OPMOD_MASK    0xE0

#define CANINTF_RX0IF         0x01
#define CANINTF_RX1IF         0x02
#define CANINTF_ERRIF         0x20
#define CANINTF_MERRF         0x80

#define STAT_RX0IF            0x01
#define STAT_RX1IF            0x02
#define STAT_RXIF_MASK        (STAT_RX0IF | STAT_RX1IF)

#define RXB0CTRL_BUKT         0x04
#define RXBnCTRL_RXM_MASK     0x60
#define RXBnCTRL_RXM_STDEXT   0x00

#define TXB_TXREQ             0x08
#define TXB_ABTF              0x40
#define TXB_MLOA              0x20
#define TXB_TXERR             0x10

#define DLC_MASK              0x0F
#define EXIDE_MASK            0x08
#define RTR_MASK              0x40

#define MCP_SIDH_INDEX        0
#define MCP_SIDL_INDEX        1
#define MCP_EID8_INDEX        2
#define MCP_EID0_INDEX        3
#define MCP_DLC_INDEX         4
#define MCP_DATA_INDEX        5

typedef enum {
    MCP2515_OK = 0,
    MCP2515_FAIL = 1,
    MCP2515_ALL_TX_BUSY = 2,
    MCP2515_FAIL_INIT = 3,
    MCP2515_FAIL_TX = 4,
    MCP2515_NO_MSG = 5,
} mcp2515_error_t;

typedef enum {
    MCP2515_CLOCK_8MHZ = 8,
    MCP2515_CLOCK_16MHZ = 16,
    MCP2515_CLOCK_20MHZ = 20,
} mcp2515_clock_t;

typedef enum {
    MCP2515_BITRATE_125KBPS,
    MCP2515_BITRATE_250KBPS,
    MCP2515_BITRATE_500KBPS,
    MCP2515_BITRATE_1000KBPS,
} mcp2515_bitrate_t;

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[MCP2515_MAX_DATA_LEN];
    bool extended;
    bool rtr;
} can_frame_t;

typedef struct {
    spi_inst_t *spi;
    uint cs_pin;
    uint sck_pin;
    uint mosi_pin;
    uint miso_pin;
    uint int_pin;
    uint32_t spi_baudrate;
} mcp2515_t;

void mcp2515_init_struct(mcp2515_t *dev,
                         spi_inst_t *spi,
                         uint sck_pin,
                         uint mosi_pin,
                         uint miso_pin,
                         uint cs_pin,
                         uint int_pin,
                         uint32_t spi_baudrate);

void mcp2515_spi_hw_init(const mcp2515_t *dev);
void mcp2515_reset(const mcp2515_t *dev);
uint8_t mcp2515_read_register(const mcp2515_t *dev, uint8_t reg);
void mcp2515_read_registers(const mcp2515_t *dev, uint8_t reg, uint8_t *buf, uint8_t len);
void mcp2515_write_register(const mcp2515_t *dev, uint8_t reg, uint8_t value);
void mcp2515_write_registers(const mcp2515_t *dev, uint8_t reg, const uint8_t *buf, uint8_t len);
void mcp2515_bit_modify(const mcp2515_t *dev, uint8_t reg, uint8_t mask, uint8_t data);
uint8_t mcp2515_read_status(const mcp2515_t *dev);

mcp2515_error_t mcp2515_set_mode(const mcp2515_t *dev, uint8_t mode);
mcp2515_error_t mcp2515_set_bitrate(const mcp2515_t *dev, mcp2515_bitrate_t bitrate, mcp2515_clock_t clock_mhz);
mcp2515_error_t mcp2515_begin(const mcp2515_t *dev, mcp2515_bitrate_t bitrate, mcp2515_clock_t clock_mhz);

bool mcp2515_check_receive(const mcp2515_t *dev);
mcp2515_error_t mcp2515_send_message(const mcp2515_t *dev, const can_frame_t *frame);
mcp2515_error_t mcp2515_read_message(const mcp2515_t *dev, can_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif