#include "mcp2515.h"

#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "pico/time.h"

static inline void cs_select(const mcp2515_t *dev) {
    gpio_put(dev->cs_pin, 0);
    sleep_us(2);
}

static inline void cs_deselect(const mcp2515_t *dev) {
    gpio_put(dev->cs_pin, 1);
    sleep_us(2);
}

static uint8_t spi_transfer_byte(const mcp2515_t *dev, uint8_t data) {
    uint8_t rx = 0;
    spi_write_read_blocking(dev->spi, &data, &rx, 1);
    return rx;
}

void mcp2515_init_struct(mcp2515_t *dev,
                         spi_inst_t *spi,
                         uint sck_pin,
                         uint mosi_pin,
                         uint miso_pin,
                         uint cs_pin,
                         uint int_pin,
                         uint32_t spi_baudrate) {
    dev->spi = spi;
    dev->sck_pin = sck_pin;
    dev->mosi_pin = mosi_pin;
    dev->miso_pin = miso_pin;
    dev->cs_pin = cs_pin;
    dev->int_pin = int_pin;
    dev->spi_baudrate = spi_baudrate;
}

void mcp2515_spi_hw_init(const mcp2515_t *dev) {
    spi_init(dev->spi, dev->spi_baudrate);
    gpio_set_function(dev->sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(dev->mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(dev->miso_pin, GPIO_FUNC_SPI);

    gpio_init(dev->cs_pin);
    gpio_set_dir(dev->cs_pin, GPIO_OUT);
    gpio_put(dev->cs_pin, 1);

    if (dev->int_pin != (uint)-1) {
        gpio_init(dev->int_pin);
        gpio_set_dir(dev->int_pin, GPIO_IN);
        gpio_pull_up(dev->int_pin);
    }
}

void mcp2515_reset(const mcp2515_t *dev) {
    cs_select(dev);
    spi_transfer_byte(dev, MCP2515_INSTR_RESET);
    cs_deselect(dev);
    sleep_ms(10);
}

uint8_t mcp2515_read_register(const mcp2515_t *dev, uint8_t reg) {
    cs_select(dev);
    spi_transfer_byte(dev, MCP2515_INSTR_READ);
    spi_transfer_byte(dev, reg);
    uint8_t value = spi_transfer_byte(dev, 0x00);
    cs_deselect(dev);
    return value;
}

void mcp2515_read_registers(const mcp2515_t *dev, uint8_t reg, uint8_t *buf, uint8_t len) {
    cs_select(dev);
    spi_transfer_byte(dev, MCP2515_INSTR_READ);
    spi_transfer_byte(dev, reg);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spi_transfer_byte(dev, 0x00);
    }
    cs_deselect(dev);
}

void mcp2515_write_register(const mcp2515_t *dev, uint8_t reg, uint8_t value) {
    cs_select(dev);
    spi_transfer_byte(dev, MCP2515_INSTR_WRITE);
    spi_transfer_byte(dev, reg);
    spi_transfer_byte(dev, value);
    cs_deselect(dev);
}

void mcp2515_write_registers(const mcp2515_t *dev, uint8_t reg, const uint8_t *buf, uint8_t len) {
    cs_select(dev);
    spi_transfer_byte(dev, MCP2515_INSTR_WRITE);
    spi_transfer_byte(dev, reg);
    for (uint8_t i = 0; i < len; i++) {
        spi_transfer_byte(dev, buf[i]);
    }
    cs_deselect(dev);
}

void mcp2515_bit_modify(const mcp2515_t *dev, uint8_t reg, uint8_t mask, uint8_t data) {
    cs_select(dev);
    spi_transfer_byte(dev, MCP2515_INSTR_BIT_MODIFY);
    spi_transfer_byte(dev, reg);
    spi_transfer_byte(dev, mask);
    spi_transfer_byte(dev, data);
    cs_deselect(dev);
}

uint8_t mcp2515_read_status(const mcp2515_t *dev) {
    cs_select(dev);
    spi_transfer_byte(dev, MCP2515_INSTR_READ_STATUS);
    uint8_t status = spi_transfer_byte(dev, 0x00);
    cs_deselect(dev);
    return status;
}

static void mcp2515_prepare_standard_id(uint16_t id, uint8_t out[4]) {
    out[MCP_SIDH_INDEX] = (uint8_t)(id >> 3);
    out[MCP_SIDL_INDEX] = (uint8_t)((id & 0x07) << 5);
    out[MCP_EID8_INDEX] = 0;
    out[MCP_EID0_INDEX] = 0;
}

mcp2515_error_t mcp2515_set_mode(const mcp2515_t *dev, uint8_t mode) {
    mcp2515_bit_modify(dev, MCP_CANCTRL, CANCTRL_REQOP_MASK, mode);

    absolute_time_t until = make_timeout_time_ms(10);
    while (!time_reached(until)) {
        uint8_t canstat = mcp2515_read_register(dev, MCP_CANSTAT);
        if ((canstat & CANSTAT_OPMOD_MASK) == mode) {
            return MCP2515_OK;
        }
    }
    return MCP2515_FAIL;
}

mcp2515_error_t mcp2515_set_bitrate(const mcp2515_t *dev, mcp2515_bitrate_t bitrate, mcp2515_clock_t clock_mhz) {
    if (mcp2515_set_mode(dev, CANCTRL_MODE_CONFIG) != MCP2515_OK) {
        return MCP2515_FAIL;
    }

    uint8_t cfg1 = 0, cfg2 = 0, cfg3 = 0;
    bool supported = true;

    switch (clock_mhz) {
        case MCP2515_CLOCK_8MHZ:
            switch (bitrate) {
                case MCP2515_BITRATE_125KBPS: cfg1 = 0x01; cfg2 = 0xB1; cfg3 = 0x85; break;
                case MCP2515_BITRATE_250KBPS: cfg1 = 0x00; cfg2 = 0xB1; cfg3 = 0x85; break;
                case MCP2515_BITRATE_500KBPS: cfg1 = 0x00; cfg2 = 0x90; cfg3 = 0x82; break;
                case MCP2515_BITRATE_1000KBPS: cfg1 = 0x00; cfg2 = 0x80; cfg3 = 0x80; break;
                default: supported = false; break;
            }
            break;
        case MCP2515_CLOCK_16MHZ:
            switch (bitrate) {
                case MCP2515_BITRATE_125KBPS: cfg1 = 0x03; cfg2 = 0xF0; cfg3 = 0x86; break;
                case MCP2515_BITRATE_250KBPS: cfg1 = 0x41; cfg2 = 0xF1; cfg3 = 0x85; break;
                case MCP2515_BITRATE_500KBPS: cfg1 = 0x00; cfg2 = 0xF0; cfg3 = 0x86; break;
                case MCP2515_BITRATE_1000KBPS: cfg1 = 0x00; cfg2 = 0xD0; cfg3 = 0x82; break;
                default: supported = false; break;
            }
            break;
        default:
            supported = false;
            break;
    }

    if (!supported) {
        return MCP2515_FAIL;
    }

    mcp2515_write_register(dev, MCP_CNF1, cfg1);
    mcp2515_write_register(dev, MCP_CNF2, cfg2);
    mcp2515_write_register(dev, MCP_CNF3, cfg3);
    return MCP2515_OK;
}

mcp2515_error_t mcp2515_begin(const mcp2515_t *dev, mcp2515_bitrate_t bitrate, mcp2515_clock_t clock_mhz) {
    mcp2515_spi_hw_init(dev);
    mcp2515_reset(dev);

    // Clear TX control blocks.
    for (uint8_t reg = MCP_TXB0CTRL; reg <= MCP_TXB0CTRL + 13; reg++) mcp2515_write_register(dev, reg, 0x00);
    for (uint8_t reg = MCP_TXB1CTRL; reg <= MCP_TXB1CTRL + 13; reg++) mcp2515_write_register(dev, reg, 0x00);
    for (uint8_t reg = MCP_TXB2CTRL; reg <= MCP_TXB2CTRL + 13; reg++) mcp2515_write_register(dev, reg, 0x00);

    mcp2515_write_register(dev, MCP_RXB0CTRL, 0x00);
    mcp2515_write_register(dev, MCP_RXB1CTRL, 0x00);

    if (mcp2515_set_bitrate(dev, bitrate, clock_mhz) != MCP2515_OK) {
        return MCP2515_FAIL_INIT;
    }

    // Enable RX0, RX1, ERR, MERR interrupts.
    mcp2515_write_register(dev, MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // Receive both standard and extended frames into both RX buffers.
    mcp2515_bit_modify(dev, MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT, RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT);
    mcp2515_bit_modify(dev, MCP_RXB1CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_STDEXT);

    // Clear masks and filters so everything passes.
    const uint8_t zero_filter[4] = {0, 0, 0, 0};
    mcp2515_write_registers(dev, MCP_RXF0SIDH, zero_filter, 4);
    mcp2515_write_registers(dev, MCP_RXF1SIDH, zero_filter, 4);
    mcp2515_write_registers(dev, MCP_RXF2SIDH, zero_filter, 4);
    mcp2515_write_registers(dev, MCP_RXF3SIDH, zero_filter, 4);
    mcp2515_write_registers(dev, MCP_RXF4SIDH, zero_filter, 4);
    mcp2515_write_registers(dev, MCP_RXF5SIDH, zero_filter, 4);
    mcp2515_write_registers(dev, MCP_RXM0SIDH, zero_filter, 4);
    mcp2515_write_registers(dev, MCP_RXM1SIDH, zero_filter, 4);

    if (mcp2515_set_mode(dev, CANCTRL_MODE_NORMAL) != MCP2515_OK) {
        return MCP2515_FAIL_INIT;
    }

    return MCP2515_OK;
}

bool mcp2515_check_receive(const mcp2515_t *dev) {
    return (mcp2515_read_status(dev) & STAT_RXIF_MASK) != 0;
}

mcp2515_error_t mcp2515_send_message(const mcp2515_t *dev, const can_frame_t *frame) {
    if (frame->extended || frame->rtr || frame->dlc > MCP2515_MAX_DATA_LEN || frame->id > 0x7FF) {
        return MCP2515_FAIL_TX;
    }

    const uint8_t tx_ctrl_regs[3] = {MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL};
    const uint8_t tx_sidh_regs[3] = {MCP_TXB0SIDH, MCP_TXB1SIDH, MCP_TXB2SIDH};

    for (int i = 0; i < 3; i++) {
        uint8_t ctrl = mcp2515_read_register(dev, tx_ctrl_regs[i]);
        if ((ctrl & TXB_TXREQ) != 0) {
            continue;
        }

        uint8_t buf[5 + MCP2515_MAX_DATA_LEN] = {0};
        mcp2515_prepare_standard_id((uint16_t)frame->id, buf);
        buf[MCP_DLC_INDEX] = frame->dlc & DLC_MASK;
        memcpy(&buf[MCP_DATA_INDEX], frame->data, frame->dlc);

        mcp2515_write_registers(dev, tx_sidh_regs[i], buf, (uint8_t)(5 + frame->dlc));
        mcp2515_bit_modify(dev, tx_ctrl_regs[i], TXB_TXREQ, TXB_TXREQ);

        sleep_ms(1);
        ctrl = mcp2515_read_register(dev, tx_ctrl_regs[i]);
        if (ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) {
            return MCP2515_FAIL_TX;
        }
        return MCP2515_OK;
    }

    return MCP2515_ALL_TX_BUSY;
}

mcp2515_error_t mcp2515_read_message(const mcp2515_t *dev, can_frame_t *frame) {
    uint8_t status = mcp2515_read_status(dev);
    uint8_t base = 0;
    uint8_t clear_mask = 0;

    if (status & STAT_RX0IF) {
        base = MCP_RXB0SIDH;
        clear_mask = CANINTF_RX0IF;
    } else if (status & STAT_RX1IF) {
        base = MCP_RXB1SIDH;
        clear_mask = CANINTF_RX1IF;
    } else {
        return MCP2515_NO_MSG;
    }

    uint8_t hdr[5] = {0};
    mcp2515_read_registers(dev, base, hdr, 5);

    frame->extended = (hdr[MCP_SIDL_INDEX] & EXIDE_MASK) != 0;
    frame->rtr = (hdr[MCP_DLC_INDEX] & RTR_MASK) != 0;

    if (frame->extended) {
        return MCP2515_FAIL; // this sample keeps things standard-ID only
    }

    frame->id = (uint32_t)((hdr[MCP_SIDH_INDEX] << 3) | (hdr[MCP_SIDL_INDEX] >> 5));
    frame->dlc = hdr[MCP_DLC_INDEX] & DLC_MASK;
    if (frame->dlc > MCP2515_MAX_DATA_LEN) {
        return MCP2515_FAIL;
    }

    memset(frame->data, 0, sizeof(frame->data));
    if (frame->dlc > 0) {
        mcp2515_read_registers(dev, base + 5, frame->data, frame->dlc);
    }

    mcp2515_bit_modify(dev, MCP_CANINTF, clear_mask, 0x00);
    return MCP2515_OK;
}