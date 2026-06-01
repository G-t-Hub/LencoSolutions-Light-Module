#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>

#define BOOTLOADER_VERSION_MAJOR 3
#define BOOTLOADER_VERSION_MINOR 0

#define NODE_CAN_ID 36UL
#define OTA_PACKET_TYPE 241UL
#define OTA_STATUS_PACKET_TYPE 242UL
#define OTA_CMD_EID8 0xF1
#define OTA_STATUS_EID8 0xF2
#define OTA_EID0 0x24

#define EEPROM_OTA_REQUEST_ADDR ((uint8_t*)87)
#define EEPROM_APP_VALID_ADDR ((uint8_t*)88)
#define OTA_REQUEST_MAGIC 0xA5
#define APP_VALID_MAGIC 0x5A
#define APP_INVALID_MAGIC 0x00

#define MCP_CS_DDR DDRB
#define MCP_CS_PORT PORTB
#define MCP_CS_PIN PB2
#define SPI_DDR DDRB
#define SPI_MOSI PB3
#define SPI_MISO PB4
#define SPI_SCK PB5

#define MCP_RESET 0xC0
#define MCP_READ 0x03
#define MCP_WRITE 0x02
#define MCP_BITMOD 0x05
#define MCP_READ_STATUS 0xA0
#define MCP_LOAD_TX0 0x40
#define MCP_RTS_TX0 0x81
#define MCP_READ_RX0 0x90
#define MCP_READ_RX1 0x94

#define MCP_CANCTRL 0x0F
#define MCP_CANINTF 0x2C
#define MCP_CNF3 0x28
#define MCP_CNF2 0x29
#define MCP_CNF1 0x2A
#define MCP_RXF0SIDH 0x00
#define MCP_RXF1SIDH 0x04
#define MCP_RXF2SIDH 0x08
#define MCP_RXF3SIDH 0x10
#define MCP_RXF4SIDH 0x14
#define MCP_RXF5SIDH 0x18
#define MCP_RXM0SIDH 0x20
#define MCP_RXM1SIDH 0x24
#define MCP_RXB0CTRL 0x60
#define MCP_RXB1CTRL 0x70

#define CANINTF_RX0IF 0x01
#define CANINTF_RX1IF 0x02

#define SUB_META 1
#define SUB_DATA 2
#define SUB_COMMIT 3

#define ST_READY 1
#define ST_META_OK 2
#define ST_DATA 3
#define ST_CRC_OK 4
#define ST_DONE 5
#define ST_ERROR 0xEE
#define SEND_CAN_STATUS 1

#define APP_END 0x7800U
#define PAGE_SIZE_BYTES SPM_PAGESIZE
#define DATA_BYTES_PER_FRAME 5
#define START_TIMEOUT_LOOPS 500
#define UPDATE_TIMEOUT_LOOPS 60000UL

typedef void (*app_start_t)(void);

static uint8_t page_buf[PAGE_SIZE_BYTES];
static uint16_t expected_size;
static uint32_t expected_crc;
static uint16_t received_size;
static uint32_t running_crc;
static uint16_t expected_seq;
static uint8_t flash_touched;

static void cs_low(void) { MCP_CS_PORT &= ~(1 << MCP_CS_PIN); }
static void cs_high(void) { MCP_CS_PORT |= (1 << MCP_CS_PIN); }

static uint8_t spi_tx(uint8_t value) {
    SPDR = value;
    while (!(SPSR & (1 << SPIF))) {}
    return SPDR;
}

static void spi_init(void) {
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << MCP_CS_PIN);
    SPI_DDR &= ~(1 << SPI_MISO);
    cs_high();
    SPCR = (1 << SPE) | (1 << MSTR);
    SPSR = (1 << SPI2X);
}

static void mcp_write(uint8_t reg, uint8_t value) {
    cs_low();
    spi_tx(MCP_WRITE);
    spi_tx(reg);
    spi_tx(value);
    cs_high();
}

static void mcp_bitmod(uint8_t reg, uint8_t mask, uint8_t value) {
    cs_low();
    spi_tx(MCP_BITMOD);
    spi_tx(reg);
    spi_tx(mask);
    spi_tx(value);
    cs_high();
}

static void mcp_reset_chip(void) {
    cs_low();
    spi_tx(MCP_RESET);
    cs_high();
    _delay_ms(10);
}

static void mcp_write_id(uint8_t reg, uint8_t sidl, uint8_t eid8, uint8_t eid0) {
    mcp_write(reg, 0);
    mcp_write(reg + 1, sidl);
    mcp_write(reg + 2, eid8);
    mcp_write(reg + 3, eid0);
}

static void mcp_init(void) {
    spi_init();
    mcp_reset_chip();
    mcp_write(MCP_CNF1, 0x00);
    mcp_write(MCP_CNF2, 0x90);
    mcp_write(MCP_CNF3, 0x82);
    mcp_write_id(MCP_RXF0SIDH, 0x08, OTA_CMD_EID8, OTA_EID0);
    mcp_write_id(MCP_RXF1SIDH, 0x08, OTA_CMD_EID8, OTA_EID0);
    mcp_write_id(MCP_RXM0SIDH, 0x03, 0xFF, 0xFF);
    mcp_write_id(MCP_RXM1SIDH, 0x03, 0xFF, 0xFF);
    mcp_write(MCP_RXB0CTRL, 0x04);
    mcp_write(MCP_RXB1CTRL, 0x00);
    mcp_write(MCP_CANCTRL, 0x00);
    _delay_ms(10);
}

static void can_send_status(uint8_t status, uint16_t seq, uint8_t detail) {
#if SEND_CAN_STATUS
    cs_low();
    spi_tx(MCP_LOAD_TX0);
    spi_tx(0);
    spi_tx(0x08);
    spi_tx(OTA_STATUS_EID8);
    spi_tx(OTA_EID0);
    spi_tx(5);
    spi_tx(222);
    spi_tx(status);
    spi_tx((uint8_t)(seq >> 8));
    spi_tx((uint8_t)seq);
    spi_tx(detail ? detail : ((BOOTLOADER_VERSION_MAJOR << 4) | BOOTLOADER_VERSION_MINOR));
    cs_high();
    cs_low();
    spi_tx(MCP_RTS_TX0);
    cs_high();
#else
    (void)status;
    (void)seq;
    (void)detail;
#endif
}

static uint8_t can_read(uint8_t *data, uint8_t *len) {
    uint8_t status;
    uint8_t read_cmd;
    uint8_t int_mask;
    uint8_t header[5];
    cs_low();
    spi_tx(MCP_READ_STATUS);
    status = spi_tx(0);
    cs_high();

    if (status & 0x01) {
        read_cmd = MCP_READ_RX0;
        int_mask = CANINTF_RX0IF;
    } else if (status & 0x02) {
        read_cmd = MCP_READ_RX1;
        int_mask = CANINTF_RX1IF;
    } else {
        return 0;
    }

    cs_low();
    spi_tx(read_cmd);
    for (uint8_t i = 0; i < sizeof(header); i++) {
        header[i] = spi_tx(0);
    }
    *len = header[4] & 0x0F;
    if (*len > 8) *len = 8;
    for (uint8_t i = 0; i < *len; i++) {
        data[i] = spi_tx(0);
    }
    cs_high();
    mcp_bitmod(MCP_CANINTF, int_mask, 0);

    return header[0] == 0 && header[1] == 0x08 &&
           header[2] == OTA_CMD_EID8 && header[3] == OTA_EID0;
}

static uint32_t crc32_update(uint32_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1)));
    }
    return crc;
}

static void app_jump(void) {
    cli();
    wdt_disable();
    SPCR = 0;
    app_start_t app = (app_start_t)0x0000;
    app();
}

static void page_reset(void) {
    memset(page_buf, 0xFF, sizeof(page_buf));
}

static void page_write(uint32_t page_addr) {
    if (!flash_touched) {
        eeprom_update_byte(EEPROM_APP_VALID_ADDR, APP_INVALID_MAGIC);
        eeprom_busy_wait();
        flash_touched = 1;
    }
    boot_page_erase(page_addr);
    boot_spm_busy_wait();
    for (uint16_t i = 0; i < PAGE_SIZE_BYTES; i += 2) {
        uint16_t w = page_buf[i] | ((uint16_t)page_buf[i + 1] << 8);
        boot_page_fill(page_addr + i, w);
    }
    boot_page_write(page_addr);
    boot_spm_busy_wait();
    boot_rww_enable();
}

static uint8_t add_firmware_byte(uint8_t value) {
    if (received_size >= expected_size || received_size >= APP_END) {
        return 0;
    }
    uint16_t page_offset = received_size % PAGE_SIZE_BYTES;
    uint32_t page_addr = received_size - page_offset;
    page_buf[page_offset] = value;
    running_crc = crc32_update(running_crc, value);
    received_size++;
    if (page_offset == PAGE_SIZE_BYTES - 1) {
        page_write(page_addr);
        page_reset();
    }
    return 1;
}

static void begin_update(const uint8_t *data) {
    expected_size = ((uint16_t)data[2] << 8) | data[3];
    if (data[1] != 0) {
        expected_size = 0;
    }
    expected_crc = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | data[7];
    received_size = 0;
    running_crc = 0xFFFFFFFFUL;
    expected_seq = 0;
    flash_touched = 0;
    page_reset();
    if (expected_size == 0 || expected_size > APP_END) {
        can_send_status(ST_ERROR, 0, 1);
        expected_size = 0;
        return;
    }
    can_send_status(ST_META_OK, 0, 0);
}

static uint8_t handle_data(const uint8_t *data, uint8_t len) {
    if (len != 8 || expected_size == 0) return 0;
    uint16_t seq = ((uint16_t)data[1] << 8) | data[2];
    if (seq != expected_seq) {
        can_send_status(ST_ERROR, expected_seq, 2);
        return 0;
    }
    for (uint8_t i = 0; i < DATA_BYTES_PER_FRAME && received_size < expected_size; i++) {
        uint8_t byte_index = i + 3;
        if (!add_firmware_byte(data[byte_index])) {
            can_send_status(ST_ERROR, seq, 3);
            return 0;
        }
    }
    expected_seq++;
    return 1;
}

static void commit_update(void) {
    if (expected_size == 0 || received_size != expected_size) {
        can_send_status(ST_ERROR, expected_seq, 4);
        return;
    }
    uint16_t page_offset = received_size % PAGE_SIZE_BYTES;
    if (page_offset != 0) {
        page_write(received_size - page_offset);
    }
    running_crc ^= 0xFFFFFFFFUL;
    if (running_crc != expected_crc) {
        can_send_status(ST_ERROR, expected_seq, 5);
        return;
    }
    eeprom_update_byte(EEPROM_APP_VALID_ADDR, APP_VALID_MAGIC);
    eeprom_update_byte(EEPROM_OTA_REQUEST_ADDR, 0);
    can_send_status(ST_CRC_OK, expected_seq, 0);
    _delay_ms(20);
    can_send_status(ST_DONE, expected_seq, 0);
    _delay_ms(100);
    app_jump();
}

int main(void) {
    MCUSR = 0;
    wdt_disable();
    cli();
    mcp_init();

    uint8_t requested = eeprom_read_byte(EEPROM_OTA_REQUEST_ADDR) == OTA_REQUEST_MAGIC;
    uint8_t app_invalid = eeprom_read_byte(EEPROM_APP_VALID_ADDR) == APP_INVALID_MAGIC;
    uint8_t data[8];
    uint8_t len = 0;

    if (!requested && !app_invalid) {
        for (uint16_t i = 0; i < START_TIMEOUT_LOOPS; i++) {
            if (can_read(data, &len) && len > 0 && data[0] == SUB_META) {
                requested = 1;
                break;
            }
            _delay_ms(1);
        }
    }

    if (!requested && !app_invalid) {
        app_jump();
    }

    eeprom_update_byte(EEPROM_OTA_REQUEST_ADDR, 0);

    uint32_t idle = 0;
    while (1) {
        if (can_read(data, &len) && len > 0) {
            idle = 0;
            if (data[0] == SUB_META && len == 8) {
                begin_update(data);
            } else if (data[0] == SUB_DATA) {
                handle_data(data, len);
            } else if (data[0] == SUB_COMMIT) {
                commit_update();
            }
        } else {
            _delay_ms(1);
            if (++idle > UPDATE_TIMEOUT_LOOPS) {
                can_send_status(ST_ERROR, expected_seq, 6);
                if (!flash_touched) {
                    eeprom_update_byte(EEPROM_APP_VALID_ADDR, APP_VALID_MAGIC);
                    app_jump();
                }
                idle = 0;
            }
        }
    }
}
