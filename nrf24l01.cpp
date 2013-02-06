#include "../lpc2103.h"
#include "config.h"
#include <inttypes.h>
#include "nrf24l01.hpp"
#include "spi.hpp"
#include "system/delay.h"

/* Black NRF24L01+ module from Ebay:
 *
 *   GND (marked)   1   2   VDD
 *             CE   3   4   CSN
 *            SCK   5   6   MOSI
 *           MISO   7   8   IRQ
 */

// Register Flags

//CONFIG register definitions
#define R_CONFIG_MASK_RX_DR      0x40
#define R_CONFIG_MASK_TX_DS      0x20
#define R_CONFIG_MASK_MAX_RT     0x10
#define R_CONFIG_EN_CRC          0x08
#define R_CONFIG_CRCO            0x04
#define R_CONFIG_CRC_1BYTE       (R_CONFIG_EN_CRC)
#define R_CONFIG_CRC_2BYTE       (R_CONFIG_EN_CRC|R_CONFIG_CRC0)
#define R_CONFIG_CRC_OFF         0
#define R_CONFIG_PWR_UP          0x02
#define R_CONFIG_PRIM_RX         0x01


//RF_SETUP register definitions
#define R_RF_SETUP_CONT_WAVE     0x80
#define R_RF_SETUP_PLL_LOCK      0x10

//SETUP_AW register definitions
#define R_SETUP_AW_3             0x01
#define R_SETUP_AW_4             0x02
#define R_SETUP_AW_5             0x03

//STATUS register definitions
#define R_STATUS_RX_DR           0x40
#define R_STATUS_TX_DS           0x20
#define R_STATUS_MAX_RT          0x10
#define R_STATUS_RX_P_NO         0x0E
#define R_STATUS_GET_RX_P_NO(x)  ((x&R_STATUS_RX_P_NO)>>1)
#define R_STATUS_RX_FIFO_EMPTY   0x0E
#define R_STATUS_TX_FULL         0x01

NRF24L01::NRF24L01(SPI &spi_, const DigitalOut &csn_, const DigitalOut &ce_) : spi(spi_), csn(csn_), ce(ce_), config(R_CONFIG_PWR_UP | R_CONFIG_CRC_1BYTE)
{
    csn = 1;
    ce = 0;
    write_reg(R_CONFIG, config); //Power up (max. 4ms)
    write_reg(R_EN_AA, 0); //Disable auto-ack for now
    write_reg(R_SETUP_AW, R_SETUP_AW_5); // 5 byte adresses
    set_speed_power(s2M, dBm_0);
    write_reg(R_STATUS, R_STATUS_MAX_RT);
    //State: Standby I
}

void NRF24L01::write_reg(uint_fast8_t reg_nr, uint_fast8_t data)
{
    csn = 0;
    spi.write(C_W_REGISTER | reg_nr);
    spi.write(data);
    csn = 1;
}

void NRF24L01::write(uint_fast8_t command, uint_fast8_t size, const uint8_t *data)
{
    csn = 0;
    spi.write(command);
    while (size--) {
        spi.write(*data++);
    }
    csn = 1;
}

void NRF24L01::write(uint_fast8_t command)
{
    csn = 0;
    spi.write(command);
    csn = 1;
}

uint8_t NRF24L01::read_reg(uint_fast8_t reg_nr)
{
    csn = 0;
    spi.write(C_R_REGISTER | reg_nr);
    uint8_t result = spi.write(0);
    csn = 1;
    return result;
}

uint_fast8_t NRF24L01::status()
{
    csn = 0;
    uint_fast8_t result = spi.write(C_NOP);
    csn = 1;
    return result;
}

void NRF24L01::start_receive()
{
    // State: Standby I
    write_reg(R_CONFIG, config | R_CONFIG_PRIM_RX);
    write(C_FLUSH_RX);
    write_reg(R_STATUS, 0x70); //Clear all status bits
    ce = 1;
    // State: RX settling (130µs) => RX Mode
}

void NRF24L01::end_receive()
{
    ce = 0;
}

void NRF24L01::send_packet(const uint8_t *data, uint_fast8_t length)
{
    // State: Standby I or RX Mode
    ce = 0;
    // State: Standby I, unknown config
    write_reg(R_CONFIG, config);
    // State: Standby I, TX config
    write_reg(R_STATUS, R_STATUS_MAX_RT | R_STATUS_TX_DS); //Clear MAX_RT bit
    write(C_W_TX_PAYLOAD, length, data);
    ce = 1;
    delay_us(10);
    ce = 0;
    // State: TX Mode followed by Standby I
}

uint_fast8_t NRF24L01::read_payload(uint8_t *buffer, uint8_t *pipe)
{
    //TODO: Read packet length
    csn = 0;
    uint8_t status_ = spi.write(C_R_RX_PAYLOAD);
    if ((status_ & R_STATUS_RX_FIFO_EMPTY) == R_STATUS_RX_FIFO_EMPTY) {
        csn = 1; //Abort transfer
        return 0;
    }
    for (unsigned i=0; i<32; i++)
    {
        buffer[i] = spi.write(0);
    }
    csn = 1;
    *pipe = R_STATUS_GET_RX_P_NO(status_);
    return 32;
}


void NRF24L01::set_channel(uint_fast8_t channel)
{
    write_reg(R_RF_CH, channel);
}

void NRF24L01::set_speed_power(NRF24L01::speed_t speed, NRF24L01::power_t power)
{
    write_reg(R_RF_SETUP, speed | power);
}

void NRF24L01::set_tx_mac(const uint8_t *mac)
{
    write(C_W_REGISTER | R_TX_ADDR, 5, mac);
    /* Also listen on the same address for ACK packets. */
    write(C_W_REGISTER | R_RX_ADDR_BASE, 5, mac);
}

void NRF24L01::set_rx_mac(uint_fast8_t pipe, const uint8_t *mac)
{
    write(C_W_REGISTER | (R_RX_ADDR_BASE + pipe), pipe <= 1 ? 5 : 1, mac);
}

void NRF24L01::set_payload_length(uint_fast8_t pipe, uint_fast8_t length)
{
    write_reg(R_RX_PW_BASE + pipe, length);
}

uint_fast8_t NRF24L01::read_retransmit_counter()
{
    return read_reg(R_OBSERVE_TX) & 0b1111;
}

bool NRF24L01::wait_transmit_complete()
{
    //TODO: Timeout?
    while (!(status() & (R_STATUS_MAX_RT | R_STATUS_TX_DS)));

    bool result = true;
    if (status() & R_STATUS_MAX_RT) result = false;
    return result;
}

unsigned NRF24L01::read_power_detector(uint_fast8_t channel)
{
    write_reg(R_CONFIG, R_CONFIG_PWR_UP | R_CONFIG_PRIM_RX);
    set_channel(channel);
    ce = 1;
    delay_us(200); //minimum = 170
    ce = 0;
    return read_reg(R_RPD);
}


#define DEBUG
#ifdef DEBUG
#include <string.h>
#include <libdcc/dcc_stdio.h>
typedef struct
{
    const char *name;
    uint8_t regnr;
    uint8_t length;
} reg_info_t;
static const reg_info_t registers[] = {
    {"CONFIG", 0x00, 1},
    {"ENAA", 0x01, 1},
    {"EN_RXADDR", 0x02, 1},
    {"SETUP_AW", 0x03, 1},
    {"SETUP_RETR", 0x04, 1},
    {"RF_CH", 0x05, 1},
    {"RF_SETUP", 0x06, 1},
    {"STATUS", 0x07, 1},
    {"RX_ADDR0", 0x0A, 5},
    {"RX_ADDR1", 0x0B, 5},
    {"TX_ADDR", 0x10, 5},
    {"RW_PW0", 0x11, 1},
    {"RW_PW1", 0x12, 1},
    {"DYNPD", 0x1C, 1},
    {0, 0, 0}
};

static inline void to_hex(char *buffer, uint8_t value)
{
    uint8_t tmp = value >> 4;
    if (tmp > 9) {
        *buffer++ = tmp - 10 + 'A';
    } else {
        *buffer++ = tmp + '0';
    }
    tmp = value & 0x0f;
    if (tmp > 9) {
        *buffer++ = tmp - 10 + 'A';
    } else {
        *buffer++ = tmp + '0';
    }
}


void NRF24L01::dump_registers()
{
    const reg_info_t *info = registers;
    char buffer[50];
    char *pos;
    while (info->name)
    {
        strcpy(buffer, info->name);
        pos = buffer + strlen(info->name);
        *pos++ = ' ';
        csn = 0;
        spi.write(C_R_REGISTER | info->regnr);
        for (int i=0; i<info->length; i++)
        {
            to_hex(pos, spi.write(0));
            pos += 2;
            *pos++ = ' ';
        }
        csn = 1;
        *pos=0;
        dbg_write_str(buffer);
        info++;
    }
}
#endif
