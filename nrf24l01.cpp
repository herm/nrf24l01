#include "nrf24l01.h"

#include <inttypes.h>

#include "config.h"
#include "spi.h"
#include "delay.h"

// Register Flags



NRF24L01::NRF24L01(SPI &spi_, const DigitalOut &csn_, const DigitalOut &ce_) : spi(spi_), csn(csn_), ce(ce_), config(NRF24L01_CONFIG::PWR_UP | NRF24L01_CONFIG::CRC_1BYTE | NRF24L01_CONFIG::EN_CRC |
       NRF24L01_CONFIG::MASK_MAX_RT | NRF24L01_CONFIG::MASK_RX_DR | NRF24L01_CONFIG::MASK_TX_DS)
{
    csn = 1;
    ce = 0;
    write_reg(NRF24L01_REG::CONFIG, config); //Power up (max. 4ms)
    write_reg(NRF24L01_REG::EN_AA, 0); //Disable auto-ack for now
    write_reg(NRF24L01_REG::EN_RXADDR, 1); //Always enable pipe 0
    write_reg(NRF24L01_REG::SETUP_AW, NRF24L01_AW::address_width(5)); // 5 byte adresses
    set_speed_power(s2M, dBm_0);
    write_reg(NRF24L01_REG::STATUS, NRF24L01_STATUS::MAX_RT);
    //State: Standby I
}

void NRF24L01::write_reg(uint_fast8_t reg_nr, uint_fast8_t data)
{
    csn = 0;
    spi.write(NRF24L01_CMD::W_REGISTER | reg_nr);
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
    spi.write(NRF24L01_CMD::R_REGISTER | reg_nr);
    uint8_t result = spi.write(0);
    csn = 1;
    return result;
}

uint_fast8_t NRF24L01::status()
{
    csn = 0;
    uint_fast8_t result = spi.write(NRF24L01_CMD::NOP);
    csn = 1;
    return result;
}

void NRF24L01::start_receive()
{
    // State: Standby I
    config |= NRF24L01_CONFIG::PRIM_RX;
    write_reg(NRF24L01_REG::CONFIG, config);
    write(NRF24L01_CMD::FLUSH_RX);
    write_reg(NRF24L01_REG::STATUS, 0x70); //Clear all status bits
    ce = 1;
    // State: RX settling (130µs) => RX Mode
}

void NRF24L01::end_receive()
{
    ce = 0;
}


void NRF24L01::send_packet(const uint8_t *data, uint_fast8_t length)
{
    config &= ~NRF24L01_CONFIG::PRIM_RX;
    // State: Standby I or RX Mode
    ce = 0;
    // State: Standby I, unknown config
    write_reg(NRF24L01_REG::CONFIG, config);
    // State: Standby I, TX config
    write_reg(NRF24L01_REG::STATUS, NRF24L01_STATUS::MAX_RT | NRF24L01_STATUS::TX_DS); //Clear MAX_RT bit
    write(NRF24L01_CMD::W_TX_PAYLOAD, length, data);
    ce = 1;
    delay_us(10);
    ce = 0;
    // State: TX Mode followed by Standby I
}

uint_fast8_t NRF24L01::read_payload(uint8_t *buffer, uint8_t *pipe)
{
    //TODO: Read packet length
    //Note: Using the status from RX_PAYLOAD is not possible. Even if it tells you
    // that data is available sometimes the data read will be all zeros.
    uint8_t status_ = status();
    if ((status_ & NRF24L01_STATUS::RX_FIFO_EMPTY) == NRF24L01_STATUS::RX_FIFO_EMPTY) {
        return 0;
    }
    csn = 0;
    spi.write(NRF24L01_CMD::R_RX_PAYLOAD);
    for (unsigned i=0; i<32; i++)
    {
        buffer[i] = spi.write(0);
    }
    csn = 1;
    if (pipe)
    {
        *pipe = NRF24L01_STATUS::get_rx_pipe_number(status_);
    }
    return 32;
}


void NRF24L01::set_channel(uint_fast8_t channel)
{
    write_reg(NRF24L01_REG::RF_CH, channel);
}

void NRF24L01::set_speed_power(NRF24L01::speed_t speed, NRF24L01::power_t power)
{
    write_reg(NRF24L01_REG::RF_SETUP, speed | power);
}

void NRF24L01::set_tx_mac(const uint8_t *mac)
{
    write(NRF24L01_CMD::W_REGISTER | NRF24L01_REG::TX_ADDR, 5, mac);
    /* Also listen on the same address for ACK packets. */
    write(NRF24L01_CMD::W_REGISTER | NRF24L01_REG::RX_ADDR_BASE, 5, mac);
}

void NRF24L01::set_rx_mac(uint_fast8_t pipe, const uint8_t *mac)
{
    write(NRF24L01_CMD::W_REGISTER | (NRF24L01_REG::RX_ADDR_BASE + pipe), pipe <= 1 ? 5 : 1, mac);
}

void NRF24L01::set_payload_length(uint_fast8_t pipe, uint_fast8_t length)
{
    write_reg(NRF24L01_REG::RX_PW_BASE + pipe, length);
}

uint_fast8_t NRF24L01::read_retransmit_counter()
{
    return read_reg(NRF24L01_REG::OBSERVE_TX) & 0b1111;
}

bool NRF24L01::wait_transmit_complete()
{
    //TODO: Timeout?
    while (!(status() & (NRF24L01_STATUS::MAX_RT | NRF24L01_STATUS::TX_DS)));

    bool result = true;
    if (status() & NRF24L01_STATUS::MAX_RT) result = false;
    return result;
}

unsigned NRF24L01::read_power_detector(uint_fast8_t channel)
{
    write_reg(NRF24L01_REG::CONFIG, NRF24L01_CONFIG::PWR_UP | NRF24L01_CONFIG::PRIM_RX);
    set_channel(channel);
    ce = 1;
    delay_us(200); //minimum = 170
    ce = 0;
    return read_reg(NRF24L01_REG::RPD);
}


#ifdef DEBUG
#include <string.h>
#include <debug.h>
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
        spi.write(NRF24L01_CMD::R_REGISTER | info->regnr);
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

void NRF24L01::dump_status()
{
    uint8_t s = status();
    dbg_write_str("--------- STATUS ---------");
    dbg_write_str("Data Ready RX IRQ: ", false);
    dbg_write_bool(s & NRF24L01_STATUS::RX_DR);
    dbg_write_str("Data Sent TX IRQ: ", false);
    dbg_write_bool(s & NRF24L01_STATUS::TX_DS);
    dbg_write_str("Max RT IRQ: ", false);
    dbg_write_bool(s & NRF24L01_STATUS::MAX_RT);
    dbg_write_str("TX fifo full: ", false);
    dbg_write_bool(s & NRF24L01_STATUS::TX_FULL);
    dbg_write_str("Pipe: ", false);
    dbg_write_bool(s & NRF24L01_STATUS::RX_DR);
    dbg_write_str("--------------------------");
}
#endif
