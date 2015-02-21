#include "nrf24l01.h"

#include <inttypes.h>

#include "config.h"
#include "spi.h"
#include "delay.h"
#include "utils.h" //PROGMEM

// Register Flags


#ifndef NRF24L01_STATIC
NRF24L01::NRF24L01(SPI &spi_, const DigitalOut &csn_, const DigitalOut &ce_) : spi(spi_), csn(csn_), ce(ce_)
#ifndef NRF24L01_READBACK_CONFIG
  , config_(NRF24L01_DEFAULT_CONFIG)
#endif
{
    init();
}
#endif

void NRF24L01::init() NRF24L01_STATIC_CONST__
{
    csn1();
    ce0();
#ifndef nrf_enabled_pipes
    #define nrf_enabled_pipes 0b000011
#endif
#if 1
#ifndef PROGMEM
#define PROGMEM
#define pgm_read_byte(x) (*(x))
#endif
    // Pipe 0 is only used for receiving
    static const uint8_t register_values[] PROGMEM =
    {
        NRF24L01_DEFAULT_CONFIG,
        nrf_enabled_pipes, //ENAA
        nrf_enabled_pipes & 0xFE, //EN_RXADDR,
        0b11,     //AW = 5
        0xff,     //15 retransmits, 4ms wait
        nrf_channel,
        s2M|dBm_0,
        NRF24L01_STATUS::MAX_RT | NRF24L01_STATUS::RX_DR | NRF24L01_STATUS::TX_DS //clear flags
    };
    for (unsigned i = 0; i < sizeof(register_values); i++)
    {
        write_reg(i, pgm_read_byte(&(register_values[i])));
    }
#else
    write_reg(NRF24L01_REG::CONFIG, NRF24L01_DEFAULT_CONFIG); //Power up (max. 4ms)
    write_reg(NRF24L01_REG::EN_AA, nrf_enabled_pipes);
    write_reg(NRF24L01_REG::EN_RXADDR, nrf_enabled_pipes & 0xFE);
    write_reg(NRF24L01_REG::SETUP_AW, NRF24L01_AW::address_width(5));
    write_reg(NRF24L01_REG::STATUS, NRF24L01_STATUS::MAX_RT| NRF24L01_STATUS::RX_DR | NRF24L01_STATUS::TX_DS);
    write_reg(NRF24L01_REG::RF_CH, nrf_channel);
    set_speed_power(s2M, dBm_0);
#endif
    write_reg(NRF24L01_REG::FEATURE, NRF24L01_FEATURE::EN_DPL /*| NRF24L01_FEATURE::EN_DYN_ACK | NRF24L01_FEATURE::EN_ACK_PAY */);
    write_reg(NRF24L01_REG::DYNPD, nrf_enabled_pipes);
    //State: Standby I

}

void NRF24L01::write_reg(uint_fast8_t reg_nr, uint_fast8_t data) NRF24L01_STATIC_CONST__
{
    csn0();
    spiwrite(NRF24L01_CMD::W_REGISTER | reg_nr);
    spiwrite(data);
    csn1();
}

void NRF24L01::write(uint_fast8_t command, uint_fast8_t size, const char *data) NRF24L01_STATIC_CONST__
{
    csn0();
    spiwrite(command);
    while (size--) {
        spiwrite(*data++);
    }
    csn1();
}

void NRF24L01::write(uint_fast8_t command) NRF24L01_STATIC_CONST__
{
    csn0();
    spiwrite(command);
    csn1();
}

uint8_t NRF24L01::read_reg(uint_fast8_t reg_nr) NRF24L01_STATIC_CONST__
{
    csn0();
    spiwrite(NRF24L01_CMD::R_REGISTER | reg_nr);
    uint8_t result = spiwrite(0);
    csn1();
    return result;
}

uint_fast8_t NRF24L01::status() NRF24L01_STATIC_CONST__
{
    csn0();
    uint_fast8_t result = spiwrite(NRF24L01_CMD::NOP);
    csn1();
    return result;
}

void NRF24L01::start_receive()
{
    // State: Standby I
    set_config(get_config() | NRF24L01_CONFIG::PRIM_RX);
    write_reg(NRF24L01_REG::EN_RXADDR, nrf_enabled_pipes & 0xFE); //Reenable all pipes, but disable pipe 0 (undo changes from send_packet)
    flush_rx();
    write_reg(NRF24L01_REG::STATUS, NRF24L01_STATUS::MAX_RT | NRF24L01_STATUS::RX_DR | NRF24L01_STATUS::TX_DS); //Clear all status bits
    ce1();
    // State: RX settling (130µs) => RX Mode
}


void NRF24L01::send_packet(const void *data, uint_fast8_t length)
{
    // State: Standby I or RX Mode
    ce0();
    // State: Standby I, unknown config
    set_config(get_config() & ~NRF24L01_CONFIG::PRIM_RX);
    flush_tx();
    write_reg(NRF24L01_REG::EN_RXADDR, 1); //Enable pipe 0  in transmit mode only to avoid receiving packets send to our destination address
    // State: Standby I, TX config
    write_reg(NRF24L01_REG::STATUS, NRF24L01_STATUS::MAX_RT | NRF24L01_STATUS::TX_DS); //Clear MAX_RT bit
    write(NRF24L01_CMD::W_TX_PAYLOAD, length, (const char*)data);
    ce1();
    delay_us(10);
    ce0();
    // State: TX Mode followed by Standby I
}

#ifdef NRF24L01_ONLY_DYN_PLD
uint_fast8_t NRF24L01::read_payload(void *buffer) NRF24L01_STATIC_CONST__
#else
uint_fast8_t NRF24L01::read_payload(void *buffer, uint8_t length) NRF24L01_STATIC_CONST__
#endif
{
    uint8_t *buf = reinterpret_cast<uint8_t *>(buffer);
    //Note: Using the status from RX_PAYLOAD is not possible. Even if it tells you
    // that data is available sometimes the data read will be all zeros.
    // TODO: But reading the status together with R_RX_PL_WID should be possible.
    uint8_t status_ = status();
    if ((status_ & NRF24L01_STATUS::RX_FIFO_EMPTY) == NRF24L01_STATUS::RX_FIFO_EMPTY) {
        return 0;
    }
#ifdef NRF24L01_ONLY_DYN_PLD
    uint8_t length;
    {
#else
    if (length == 0)
    {
#endif
        csn0();
        spiwrite(NRF24L01_CMD::R_RX_PL_WID);
        length = spiwrite(0);
        csn1();
        if (length > 32)
        {
            flush_rx();
            return 0;
        }
    }
    csn0();
    spiwrite(NRF24L01_CMD::R_RX_PAYLOAD);
    for (unsigned i=0; i<length; i++)
    {
        buf[i] = spiwrite(0);
    }
    csn1();
    return length;
}


void NRF24L01::set_channel(uint_fast8_t channel) NRF24L01_STATIC_CONST__
{
    write_reg(NRF24L01_REG::RF_CH, channel);
}

void NRF24L01::set_speed_power(NRF24L01::speed_t speed, NRF24L01::power_t power) NRF24L01_STATIC_CONST__
{
    write_reg(NRF24L01_REG::RF_SETUP, speed | power);
}

void NRF24L01::set_payload_length(uint_fast8_t pipe, uint_fast8_t length) NRF24L01_STATIC_CONST__
{
    write_reg(NRF24L01_REG::RX_PW_BASE + pipe, length);
}

uint_fast8_t NRF24L01::read_retransmit_counter() NRF24L01_STATIC_CONST__
{
    return read_reg(NRF24L01_REG::OBSERVE_TX) & 0b1111;
}

bool NRF24L01::wait_transmit_complete() NRF24L01_STATIC_CONST__
{
    //TODO: Timeout?
    uint8_t status_;
    do
    {
        status_ = status();
    } while (!(status_ & (NRF24L01_STATUS::MAX_RT | NRF24L01_STATUS::TX_DS)));
    return status_ & NRF24L01_STATUS::TX_DS;
}

unsigned NRF24L01::read_power_detector(uint_fast8_t channel) NRF24L01_STATIC_CONST__
{
    write_reg(NRF24L01_REG::CONFIG, NRF24L01_CONFIG::PWR_UP | NRF24L01_CONFIG::PRIM_RX);
    set_channel(channel);
    ce1();
    delay_us(200); //minimum = 170
    ce0();
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
    {"RX_PW0", 0x11, 1},
    {"RX_PW1", 0x12, 1},
    {"FIFO", 0x17, 1},
    {"DYNPD", 0x1C, 1},
    {"FEATURE", 0x1D, 1},
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
        csn0();
        spiwrite(NRF24L01_CMD::R_REGISTER | info->regnr);
        for (int i=0; i<info->length; i++)
        {
            to_hex(pos, spiwrite(0));
            pos += 2;
            *pos++ = ' ';
        }
        csn1();
        *pos=0;
        dbg_write_str(buffer);
        info++;
    }
}

void NRF24L01::dump_status()
{
    uint8_t s = status();
    dbg_write_str("--------- STATUS ---------");
    dbg_write_str("Config:", false);
    dbg_write_u8(read_reg(NRF24L01_REG::CONFIG));
    dbg_write_str("Data Ready RX IRQ: ", false);
    dbg_write_bool(s & NRF24L01_STATUS::RX_DR);
    dbg_write_str("Data Sent TX IRQ: ", false);
    dbg_write_bool(s & NRF24L01_STATUS::TX_DS);
    dbg_write_str("Max RT IRQ: ", false);
    dbg_write_bool(s & NRF24L01_STATUS::MAX_RT);
    dbg_write_str("TX fifo full: ", false);
    dbg_write_bool(s & NRF24L01_STATUS::TX_FULL);
    dbg_write_str("Pipe: ", false);
    dbg_write_u8((s & NRF24L01_STATUS::RX_P_NO) >> 1);
    dbg_write_str("--------------------------");
}
#endif
