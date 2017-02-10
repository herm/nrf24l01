#ifndef NRF24L01_HPP
#define NRF24L01_HPP

/* Limitations: Only dynamic payload length is supported.
 * Possible defines:
 * NRF24L01_STATIC: Make all functions static to reduce code size. Implies that the SPI module is static as well.
 * NRF24L01_PORT_CE:  Required for NRF24L01_STATIC
 * NRF24L01_PIN_CE:   Required for NRF24L01_STATIC
 * NRF24L01_PORT_CSN: Required for NRF24L01_STATIC
 * NRF24L01_PIN_CSN:  Required for NRF24L01_STATIC
 * NRF24L01_READBACK_CONFIG: Don't store a copy of the current config in ram.
 * NRF24L01_STATIC_CONFIG: Smaller variant compared to READBACK_CONFIG, but doesn't allow enabling or disabling IRQs at runtime.
 * NRF24L01_DEFAULT_CONFIG: Default config options
 */

#include <inttypes.h>
#include "config.h"
#include "pin.h"
#include "spi.h"

namespace NRF24L01_CMD
{
enum {
R_REGISTER              = 0x00,
W_REGISTER              = 0x20,
R_RX_PAYLOAD            = 0x61,
W_TX_PAYLOAD            = 0xA0,
FLUSH_TX                = 0xE1,
FLUSH_RX                = 0xE2,
REUSE_TX_PL             = 0xE3,
R_RX_PL_WID             = 0x60,
W_ACK_PAYLOAD           = 0xA8,
W_TX_PAYLOAD_NOCACK     = 0xB0,
NOP                     = 0xFF
};
} //NRF24L01_CMD

namespace NRF24L01_REG
{
enum{
CONFIG        = 0x00,
EN_AA         = 0x01,
EN_RXADDR     = 0x02,
SETUP_AW      = 0x03,
SETUP_RETR    = 0x04,
RF_CH         = 0x05,
RF_SETUP      = 0x06,
STATUS        = 0x07,
OBSERVE_TX    = 0x08,
RPD           = 0x09,
RX_ADDR_BASE  = 0x0A,
TX_ADDR       = 0x10,
RX_PW_BASE    = 0x11,
FIFO_STATUS   = 0x17,
DYNPD		  = 0x1c,
FEATURE       = 0x1d
};
} //NRF24L01_REG

namespace NRF24L01_FEATURE
{
enum {
EN_DPL          = 0b100,
EN_ACK_PAY      = 0b010,
EN_DYN_ACK      = 0b001
};
} //NRF24L01_FEATURE

//CONFIG register definitions
namespace NRF24L01_CONFIG
{
enum {
MASK_RX_DR      = 0x40,
MASK_TX_DS      = 0x20,
MASK_MAX_RT     = 0x10,
EN_CRC          = 0x08,
CRC0            = 0x04,
CRC_1BYTE       = EN_CRC,
CRC_2BYTE       = EN_CRC|CRC0,
CRC_OFF         = 0,
PWR_UP          = 0x02,
PRIM_RX         = 0x01,
};
} //NRF24L01_CONFIG


//STATUS register definitions
namespace NRF24L01_STATUS
{
enum {
RX_DR           = 0x40,
TX_DS           = 0x20,
MAX_RT          = 0x10,
RX_P_NO         = 0x0E,
RX_FIFO_EMPTY   = 0x0E,
TX_FULL         = 0x01,
};

static force_inline uint8_t get_rx_pipe_number(uint8_t status)
{
    return (status & RX_P_NO) >> 1;
}
} //NRF24L01_STATUS

namespace NRF24L01_AW
{
static force_inline uint8_t address_width(uint8_t bytes)
{
    return bytes - 2;
}
}

#define NRF24L01_MAX_PAYLOAD_LENGTH 32

#ifndef NRF24L01_DEFAULT_CONFIG
#define NRF24L01_DEFAULT_CONFIG (NRF24L01_CONFIG::PWR_UP | NRF24L01_CONFIG::CRC_2BYTE | NRF24L01_CONFIG::EN_CRC)
#endif

#ifdef NRF24L01_STATIC
#define NRF24L01_STATIC__ static
#define NRF24L01_STATIC_CONST__
#define NRF24L01_STATIC_CONFIG
#else
#define NRF24L01_STATIC__
#define NRF24L01_STATIC_CONST__ const
#endif

#ifndef nrf_enabled_pipes
    #error nrf_enabled_pipes must be defined in the config file, e.g. #define nrf_enabled_pipes 0b000011
#endif

class NRF24L01
{
public:
    typedef enum
    {
        s250k = 0x20,
        s1M   = 0,
        s2M   = 0x08
    } speed_t;

    typedef enum
    {
        dBm_18 = 0,
        dBm_12 = 0x02,
        dBm_6  = 0x04,
        dBm_0 = 0x06
    } power_t;

    typedef enum
    {
        irq_rx = 0x40,
        irq_tx = 0x20,
        irq_max_rt = 0x10
    } irq_t;

#ifndef NRF24L01_STATIC
    NRF24L01(SPI &spi_, DigitalOut const &csn, DigitalOut const &ce);
#endif
    NRF24L01_STATIC__ uint8_t init() NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ void write_reg(uint_fast8_t reg_nr, uint_fast8_t data) NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ void write(uint_fast8_t command, uint_fast8_t size, const char* data) NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ void write(uint_fast8_t command) NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ uint8_t read_reg(uint_fast8_t reg_nr) NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ uint_fast8_t status() NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ void start_receive();
    NRF24L01_STATIC__ force_inline void stop_receive() NRF24L01_STATIC_CONST__ { ce0(); }
    NRF24L01_STATIC__ void send_packet(const void *data, uint_fast8_t length);
    /** Returns number of bytes read or 0 if no packet is available.*/
    NRF24L01_STATIC__ uint_fast8_t read_payload(void *buffer, uint8_t max_length) NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ void set_channel(uint_fast8_t channel) NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ void set_speed_power(speed_t speed, power_t power) NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ void set_tx_mac(char const* mac) NRF24L01_STATIC_CONST__
    {
        write(NRF24L01_CMD::W_REGISTER | NRF24L01_REG::TX_ADDR, 5, mac);
        /* Also listen on the same address for ACK packets. */
        write(NRF24L01_CMD::W_REGISTER | NRF24L01_REG::RX_ADDR_BASE, 5, mac);
    }
    NRF24L01_STATIC__ void set_rx_mac(uint_fast8_t pipe, char const* mac) NRF24L01_STATIC_CONST__
    {
        write(NRF24L01_CMD::W_REGISTER | (NRF24L01_REG::RX_ADDR_BASE + pipe), pipe <= 1 ? 5 : 1, mac);
    }
    NRF24L01_STATIC__ void set_payload_length(uint_fast8_t pipe, uint_fast8_t length) NRF24L01_STATIC_CONST__;
    /** delay in µs */
    NRF24L01_STATIC__ force_inline void set_retransmit(uint_fast16_t delay, uint_fast8_t count) NRF24L01_STATIC_CONST__
    {
        /* 0 = 250µs, 1 = 500µs, always round to next higher value. */
        delay -= 1;
        delay /= 250;
        if (delay > 15) delay = 15;
        if (count > 15) count = 15;
        write_reg(NRF24L01_REG::SETUP_RETR, delay << 4 | count);
    }

    NRF24L01_STATIC__ force_inline void set_autoack(uint_fast8_t pipes) NRF24L01_STATIC_CONST__
    {
        write_reg(NRF24L01_REG::EN_AA, pipes & 0b00111111);
    }

    NRF24L01_STATIC__ force_inline void set_enabled_pipes(uint_fast8_t pipes) NRF24L01_STATIC_CONST__
    {
        write_reg(NRF24L01_REG::EN_RXADDR, pipes & 0b00111111);
    }

    NRF24L01_STATIC__ force_inline void flush_tx() NRF24L01_STATIC_CONST__
    {
        write(NRF24L01_CMD::FLUSH_TX);
    }

    NRF24L01_STATIC__ force_inline void flush_rx() NRF24L01_STATIC_CONST__
    {
        write(NRF24L01_CMD::FLUSH_RX);
    }


    NRF24L01_STATIC__ force_inline uint8_t get_config() NRF24L01_STATIC_CONST__
    {
#if defined(NRF24L01_STATIC_CONFIG)
        return NRF24L01_DEFAULT_CONFIG;
#elif defined(NRF24L01_READBACK_CONFIG)
        return read_reg(NRF24L01_REG::CONFIG);
#else
        return config_;
#endif
    }

    NRF24L01_STATIC__ force_inline void set_config(uint8_t config)
    {
#if !defined(NRF24L01_STATIC_CONFIG) && !defined(NRF24L01_READBACK_CONFIG)
        config_ = config;
#endif
        write_reg(NRF24L01_REG::CONFIG, config);
    }


#ifndef NRF24L01_STATIC_CONFIG
    NRF24L01_STATIC__ force_inline void enable_interrupts(uint8_t interrupts)
    {
        //nrf24l01 uses inverted logic: 1 = interrupt disabled
        set_config(get_config() & ~interrupts);
    }

    NRF24L01_STATIC__ force_inline void disable_interrupts(uint8_t interrupts)
    {
        //nrf24l01 uses inverted logic: 1 = interrupt disabled
        set_config(get_config() | ~interrupts);
    }
#endif

    /* Note: You must also call set_autoack for these pipes. */
    NRF24L01_STATIC__ force_inline void set_dyn_payload_length(uint_fast8_t pipes)
    {
        write_reg(NRF24L01_REG::DYNPD, pipes & 0b00111111);
    }


    NRF24L01_STATIC__ uint_fast8_t read_retransmit_counter() NRF24L01_STATIC_CONST__;
    /* Returns true on success. */
    NRF24L01_STATIC__ bool wait_transmit_complete() NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ void  wait_transmit_complete_no_status() NRF24L01_STATIC_CONST__ {
    while (!(status() & (NRF24L01_STATUS::MAX_RT | NRF24L01_STATUS::TX_DS)));
    }

    NRF24L01_STATIC__ unsigned read_power_detector(uint_fast8_t channel) NRF24L01_STATIC_CONST__;
    NRF24L01_STATIC__ void dump_registers();
    NRF24L01_STATIC__ void dump_status();

    NRF24L01_STATIC__ force_inline bool data_ready()
    {
        return (status() & NRF24L01_STATUS::RX_FIFO_EMPTY) != NRF24L01_STATUS::RX_FIFO_EMPTY;
    }

#ifdef NRF24L01_STATIC
    static force_inline void ce0() { NRF24L01_PORT_CE &= ~ _BV(NRF24L01_PIN_CE); }
    static force_inline void ce1() { NRF24L01_PORT_CE |= _BV(NRF24L01_PIN_CE); }
    static force_inline void csn0() { NRF24L01_PORT_CSN &= ~ _BV(NRF24L01_PIN_CSN); }
    static force_inline void csn1() { NRF24L01_PORT_CSN |= _BV(NRF24L01_PIN_CSN); }
    static force_inline uint8_t spiwrite(uint8_t data) { return SPI::write(data); }
#else
    force_inline void ce0() const { ce = 0; }
    force_inline void ce1() const { ce = 1; }
    force_inline void csn0() const { csn = 0; }
    force_inline void csn1() const { csn = 1; }
    force_inline uint8_t spiwrite(uint8_t data) const { return spi.write(data); }
#endif
private:
#if !defined(NRF24L01_STATIC)
    SPI &spi;
    DigitalOut csn;
    DigitalOut ce;
#endif
#if !defined(NRF24L01_READBACK_CONFIG) && !defined(NRF24L01_STATIC_CONFIG)
    uint8_t config_;
#endif
};

#endif // NRF24L01_HPP
