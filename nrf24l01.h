#ifndef NRF24L01_HPP
#define NRF24L01_HPP

#include <inttypes.h>
#include "pin.h"

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

class SPI;
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

    NRF24L01(SPI &spi_, DigitalOut const &csn, DigitalOut const &ce);
    void write_reg(uint_fast8_t reg_nr, uint_fast8_t data);
    void write(uint_fast8_t command, uint_fast8_t size, const uint8_t* data);
    void write(uint_fast8_t command);
    uint8_t read_reg(uint_fast8_t reg_nr);
    uint_fast8_t status();
    void start_receive();
    void end_receive();
    void send_packet(const void *data, uint_fast8_t length);
    /** Returns number of bytes read or 0 if no packet is available. Buffer must be long enough for the packet. 32 Bytes is always enough. */
    uint_fast8_t read_payload(void *buffer, uint8_t length);
    void set_channel(uint_fast8_t channel);
    void set_speed_power(speed_t speed, power_t power);
    void set_tx_mac(uint8_t const* mac);
    force_inline void set_tx_mac(char const* mac) { set_tx_mac((uint8_t const*)mac); }
    void set_rx_mac(uint_fast8_t pipe, uint8_t const* mac);
    void set_payload_length(uint_fast8_t pipe, uint_fast8_t length);
    /** delay in µs */
    force_inline void set_retransmit(uint_fast16_t delay, uint_fast8_t count)
    {
        /* 0 = 250µs, 1 = 500µs, always round to next higher value. */
        delay -= 1;
        delay /= 250;
        if (delay > 15) delay = 15;
        if (count > 15) count = 15;
        write_reg(NRF24L01_REG::SETUP_RETR, delay << 4 | count);
    }

    force_inline void set_autoack(uint_fast8_t pipes)
    {
        write_reg(NRF24L01_REG::EN_AA, pipes & 0b00111111);
    }

    force_inline void set_enabled_pipes(uint_fast8_t pipes)
    {
        write_reg(NRF24L01_REG::EN_RXADDR, pipes & 0b00111111);
    }

    force_inline void enable_interrupts(uint8_t interrupts)
    {
        config &= ~interrupts; //nrf24l01 uses inverted logic: 1 = interrupt disabled
        write_reg(NRF24L01_REG::CONFIG, config);
    }

    force_inline void disable_interrupts(uint8_t interrupts)
    {
        config |= interrupts; //nrf24l01 uses inverted logic: 1 = interrupt disabled
        write_reg(NRF24L01_REG::CONFIG, config);
    }

    /* Note: You must also call set_autoack for these pipes. */
    force_inline void set_dyn_payload_length(uint_fast8_t pipes)
    {
        write_reg(NRF24L01_REG::DYNPD, pipes & 0b00111111);
        if (pipes)
        {
            //Enable all enhanced shockburst
            write_reg(NRF24L01_REG::FEATURE, NRF24L01_FEATURE::EN_DPL | NRF24L01_FEATURE::EN_DYN_ACK | NRF24L01_FEATURE::EN_ACK_PAY);
        }
    }


    uint_fast8_t read_retransmit_counter();
    /* Returns true on success. */
    bool wait_transmit_complete();

    unsigned read_power_detector(uint_fast8_t channel);
    void dump_registers();
    void dump_status();

    force_inline bool data_ready()
    {
        return !(status() & NRF24L01_STATUS::RX_FIFO_EMPTY);
    }
private:
    SPI &spi;
    DigitalOut csn;
    DigitalOut ce;
    uint8_t config;
};

#endif // NRF24L01_HPP
