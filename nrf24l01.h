#ifndef NRF24L01_HPP
#define NRF24L01_HPP

#include <inttypes.h>
#include "pin.h"

/* https://github.com/r0ket/r0ket/blob/master/firmware/funk/nrf24l01p.h */
// Commands
#define C_R_REGISTER        0x00
#define C_W_REGISTER        0x20
#define C_R_RX_PAYLOAD      0x61
#define C_W_TX_PAYLOAD      0xA0
#define C_FLUSH_TX          0xE1
#define C_FLUSH_RX          0xE2
#define C_REUSE_TX_PL       0xE3
#define C_R_RX_PL_WID       0x60
#define C_W_ACK_PAYLOAD     0xA8
#define C_W_TX_PAYLOAD_NOCACK	0xB0
#define C_NOP			0xFF

// Registers
#define R_CONFIG        0x00
#define R_EN_AA         0x01
#define R_EN_RXADDR     0x02
#define R_SETUP_AW      0x03
#define R_SETUP_RETR    0x04
#define R_RF_CH         0x05
#define R_RF_SETUP      0x06
#define R_STATUS        0x07
#define R_OBSERVE_TX    0x08
#define R_RPD           0x09
#define R_RX_ADDR_BASE  0x0A
#define R_TX_ADDR		0x10
#define R_RX_PW_BASE    0x11
#define R_FIFO_STATUS   0x17
#define R_DYNPD			0x1c

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

    NRF24L01(SPI &spi_, DigitalOut const &csn, DigitalOut const &ce);
    void write_reg(uint_fast8_t reg_nr, uint_fast8_t data);
    void write(uint_fast8_t command, uint_fast8_t size, const uint8_t* data);
    void write(uint_fast8_t command);
    uint8_t read_reg(uint_fast8_t reg_nr);
    uint_fast8_t status();
    void start_receive();
    void end_receive();
    void send_packet(const uint8_t* data, uint_fast8_t length);
    /** Returns pipe number of bytes read or 0 if no packet is available. Buffer must be long enough for the packet. 32 Bytes is always enough. */
    uint_fast8_t read_payload(uint8_t *buffer, uint8_t *pipe=0);
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
        write_reg(R_SETUP_RETR, delay << 4 | count);
    }
    force_inline void set_autoack(uint_fast8_t pipes)
    {
        write_reg(R_EN_AA, pipes & 0b00111111);
    }
    uint_fast8_t read_retransmit_counter();
    /* Returns true on success. */
    bool wait_transmit_complete();

    unsigned read_power_detector(uint_fast8_t channel);
    void dump_registers();
    void dump_status();
private:
    SPI &spi;
    DigitalOut csn;
    DigitalOut ce;
    uint8_t config;
};

#endif // NRF24L01_HPP
