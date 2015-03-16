#ifndef TINYUDP_H
#define TINYUDP_H
#include "nrf24l01.h"

extern char mac[6];

struct __attribute__ ((__packed__)) __attribute__((__may_alias__)) tiny_udp_packet
{
    uint8_t size;
    uint8_t source_ip;
    uint8_t dest_ip;
    uint8_t ports;
    uint8_t flags;
    int8_t payload_bytes_left() const { return max_size() - size; }
    uint8_t max_size() const { return 28; }
};

static inline void init_udp()
{
    mac[0] = device_ip;
    NRF24L01::set_rx_mac(1, mac);
}

bool send_udp_packet(tiny_udp_packet &buf, uint8_t ip, uint8_t port);

#endif // TINYUDP_H
