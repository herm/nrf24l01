#ifndef TINYUDP_H
#define TINYUDP_H
#include "config.h"
#include <stdint.h>

#ifdef TINY_UDP_ENABLED
#include "nrf24l01.h"
#if NRF24L01_DEFAULT_ENABLED_PIPES & 0b1
#warning "Pipe 0 is always enabled. Most likely this is not what you want for TinyUDP."
#endif

namespace TinyUDP
{
#define user_flags (0x0f)
#define protocol_flags (~user_flags)

struct __attribute__ ((__packed__)) __attribute__((__may_alias__)) Packet
{
    private:
    uint8_t size_; // Total size of this packet excluding this field
    public:
    uint8_t source_ip;
    uint8_t dest_ip;
    uint8_t port;
    uint8_t flags;

    uint8_t packet_size() const { return size_; }
    void set_packet_size(uint8_t size) { size_ = size; }
    uint8_t max_packet_size() const { return 32; }
    int8_t size_left() const { return max_packet_size() - size_; }

    uint8_t udp_overhead() const { return sizeof(Packet) - sizeof(size_); }
    uint8_t payload_size() const { return size_ - udp_overhead(); }
    void set_payload_size(uint8_t size) { size_ = size + udp_overhead(); }
    uint8_t max_payload_size() const { return max_packet_size() - udp_overhead(); }
};

void set_device_ip(uint8_t ip);

static inline void init()
{
#ifdef TINY_UDP_DEFAULT_IP
    set_device_ip(TINY_UDP_DEFAULT_IP);
#endif
}

/* Blocking functions. */
bool send(Packet &buf, uint8_t ip, uint8_t port);

/* Non-blocking functions. */
void send_nowait(Packet &buf, uint8_t ip, uint8_t port);
void listen_nowait();

/** Receive an UDP packet in buf.
 * Note: This function might modify buf even if no valid packet is received!
 * Only rely on data in buf if this function returns true.
 */
bool receive_packet(Packet &buf, uint8_t max_length);
}
#endif
#endif // TINYUDP_H
