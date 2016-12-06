#ifndef TINYUDP_H
#define TINYUDP_H
#include "config.h"

#ifdef TINY_UDP_ENABLED
#include "nrf24l01.h"
#ifndef device_ip
#warning "No device_ip defined. Compilation will fail"
#endif

#define user_flags (0x0f)
#define protocol_flags (~user_flags)

extern char mac[6];

struct __attribute__ ((__packed__)) __attribute__((__may_alias__)) tiny_udp_packet
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

    uint8_t udp_overhead() const { return sizeof(tiny_udp_packet) - sizeof(size_); }
    uint8_t payload_size() const { return size_ - udp_overhead(); }
    void set_payload_size(uint8_t size) { size_ = size + udp_overhead(); }
    uint8_t max_payload_size() const { return max_packet_size() - udp_overhead(); }
};

static inline void init_udp()
{
    mac[0] = device_ip;
    NRF24L01::set_rx_mac(1, mac);
}

bool send_udp_packet(tiny_udp_packet &buf, uint8_t ip, uint8_t port);
void send_udp_packet_nowait(tiny_udp_packet &buf, uint8_t ip, uint8_t port);

/** Receive an UDP packet in buf.
 * Note: This function might modify buf even if no valid packet is received!
 * Only rely on data in buf if this function returns true.
 */
bool receive_udp_packet(tiny_udp_packet &buf, uint8_t max_length);
#endif
#endif // TINYUDP_H
