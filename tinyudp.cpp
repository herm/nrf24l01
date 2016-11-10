#ifdef TINY_UDP_ENABLED
#include "tinyudp.h"
#include "config.h"

char mac[6] = nrf_mac;

/* Usually only one of send_udp_packet and send_udp_packet_nowait is used. */
static force_inline void send_udp_packet_internal(tiny_udp_packet &buf, uint8_t ip, uint8_t port)
{
    mac[0] = ip;
    NRF24L01::set_tx_mac(mac);
#ifdef device_ip
    buf.source_ip = device_ip;
#endif
    buf.dest_ip = ip;
    buf.ports = port;
    buf.flags &= 0x0f;
    NRF24L01::send_packet(&(buf.source_ip), buf.size + sizeof(tiny_udp_packet) -  sizeof(buf.size));
}

bool send_udp_packet(tiny_udp_packet &buf, uint8_t ip, uint8_t port)
{
    send_udp_packet_internal(buf, ip, port);
    bool result = NRF24L01::wait_transmit_complete();
    NRF24L01::start_receive(); //TODO
    return result;
}

//TOOD: Compiling this function even if it isn't used increases code size by 28 bytes: why?
void send_udp_packet_nowait(tiny_udp_packet &buf, uint8_t ip, uint8_t port)
{
    send_udp_packet_internal(buf, ip, port);
}

bool receive_udp_packet(tiny_udp_packet &buf, uint8_t min_size)
{
    uint8_t size = NRF24L01::read_payload(&buf.source_ip);
    if (size < min_size || (buf.flags & protocol_flags) || buf.dest_ip != device_ip) return false;
    buf.size = size - (sizeof(tiny_udp_packet) - sizeof(buf.size));
    return true;
}
#endif
