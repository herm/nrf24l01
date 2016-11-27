#include "config.h"

#ifdef TINY_UDP_ENABLED
#include "tinyudp.h"

char mac[6] = nrf_mac;

/* Usually only one of send_udp_packet and send_udp_packet_nowait is used. */
static force_inline void send_udp_packet_internal(tiny_udp_packet &buf, uint8_t ip, uint8_t port)
{
    mac[0] = ip;
    NRF24L01::set_tx_mac(mac);
    buf.source_ip = device_ip;
    buf.dest_ip = ip;
    buf.port = port;
    buf.flags &= user_flags;
    NRF24L01::send_packet(&(buf.source_ip), buf.packet_size());
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

bool receive_udp_packet(tiny_udp_packet &buf)
{
    uint8_t size = NRF24L01::read_payload(&buf.source_ip);
    buf.set_packet_size(size);
    if (size < sizeof(tiny_udp_packet) || (buf.flags & protocol_flags) || buf.dest_ip != device_ip) return false;
    return true;
}
#endif
