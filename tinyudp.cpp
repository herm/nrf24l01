#include "config.h"

#ifdef TINY_UDP_ENABLED
#include "tinyudp.h"

char mac[6] = nrf_mac;

/* Usually only one of send_udp_packet and send_udp_packet_nowait is used. */
void send_udp_packet_nowait(tiny_udp_packet &buf, uint8_t ip, uint8_t port)
{
    NRF24L01::stop_receive();
    mac[0] = ip;
    NRF24L01::set_tx_mac(mac);
    NRF24L01::set_enabled_pipes(0b1);
    buf.source_ip = device_ip;
    buf.dest_ip = ip;
    buf.port = port;
    buf.flags &= user_flags;
    NRF24L01::send_packet(&(buf.source_ip), buf.packet_size());
}

void listen_for_udp_nowait()
{
    NRF24L01::set_enabled_pipes(nrf_enabled_pipes);
    NRF24L01::start_receive();
}

bool send_udp_packet(tiny_udp_packet &buf, uint8_t ip, uint8_t port)
{
    send_udp_packet_nowait(buf, ip, port);
    bool result = NRF24L01::wait_transmit_complete();
    listen_for_udp_nowait();
    return result;
}

bool receive_udp_packet(tiny_udp_packet &buf, uint8_t max_length)
{
    uint8_t size = NRF24L01::read_payload(&buf.source_ip, max_length);
    buf.set_packet_size(size);
    if (size < sizeof(tiny_udp_packet) || (buf.flags & protocol_flags) || buf.dest_ip != device_ip) return false;
    return true;
}
#endif
