#include "tinyudp.h"
#include "config.h"

char mac[6] = nrf_mac;

bool send_udp_packet(tiny_udp_packet &buf, uint8_t ip, uint8_t port)
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
    return NRF24L01::wait_transmit_complete();
}
