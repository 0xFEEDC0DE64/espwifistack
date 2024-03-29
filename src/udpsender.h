#pragma once

// system includes
#include <string>
#include <string_view>
#include <expected>

// esp-idf includes
#include <esp_interface.h>
#include <esp_netif_ip_addr.h>
#include <esp_netif_types.h>
#include <lwip/sockets.h>

namespace wifi_stack {

class UdpSender
{
public:
    UdpSender();
    ~UdpSender();

    bool ready() const { return m_udp_server != -1; }

    std::expected<void, std::string> send(esp_interface_t interf, uint16_t port, std::string_view buf);
    std::expected<void, std::string> send(esp_netif_t *interf, uint16_t port, std::string_view buf);
    std::expected<void, std::string> send(const esp_netif_ip_info_t &ip, uint16_t port, std::string_view buf);
    std::expected<void, std::string> send(const struct sockaddr_in &recipient, std::string_view buf);
    std::expected<void, std::string> send(const struct sockaddr_in6 &recipient, std::string_view buf);
    std::expected<void, std::string> send(ip_addr_t ip, uint16_t port, std::string_view buf);
    std::expected<void, std::string> send(esp_ip_addr_t ip, uint16_t port, std::string_view buf);

private:
    const int m_udp_server;
};

} // namespace wifi_stack
