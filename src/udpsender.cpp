#include "udpsender.h"

// esp-idf includes
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <errno.h>
#include <esp_log.h>

// 3rdparty lib includes
#include <fmt/core.h>
#include <futurecpp.h>

// local includes
#include "espwifistack.h"

namespace wifi_stack {
namespace {
constexpr const char * const TAG = "UDPSENDER";
} // namespace

UdpSender::UdpSender() :
    m_udp_server{socket(AF_INET, SOCK_DGRAM, 0)}
{
    if (!ready())
    {
        ESP_LOGE(TAG, "could not create socket: %d", errno);
        return;
    }

    fcntl(m_udp_server, F_SETFL, O_NONBLOCK);
}

UdpSender::~UdpSender()
{
    if (ready())
        close(m_udp_server);
}

tl::expected<void, std::string> UdpSender::send(esp_interface_t interf, uint16_t port, std::string_view buf)
{
    const auto interfPtr = esp_netifs[interf];
    if (!interfPtr)
        return tl::make_unexpected(fmt::format("esp_netifs[{}] is invalid", std::to_underlying(interf)));

    return send(interfPtr, port, buf);
}

tl::expected<void, std::string> UdpSender::send(esp_netif_t *interf, uint16_t port, std::string_view buf)
{
    if (!interf)
        return tl::make_unexpected("invalid interf");

    esp_netif_ip_info_t ip;
    if (const auto result = esp_netif_get_ip_info(interf, &ip); result != ESP_OK)
        return tl::make_unexpected(fmt::format("esp_netif_get_ip_info() failed with {}", esp_err_to_name(result)));

    return send(ip, port, buf);
}

tl::expected<void, std::string> UdpSender::send(const esp_netif_ip_info_t &ip, uint16_t port, std::string_view buf)
{
    struct sockaddr_in recipient;

    recipient.sin_addr.s_addr = wifi_calculate_broadcast(ip_address_t(ip.gw.addr), ip_address_t(ip.netmask.addr)).value();

    recipient.sin_family = AF_INET;
    recipient.sin_port = htons(port);

    return send(recipient, buf);
}

tl::expected<void, std::string> UdpSender::send(const struct sockaddr_in &recipient, std::string_view buf)
{
    if (!ready())
        return tl::make_unexpected("initializing failed, not ready to send");

    if (const ssize_t sent = sendto(m_udp_server, buf.data(), buf.size(), 0, (const struct sockaddr*)&recipient, sizeof(recipient)); sent < 0)
        return tl::make_unexpected(fmt::format("send failed with {} (errno={})", sent, errno));
    else if (sent != buf.size())
        return tl::make_unexpected(fmt::format("sent bytes does not match, expected={}, sent={}", buf.size(), sent));

    return {};
}

tl::expected<void, std::string> UdpSender::send(const struct sockaddr_in6 &recipient, std::string_view buf)
{
    if (!ready())
        return tl::make_unexpected("initializing failed, not ready to send");

    if (const ssize_t sent = sendto(m_udp_server, buf.data(), buf.size(), 0, (const struct sockaddr*)&recipient, sizeof(recipient)); sent < 0)
        return tl::make_unexpected(fmt::format("send failed with {} (errno={})", sent, errno));
    else if (sent != buf.size())
        return tl::make_unexpected(fmt::format("sent bytes does not match, expected={}, sent={}", buf.size(), sent));

    return {};
}

tl::expected<void, std::string> UdpSender::send(ip_addr_t ip, uint16_t port, std::string_view buf)
{
    switch (ip.type)
    {
    case IPADDR_TYPE_V4:
    {
        struct sockaddr_in recipient;
        recipient.sin_family = AF_INET;
        recipient.sin_addr.s_addr = ip.u_addr.ip4.addr;
        recipient.sin_port = htons(port);
        return send(recipient, buf);
    }
    case IPADDR_TYPE_V6:
    {
        struct sockaddr_in6 recipient;
        recipient.sin6_family = AF_INET6;
        recipient.sin6_addr.un.u32_addr[0] = ip.u_addr.ip6.addr[0];
        recipient.sin6_addr.un.u32_addr[1] = ip.u_addr.ip6.addr[1];
        recipient.sin6_addr.un.u32_addr[2] = ip.u_addr.ip6.addr[2];
        recipient.sin6_addr.un.u32_addr[3] = ip.u_addr.ip6.addr[3];
        //recipient.sin6_scope_id = ip.u_addr.ip6.zone;
        recipient.sin6_port = htons(port);
        return send(recipient, buf);
    }
    default:
        return tl::make_unexpected(fmt::format("unsupported ip type {}", ip.type));
    }
}

tl::expected<void, std::string> UdpSender::send(esp_ip_addr_t ip, uint16_t port, std::string_view buf)
{
    switch (ip.type)
    {
    case ESP_IPADDR_TYPE_V4:
    {
        struct sockaddr_in recipient;
        recipient.sin_family = AF_INET;
        recipient.sin_addr.s_addr = ip.u_addr.ip4.addr;
        recipient.sin_port = htons(port);
        return send(recipient, buf);
    }
    case ESP_IPADDR_TYPE_V6:
    {
        struct sockaddr_in6 recipient;
        recipient.sin6_family = AF_INET6;
        recipient.sin6_addr.un.u32_addr[0] = ip.u_addr.ip6.addr[0];
        recipient.sin6_addr.un.u32_addr[1] = ip.u_addr.ip6.addr[1];
        recipient.sin6_addr.un.u32_addr[2] = ip.u_addr.ip6.addr[2];
        recipient.sin6_addr.un.u32_addr[3] = ip.u_addr.ip6.addr[3];
        //recipient.sin6_scope_id = ip.u_addr.ip6.zone;
        recipient.sin6_port = htons(port);
        return send(recipient, buf);
    }
    default:
        return tl::make_unexpected(fmt::format("unsupported ip type {}", ip.type));
    }
}

} // namespace wifi_stack
