#pragma once

// system includes
#include <cstring>
#include <optional>
#include <array>
#include <string_view>
#include <memory>

// esp-idf includes
#include <lwip/ip_addr.h>
#include <lwip/udp.h>
#include <lwip/pbuf.h>
#include <esp_netif.h>

// local includes
#include "cppmacros.h"
#include "delayedconstruction.h"
#include "espwifiutils.h"
#include "wrappers/queue.h"

namespace wifi_stack {
using pbufUniquePtr = std::unique_ptr<pbuf, decltype(&pbuf_free)>;

struct UdpPacketWrapper
{
    //UdpPacketWrapper(pbufUniquePtr &&pb, const ip_addr_t *addr, uint16_t port, struct netif * netif);
    ~UdpPacketWrapper() = default;

    UdpPacketWrapper(UdpPacketWrapper &&other) = default;
    UdpPacketWrapper(const UdpPacketWrapper &other) = delete;

    UdpPacketWrapper &operator=(UdpPacketWrapper &&other) = default;
    UdpPacketWrapper &operator=(const UdpPacketWrapper &other) = delete;

    auto data() const { return _data; }

    tcpip_adapter_if_t tcpIpAdapter() const;

    struct netif *ntif() const { return _ntif; }

    bool isBroadcast() const { return ip_addr_isbroadcast(&(_local.addr), _ntif); }
    bool isMulticast() const { return ip_addr_ismulticast(&(_local.addr)); }

    ip_addr_t localAddr() const { return _local.addr; }

    uint16_t localPort() const { return _local.port; }

    ip_addr_t remoteAddr() const { return _remote.addr; }

    uint16_t remotePort() const { return _remote.port; }

    wifi_stack::mac_t remoteMac() const { return _remoteMac; }

    pbufUniquePtr _pb;
    std::string_view _data;
    struct netif *_ntif;
    struct {
        ip_addr_t addr;
        uint16_t port;
    } _local, _remote;
    wifi_stack::mac_t _remoteMac;
};

class AsyncUdpListener
{
    CPP_DISABLE_COPY_MOVE(AsyncUdpListener)

public:
    AsyncUdpListener() = default;

    bool listen(const ip_addr_t *addr, uint16_t port);

//    bool listen(const IPAddress addr, uint16_t port)
//    {
//        ip_addr_t laddr;
//        laddr.type = IPADDR_TYPE_V4;
//        laddr.u_addr.ip4.addr = addr;
//        return listen(&laddr, port);
//    }

//    bool listen(const IPv6Address addr, uint16_t port)
//    {
//        ip_addr_t laddr;
//        laddr.type = IPADDR_TYPE_V6;
//        memcpy((uint8_t*)(laddr.u_addr.ip6.addr), (const uint8_t*)addr, 16);
//        return listen(&laddr, port);
//    }

    bool listen(uint16_t port)
    {
        return listen(IP_ANY_TYPE, port);
    }

    std::optional<UdpPacketWrapper> poll(TickType_t xTicksToWait = 0);

    void _udp_task_post(udp_pcb *pcb, pbuf *pb, const ip_addr_t *addr, uint16_t port, struct netif *netif);

private:
    bool _init();
    void close();

private:
    cpputils::DelayedConstruction<espcpputils::queue> _udp_queue;
    udp_pcb *_pcb{};
    bool _connected{};
};

} // namespace wifi_stack
