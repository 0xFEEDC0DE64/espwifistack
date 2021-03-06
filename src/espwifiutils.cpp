#include "espwifiutils.h"

// system includes
#include <cstdio>
#include <bitset>
#include <utility>

// esp-idf includes
#include <esp_log.h>

// 3rdparty lib includes
#include <fmt/core.h>

namespace wifi_stack {
namespace {
constexpr const char * const TAG = "WIFI_STACK";
} // namespace

bool wifi_ap_config_equal(const wifi_ap_config_t& lhs, const wifi_ap_config_t& rhs)
{
    auto leftSsid = lhs.ssid_len ?
                std::string_view{reinterpret_cast<const char*>(lhs.ssid), lhs.ssid_len} :
                std::string_view{reinterpret_cast<const char*>(lhs.ssid)};
    auto rightSsid = rhs.ssid_len ?
                std::string_view{reinterpret_cast<const char*>(rhs.ssid), rhs.ssid_len} :
                std::string_view{reinterpret_cast<const char*>(rhs.ssid)};

    return leftSsid == rightSsid &&
           std::string_view{reinterpret_cast<const char*>(lhs.password)} == std::string_view{reinterpret_cast<const char*>(rhs.password)} &&
           lhs.channel == rhs.channel &&
           lhs.authmode == rhs.authmode &&
           lhs.ssid_hidden == rhs.ssid_hidden &&
           lhs.max_connection == rhs.max_connection &&
           lhs.beacon_interval == rhs.beacon_interval &&
           lhs.pairwise_cipher == rhs.pairwise_cipher;
}

bool wifi_sta_config_equal(const wifi_sta_config_t& lhs, const wifi_sta_config_t& rhs)
{
    return std::string_view{reinterpret_cast<const char*>(lhs.ssid)} == std::string_view{reinterpret_cast<const char*>(rhs.ssid)} &&
           std::string_view{reinterpret_cast<const char*>(lhs.password)} == std::string_view{reinterpret_cast<const char*>(rhs.password)} &&
           lhs.scan_method == rhs.scan_method &&
           lhs.bssid_set == rhs.bssid_set &&
           (lhs.bssid_set ? (*reinterpret_cast<const mac_t *>(lhs.bssid) == *reinterpret_cast<const mac_t *>(rhs.bssid)) : true) &&
           lhs.channel == rhs.channel &&
           lhs.listen_interval == rhs.listen_interval &&
           lhs.sort_method == rhs.sort_method &&
           lhs.threshold.rssi == rhs.threshold.rssi &&
           lhs.threshold.authmode == rhs.threshold.authmode &&
           lhs.pmf_cfg.capable == rhs.pmf_cfg.capable &&
           lhs.pmf_cfg.required == rhs.pmf_cfg.required &&
           bool(lhs.rm_enabled) == bool(rhs.rm_enabled) &&
           bool(lhs.btm_enabled) == bool(rhs.btm_enabled);
}

std::string toString(wifi_auth_mode_t authMode)
{
    switch (authMode)
    {
    case WIFI_AUTH_OPEN:            return "OPEN";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2_WPA3_PSK";
    case WIFI_AUTH_WAPI_PSK:        return "WAPI_PSK";
    case WIFI_AUTH_MAX:             return "MAX";
    default:
        ESP_LOGW(TAG, "Unknown wifi_auth_mode_t(%i)", std::to_underlying(authMode));
        return fmt::format("Unknown wifi_auth_mode_t({})", std::to_underlying(authMode));
    }
}

std::string toString(wifi_cipher_type_t cipherType)
{
    switch (cipherType)
    {
    case WIFI_CIPHER_TYPE_NONE:        return "NONE";
    case WIFI_CIPHER_TYPE_WEP40:       return "WEP40";
    case WIFI_CIPHER_TYPE_WEP104:      return "WEP104";
    case WIFI_CIPHER_TYPE_TKIP:        return "TKIP";
    case WIFI_CIPHER_TYPE_CCMP:        return "CCMP";
    case WIFI_CIPHER_TYPE_TKIP_CCMP:   return "TKIP_CCMP";
    case WIFI_CIPHER_TYPE_AES_CMAC128: return "AES_CMAC128";
    case WIFI_CIPHER_TYPE_SMS4:        return "SMS4";
    case WIFI_CIPHER_TYPE_UNKNOWN:     return "UNKNOWN";
    default:
        ESP_LOGW(TAG, "Unknown wifi_cipher_type_t(%i)", std::to_underlying(cipherType));
        return fmt::format("Unknown wifi_cipher_type_t({})", std::to_underlying(cipherType));
    }
}

std::string toString(esp_interface_t interface)
{
    switch (interface)
    {
    case ESP_IF_WIFI_STA: return "STA";
    case ESP_IF_WIFI_AP:  return "AP";
    case ESP_IF_ETH:      return "ETH";
    default:
        ESP_LOGW(TAG, "Unknown esp_interface_t(%i)", std::to_underlying(interface));
        return fmt::format("Unknown esp_interface_t({})", std::to_underlying(interface));
    }
}

std::string toString(esp_netif_dhcp_status_t status)
{
    switch (status)
    {
    case ESP_NETIF_DHCP_INIT:    return "INIT";
    case ESP_NETIF_DHCP_STARTED: return "STARTED";
    case ESP_NETIF_DHCP_STOPPED: return "STOPPED";
    default:
        ESP_LOGW(TAG, "Unknown esp_netif_dhcp_status_t(%i)", std::to_underlying(status));
        return fmt::format("Unknown esp_netif_dhcp_status_t({})", std::to_underlying(status));
    }
}

template<> tl::expected<mac_t, std::string> fromString<mac_t>(std::string_view str)
{
    mac_t result;
    if (std::sscanf(str.data(), "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
                    &result[0], &result[1], &result[2], &result[3], &result[4], &result[5]) == 6)
        return result;

    return tl::make_unexpected(fmt::format("invalid format ({})", str));
}

std::string toString(const mac_t &val)
{
    return fmt::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                       val.at(0), val.at(1), val.at(2), val.at(3), val.at(4), val.at(5));
}

std::string toString(const std::optional<mac_t> &val)
{
    return val ? toString(*val) : "nullopt";
}

template<> tl::expected<ip_address_t, std::string> fromString<ip_address_t>(std::string_view str)
{
    // TODO: add support for "a", "a.b", "a.b.c" formats
    // TODO: replace with scanf for better performance

    ip_address_t result;

    uint16_t acc = 0; // Accumulator
    uint8_t dots = 0;

    for (char c : str)
    {
        if (c >= '0' && c <= '9')
        {
            acc = acc * 10 + (c - '0');
            if (acc > 255)
                return tl::make_unexpected("Value out of [0..255] range");
        }
        else if (c == '.')
        {
            if (dots == 3)
                return tl::make_unexpected("Too many dots (there must be 3 dots)");

            result[dots++] = acc;
            acc = 0;
        }
        else
            return tl::make_unexpected("Invalid char");
    }

    if (dots != 3)
        return tl::make_unexpected("Too few dots (there must be 3 dots)");

    result[3] = acc;

    return result;
}

std::string toString(ip_address_t val)
{
    return fmt::format("{}.{}.{}.{}", val[0], val[1], val[2], val[3]);
}

std::string toString(const std::optional<ip_address_t> &val)
{
    return val ? toString(*val) : "nullopt";
}

ip_address_t wifi_calculate_network_id(ip_address_t ip, ip_address_t subnet)
{
    ip_address_t networkID;

    for (size_t i = 0; i < 4; i++)
        networkID[i] = subnet[i] & ip[i];

    return networkID;
}

ip_address_t wifi_calculate_broadcast(ip_address_t ip, ip_address_t subnet)
{
    ip_address_t broadcastIp;

    for (int i = 0; i < 4; i++)
        broadcastIp[i] = ~subnet[i] | ip[i];

    return broadcastIp;
}

uint8_t wifi_calculate_subnet_cidr(ip_address_t subnetMask)
{
    return std::bitset<32>{subnetMask.value()}.count();
}

std::string toString(ip4_addr_t val)
{
    return toString(ip_address_t{val.addr});
}

std::string toString(const ip6_addr_t &val)
{
    char str[40];
    ip6addr_ntoa_r(&val, str, sizeof(str));
    return std::string{str};
}

std::string toString(ip_addr_t val)
{
    switch (val.type)
    {
    case IPADDR_TYPE_V4: return toString(val.u_addr.ip4);
    case IPADDR_TYPE_V6: return toString(val.u_addr.ip6);
    default:
        //ESP_LOGW(TAG, "Unknown ipv%hhu", val.type);
        return fmt::format("Unknown ipv{}", val.type);
    }
}

std::string toString(const esp_ip_addr_t &val)
{
    switch (val.type)
    {
    case IPADDR_TYPE_V4: return toString(val.u_addr.ip4);
    case IPADDR_TYPE_V6: return toString(val.u_addr.ip6);
    default:
        ESP_LOGW(TAG, "Unknown ipv%hhu", val.type);
        return fmt::format("Unknown ipv{}", val.type);
    }
}

} // namespace wifi_stack
