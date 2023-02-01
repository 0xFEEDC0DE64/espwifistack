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

const char * toString(wifi_err_reason_t reason)
{
    switch (reason)
    {
    case WIFI_REASON_UNSPECIFIED:                        return "UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE:                        return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE:                         return "AUTH_LEAVE";
    case WIFI_REASON_ASSOC_EXPIRE:                       return "ASSOC_EXPIRE";
    case WIFI_REASON_ASSOC_TOOMANY:                      return "ASSOC_TOOMANY";
    case WIFI_REASON_NOT_AUTHED:                         return "NOT_AUTHED";
    case WIFI_REASON_NOT_ASSOCED:                        return "NOT_ASSOCED";
    case WIFI_REASON_ASSOC_LEAVE:                        return "ASSOC_LEAVE";
    case WIFI_REASON_ASSOC_NOT_AUTHED:                   return "ASSOC_NOT_AUTHED";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD:                return "DISASSOC_PWRCAP_BAD";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD:               return "DISASSOC_SUPCHAN_BAD";
    case WIFI_REASON_BSS_TRANSITION_DISASSOC:            return "BSS_TRANSITION_DISASSOC";
    case WIFI_REASON_IE_INVALID:                         return "IE_INVALID";
    case WIFI_REASON_MIC_FAILURE:                        return "MIC_FAILURE";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:             return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:           return "GROUP_KEY_UPDATE_TIMEOUT";
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:                 return "IE_IN_4WAY_DIFFERS";
    case WIFI_REASON_GROUP_CIPHER_INVALID:               return "GROUP_CIPHER_INVALID";
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID:            return "PAIRWISE_CIPHER_INVALID";
    case WIFI_REASON_AKMP_INVALID:                       return "AKMP_INVALID";
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION:              return "UNSUPP_RSN_IE_VERSION";
    case WIFI_REASON_INVALID_RSN_IE_CAP:                 return "INVALID_RSN_IE_CAP";
    case WIFI_REASON_802_1X_AUTH_FAILED:                 return "802_1X_AUTH_FAILED";
    case WIFI_REASON_CIPHER_SUITE_REJECTED:              return "CIPHER_SUITE_REJECTED";
    case WIFI_REASON_TDLS_PEER_UNREACHABLE:              return "TDLS_PEER_UNREACHABLE";
    case WIFI_REASON_TDLS_UNSPECIFIED:                   return "TDLS_UNSPECIFIED";
    case WIFI_REASON_SSP_REQUESTED_DISASSOC:             return "SSP_REQUESTED_DISASSOC";
    case WIFI_REASON_NO_SSP_ROAMING_AGREEMENT:           return "NO_SSP_ROAMING_AGREEMENT";
    case WIFI_REASON_BAD_CIPHER_OR_AKM:                  return "BAD_CIPHER_OR_AKM";
    case WIFI_REASON_NOT_AUTHORIZED_THIS_LOCATION:       return "NOT_AUTHORIZED_THIS_LOCATION";
    case WIFI_REASON_SERVICE_CHANGE_PERCLUDES_TS:        return "SERVICE_CHANGE_PERCLUDES_TS";
    case WIFI_REASON_UNSPECIFIED_QOS:                    return "UNSPECIFIED_QOS";
    case WIFI_REASON_NOT_ENOUGH_BANDWIDTH:               return "NOT_ENOUGH_BANDWIDTH";
    case WIFI_REASON_MISSING_ACKS:                       return "MISSING_ACKS";
    case WIFI_REASON_EXCEEDED_TXOP:                      return "EXCEEDED_TXOP";
    case WIFI_REASON_STA_LEAVING:                        return "STA_LEAVING";
    case WIFI_REASON_END_BA:                             return "END_BA";
    case WIFI_REASON_UNKNOWN_BA:                         return "UNKNOWN_BA";
    case WIFI_REASON_TIMEOUT:                            return "TIMEOUT";
    case WIFI_REASON_PEER_INITIATED:                     return "PEER_INITIATED";
    case WIFI_REASON_AP_INITIATED:                       return "AP_INITIATED";
    case WIFI_REASON_INVALID_FT_ACTION_FRAME_COUNT:      return "INVALID_FT_ACTION_FRAME_COUNT";
    case WIFI_REASON_INVALID_PMKID:                      return "INVALID_PMKID";
    case WIFI_REASON_INVALID_MDE:                        return "INVALID_MDE";
    case WIFI_REASON_INVALID_FTE:                        return "INVALID_FTE";
    case WIFI_REASON_TRANSMISSION_LINK_ESTABLISH_FAILED: return "TRANSMISSION_LINK_ESTABLISH_FAILED";
    case WIFI_REASON_ALTERATIVE_CHANNEL_OCCUPIED:        return "ALTERATIVE_CHANNEL_OCCUPIED";
    case WIFI_REASON_BEACON_TIMEOUT:                     return "BEACON_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND:                        return "NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL:                          return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL:                         return "ASSOC_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:                  return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_CONNECTION_FAIL:                    return "CONNECTION_FAIL";
    case WIFI_REASON_AP_TSF_RESET:                       return "AP_TSF_RESET";
    case WIFI_REASON_ROAMING:                            return "ROAMING";
    case WIFI_REASON_ASSOC_COMEBACK_TIME_TOO_LONG:       return "ASSOC_COMEBACK_TIME_TOO_LONG";
    }

    ESP_LOGE(TAG, "unknown reason %" PRIu8, reason);
    return "UNKNOWN";
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
