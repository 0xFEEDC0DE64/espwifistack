#pragma once

// system includes
#include <string>
#include <array>
#include <cstdint>
#include <optional>

// esp-idf includes
#include <esp_wifi_types.h>

// local includes
#include "espwifiutils.h"

namespace wifi_stack {
struct static_ip_config
{
    ip_address_t ip;
    ip_address_t subnet;
    ip_address_t gateway;

    friend bool operator==(const static_ip_config &left, const static_ip_config &right)
    {
        return left.ip == right.ip &&
               left.subnet == right.subnet &&
               left.gateway == right.gateway;
    }

    friend bool operator!=(const static_ip_config &left, const static_ip_config &right)
    {
        return !(left == right);
    }
};

struct static_dns_config
{
    std::optional<ip_address_t> main;
    std::optional<ip_address_t> backup;
    std::optional<ip_address_t> fallback;

    friend bool operator==(const static_dns_config &left, const static_dns_config &right)
    {
        return  left.main == right.main &&
                left.backup == right.backup &&
                left.fallback == right.fallback;
    }

    friend bool operator!=(const static_dns_config &left, const static_dns_config &right)
    {
        return !(left == right);
    }
};

struct wifi_entry
{
    std::string ssid;
    std::string key;
    std::optional<static_ip_config> static_ip;
    static_dns_config static_dns;

    friend bool operator==(const wifi_entry &left, const wifi_entry &right)
    {
        return left.ssid == right.ssid &&
               left.key == right.key &&
               left.static_ip == right.static_ip &&
               left.static_dns == right.static_dns;
    }

    friend bool operator!=(const wifi_entry &left, const wifi_entry &right)
    {
        return !(left == right);
    }
};

struct sta_config
{
    std::array<wifi_entry, 10> wifis;
    int8_t min_rssi;

    friend bool operator==(const sta_config &left, const sta_config &right)
    {
        return left.wifis == right.wifis &&
               left.min_rssi == right.min_rssi;
    }

    friend bool operator!=(const sta_config &left, const sta_config &right)
    {
        return !(left == right);
    }
};

struct ap_config
{
    std::string ssid;
    std::string key;
    static_ip_config static_ip;
    int channel;
    wifi_auth_mode_t authmode;
    bool ssid_hidden;
    int max_connection;
    uint16_t beacon_interval;

    friend bool operator==(const ap_config &left, const ap_config &right)
    {
        return left.ssid == right.ssid &&
               left.key == right.key &&
               left.static_ip == right.static_ip &&
               left.channel == right.channel &&
               left.authmode == right.authmode &&
               left.ssid_hidden == right.ssid_hidden &&
               left.max_connection == right.max_connection &&
               left.beacon_interval == right.beacon_interval;
    }

    friend bool operator!=(const ap_config &left, const ap_config &right)
    {
        return !(left == right);
    }
};

struct config
{
    bool wifiEnabled;
    std::string hostname;
    sta_config sta;
    ap_config ap;

    friend bool operator==(const config &left, const config &right)
    {
        return left.wifiEnabled == right.wifiEnabled &&
               left.hostname == right.hostname &&
               left.sta == right.sta &&
               left.ap == right.ap;
    }

    friend bool operator!=(const config &left, const config &right)
    {
        return !(left == right);
    }
};
} // namespace wifi_stack
