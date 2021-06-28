#pragma once

// system includes
#include <string>
#include <array>
#include <cstdint>

// esp-idf includes
#include <esp_wifi_types.h>

// local includes
#include "espwifiutils.h"

namespace wifi_stack {
struct wifi_entry
{
    std::string ssid;
    std::string key;

    friend bool operator==(const wifi_entry &left, const wifi_entry &right)
    {
        return left.ssid == right.ssid &&
               left.key == right.key;
    }

    friend bool operator!=(const wifi_entry &left, const wifi_entry &right)
    {
        return !(left == right);
    }
};

struct ap_config : public wifi_entry
{
    int channel;
    wifi_auth_mode_t authmode;
    bool ssid_hidden;
    int max_connection;
    uint16_t beacon_interval;
    ip_address_t ip;
    ip_address_t subnet;

    friend bool operator==(const ap_config &left, const ap_config &right)
    {
        return *static_cast<const wifi_entry *>(&left) == *static_cast<const wifi_entry *>(&right) &&
               left.channel == right.channel &&
               left.authmode == right.authmode &&
               left.ssid_hidden == right.ssid_hidden &&
               left.max_connection == right.max_connection &&
               left.beacon_interval == right.beacon_interval &&
               left.ip == right.ip &&
               left.subnet == right.subnet;
    }

    friend bool operator!=(const ap_config &left, const ap_config &right)
    {
        return !(left == right);
    }
};

struct ip_setting
{
    bool dhcpEnabled;
    ip_address_t staticIp;
    ip_address_t staticGateway;
    ip_address_t staticSubnet;
    ip_address_t staticDns1;
    ip_address_t staticDns2;

    friend bool operator==(const ip_setting &left, const ip_setting &right)
    {
        return left.dhcpEnabled == right.dhcpEnabled &&
               left.staticIp == right.staticIp &&
               left.staticGateway == right.staticGateway &&
               left.staticSubnet == right.staticSubnet &&
               left.staticDns1 == right.staticDns1 &&
               left.staticDns2 == right.staticDns2;
    }

    friend bool operator!=(const ip_setting &left, const ip_setting &right)
    {
        return !(left == right);
    }
};

struct config
{
    bool wifiEnabled;
    std::string hostname;
    std::array<wifi_entry, 10> wifis;
    ip_setting sta_ip;
    ap_config ap;
    int8_t min_rssi;
};
} // namespace wifi_stack
