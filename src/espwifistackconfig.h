#pragma once

#include "sdkconfig.h"

// system includes
#include <string>
#include <array>
#include <cstdint>
#include <optional>
#include <variant>

// esp-idf includes
#include <esp_wifi_types.h>
#include <hal/gpio_types.h>

// 3rdparty lib includes
#include <espchrono.h>

// local includes
#include "espwifiutils.h"

inline bool operator==(const wifi_country_t &left, const wifi_country_t &right)
{
    return std::equal(std::begin(left.cc), std::end(left.cc), std::begin(right.cc)) &&
           left.schan == right.schan &&
           left.nchan == right.nchan &&
           left.max_tx_power == right.max_tx_power &&
           left.policy == right.policy;
}

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

struct sta_active_scan_config
{
    espchrono::milliseconds32 min_per_chan{100};
    espchrono::milliseconds32 max_per_chan{300};

    friend bool operator==(const sta_active_scan_config &left, const sta_active_scan_config &right)
    {
        return left.min_per_chan == right.min_per_chan &&
               left.max_per_chan == right.max_per_chan;
    }

    friend bool operator!=(const sta_active_scan_config &left, const sta_active_scan_config &right)
    {
        return !(left == right);
    }
};

struct sta_passive_scan_config
{
    espchrono::milliseconds32 max_per_chan{300};

    friend bool operator==(const sta_passive_scan_config &left, const sta_passive_scan_config &right)
    {
        return left.max_per_chan == right.max_per_chan;
    }

    friend bool operator!=(const sta_passive_scan_config &left, const sta_passive_scan_config &right)
    {
        return !(left == right);
    }
};

struct sta_scan_config
{
    std::optional<espchrono::milliseconds32> interval = espchrono::minutes32{10};
    uint8_t channel = 0;
    bool show_hidden = false;
    std::variant<sta_active_scan_config, sta_passive_scan_config> time = sta_active_scan_config{};

    friend bool operator==(const sta_scan_config &left, const sta_scan_config &right)
    {
        return left.interval == right.interval &&
               left.channel == right.channel &&
               left.show_hidden == right.show_hidden &&
               left.time == right.time;
    }

    friend bool operator!=(const sta_scan_config &left, const sta_scan_config &right)
    {
        return !(left == right);
    }
};

#ifdef CONFIG_WIFI_DUAL_ANT
struct dual_ant_config
{
    gpio_num_t selectPin0{GPIO_NUM_2};
    gpio_num_t selectPin1{GPIO_NUM_25};

    friend bool operator==(const dual_ant_config &left, const dual_ant_config &right)
    {
        return left.selectPin0 == right.selectPin0 &&
               left.selectPin1 == right.selectPin1;
    }

    friend bool operator!=(const dual_ant_config &left, const dual_ant_config &right)
    {
        return !(left == right);
    }
};
#endif

struct sta_config
{
    std::string hostname;
    std::array<wifi_entry, 10> wifis;
    int8_t min_rssi = -90;
    bool long_range = false;
    sta_scan_config scan;

    friend bool operator==(const sta_config &left, const sta_config &right)
    {
        return left.hostname == right.hostname &&
               left.wifis == right.wifis &&
               left.min_rssi == right.min_rssi &&
               left.long_range == right.long_range &&
               left.scan == right.scan;
    }

    friend bool operator!=(const sta_config &left, const sta_config &right)
    {
        return !(left == right);
    }
};

struct ap_config
{
    std::string hostname;
    std::string ssid;
    std::string key;
    static_ip_config static_ip;
    uint8_t channel = 1;
    wifi_auth_mode_t authmode = WIFI_AUTH_WPA2_PSK;
    bool ssid_hidden = false;
    int max_connection = 4;
    uint16_t beacon_interval = 100;
    bool long_range = false;

    friend bool operator==(const ap_config &left, const ap_config &right)
    {
        return left.hostname == right.hostname &&
               left.ssid == right.ssid &&
               left.key == right.key &&
               left.static_ip == right.static_ip &&
               left.channel == right.channel &&
               left.authmode == right.authmode &&
               left.ssid_hidden == right.ssid_hidden &&
               left.max_connection == right.max_connection &&
               left.beacon_interval == right.beacon_interval &&
               left.long_range == right.long_range;
    }

    friend bool operator!=(const ap_config &left, const ap_config &right)
    {
        return !(left == right);
    }
};

#ifdef CONFIG_ETH_ENABLED

struct eth_config
{
    std::string hostname;
    std::optional<static_ip_config> static_ip;
    static_dns_config static_dns;

    friend bool operator==(const eth_config &left, const eth_config &right)
    {
        return left.hostname == right.hostname &&
               left.static_ip == right.static_ip &&
               left.static_dns == right.static_dns;
    }

    friend bool operator!=(const eth_config &left, const eth_config &right)
    {
        return !(left == right);
    }
};
#endif

struct config
{
    std::optional<mac_t> base_mac_override;
#ifdef CONFIG_WIFI_DUAL_ANT
    std::optional<dual_ant_config> dual_ant;
#endif
    std::optional<sta_config> sta;
    std::optional<ap_config> ap;
#ifdef CONFIG_ETH_ENABLED
    std::optional<eth_config> eth;
#endif
    std::optional<wifi_country_t> country;

    friend bool operator==(const config &left, const config &right)
    {
        return left.base_mac_override == right.base_mac_override &&
#ifdef CONFIG_WIFI_DUAL_ANT
               left.dual_ant == right.dual_ant &&
#endif
               left.sta == right.sta &&
               left.ap == right.ap &&
#ifdef CONFIG_ETH_ENABLED
               left.eth == right.eth &&
#endif
               left.country == right.country;
    }

    friend bool operator!=(const config &left, const config &right)
    {
        return !(left == right);
    }
};
} // namespace wifi_stack
