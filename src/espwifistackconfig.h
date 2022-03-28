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

// 3rdparty lib includes
#include <espchrono.h>

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

struct dual_ant_config
{

};

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

enum eth_clock_mode_t { ETH_CLOCK_GPIO0_IN, ETH_CLOCK_GPIO0_OUT, ETH_CLOCK_GPIO16_OUT, ETH_CLOCK_GPIO17_OUT };

enum eth_phy_type_t { ETH_PHY_LAN87XX, ETH_PHY_TLK110, ETH_PHY_RTL8201, ETH_PHY_DP83848, ETH_PHY_DM9051, ETH_PHY_KSZ80XX, ETH_PHY_MAX };

struct eth_config
{
    std::string hostname;
    std::optional<static_ip_config> static_ip;
    static_dns_config static_dns;

    uint8_t phy_addr;
    int reset_gpio;
    int mdc;
    int mdio;
    eth_phy_type_t type;
    eth_clock_mode_t clk_mode;

    friend bool operator==(const eth_config &left, const eth_config &right)
    {
        return left.hostname == right.hostname &&
               left.static_ip == right.static_ip &&
               left.static_dns == right.static_dns &&
               left.phy_addr == right.phy_addr &&
               left.reset_gpio == right.reset_gpio &&
               left.mdc == right.mdc &&
               left.mdio == right.mdio &&
               left.type == right.type &&
               left.clk_mode == right.clk_mode;
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
    std::optional<dual_ant_config> dual_ant;
    std::optional<sta_config> sta;
    std::optional<ap_config> ap;
#ifdef CONFIG_ETH_ENABLED
    std::optional<eth_config> eth;
#endif
    std::optional<wifi_country_t> country;

    friend bool operator==(const config &left, const config &right)
    {
        return left.base_mac_override == right.base_mac_override &&
               left.sta == right.sta &&
               left.ap == right.ap
#ifdef CONFIG_ETH_ENABLED
                && left.eth == right.eth
#endif
                ;
    }

    friend bool operator!=(const config &left, const config &right)
    {
        return !(left == right);
    }
};
} // namespace wifi_stack
