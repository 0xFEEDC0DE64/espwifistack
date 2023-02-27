#pragma once

#include "sdkconfig.h"

// system includes
#include <string>
#include <string_view>
#include <vector>
#include <optional>
#include <expected>

// esp-idf includes
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp_netif_types.h>
#ifdef CONFIG_ETH_ENABLED
#include <esp_eth_driver.h>
#endif

// 3rdparty lib includes
#include <espchrono.h>
#include <cppsignal.h>
#include <ring-buffer.h>

// local includes
#include "espwifistackconfig.h"
#include "espwifistackenums.h"
#include "espwifiutils.h"

namespace wifi_stack {
struct scan_result
{
    std::vector<wifi_ap_record_t> entries;
    espchrono::millis_clock::time_point finished;
};

extern esp_netif_t* esp_netifs[ESP_IF_MAX];

extern const WiFiState &wifiStateMachineState;

extern cpputils::Signal<> scanResultChanged;

extern const std::optional<espchrono::millis_clock::time_point> &lastStaSwitchedFromConnected;
extern const std::optional<espchrono::millis_clock::time_point> &lastStaSwitchedToConnected;

//! Call once at startup
void init(const config &config);

//! Call repeatedly, approx. every 100ms
void update(const config &config);

extern const bool &esp_wifi_started;

extern const uint8_t &sta_error_count;
struct sta_error_t {
    espchrono::millis_clock::time_point timestamp;
    std::string ssid;
    mac_t bssid;
    wifi_err_reason_t reason;

    std::string toString() const;
};
extern const ring_buffer<sta_error_t, 5> &last_sta_errors;

extern const std::optional<espchrono::millis_clock::time_point> &last_wifi_connect_failed;

extern const std::optional<sta_error_t> &last_sta_error;

extern const std::vector<mac_t> &pastConnectPlan;
extern const mac_t &currentConnectPlanEntry;
extern const std::vector<mac_t> &connectPlan;

#ifdef CONFIG_ETH_ENABLED
extern const std::optional<std::expected<void, std::string>> &eth_init_status;
#endif

wifi_mode_t get_wifi_mode();

//! Tells the status of the STA interface (connected, ...)
WiFiStaStatus get_sta_status();

//! Tries to begin a new scan, if succeeds clears the old scan result
std::expected<void, std::string> begin_scan(const sta_config &sta_config);

//! Tells the status of the currently running scan (finished, ...)
WiFiScanStatus get_scan_status();

//! Retrieves the current scan result (TODO: create a fresh container
//! every time to free memory and avoid cross thread access crashes)
const std::optional<scan_result> &get_scan_result();

//! Clears the scan result
void delete_scan_result();

//! Util wrappers
using mac_or_error = std::expected<mac_t, std::string>;
std::expected<wifi_ap_record_t, std::string> get_sta_ap_info();
mac_or_error get_mac_addr(wifi_interface_t ifx);
mac_or_error get_default_mac_addr();
mac_or_error get_custom_mac_addr();
mac_or_error get_base_mac_addr();
std::expected<void, std::string> set_base_mac_addr(mac_t mac_addr);
std::expected<esp_netif_ip_info_t, std::string> get_ip_info(esp_netif_t *esp_netif);
std::expected<std::string_view, std::string> get_hostname_for_interface(esp_interface_t interf);
std::expected<std::string_view, std::string> get_hostname_for_interface(esp_netif_t *esp_netif);

#ifdef CONFIG_ETH_ENABLED
esp_eth_handle_t getEthHandle();
bool get_eth_connected();
bool get_eth_has_ip();
#endif
} // namespace wifi_stack
