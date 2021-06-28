#pragma once

// system includes
#include <string>
#include <vector>

// esp-idf includes
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp_netif_types.h>

// 3rdparty lib includes
#include <tl/expected.hpp>

// local includes
#include "espwifistackconfig.h"
#include "espwifistackenums.h"
#include "espwifiutils.h"
#include "espchrono.h"
#include "cppsignal.h"

namespace wifi_stack {
struct scan_result
{
    std::vector<wifi_ap_record_t> entries;
    espchrono::millis_clock::time_point finished;
};

extern esp_netif_t* esp_netifs[ESP_IF_MAX];

extern const WiFiState &wifiStateMachineState;

extern cpputils::Signal<> scanResultChanged;

//! Call once at startup
void init(const config &config);

//! Call repeatedly, approx. every 100ms
void update(const config &config);

//! Tells the status of the STA interface (connected, ...)
WiFiStaStatus get_sta_status();

//! Tries to begin a new scan, if succeeds clears the old scan result
esp_err_t begin_scan(const config &config);

//! Tells the status of the currently running scan (finished, ...)
WiFiScanStatus get_scan_status();

//! Retrieves the current scan result (TODO: create a fresh container
//! every time to free memory and avoid cross thread access crashes)
const std::optional<scan_result> &get_scan_result();

//! Clears the scan result
void delete_scan_result();

tl::expected<wifi_ap_record_t, std::string> get_sta_ap_info();

tl::expected<wifi_stack::mac_t, std::string> get_base_mac_addr();

} // namespace wifi_stack