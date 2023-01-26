#include "espwifistack.h"

#include "sdkconfig.h"
#define LOG_LOCAL_LEVEL CONFIG_LOG_LOCAL_LEVEL_WIFI_STACK

// system includes
#include <optional>
#include <string>
#include <queue>
#include <functional>
#include <atomic>
#include <utility>

// esp-idf includes
#include <esp_log.h>
#include <esp_debug_helpers.h>
#include <dhcpserver/dhcpserver.h>
#include <dhcpserver/dhcpserver_options.h>
#include <lwip/dns.h>
#include <esp_netif_net_stack.h>
#if LWIP_IPV6 && LWIP_IPV6_DHCP6_STATELESS
#include <lwip/dhcp6.h>
#endif
#include <lwip/netif.h>
#include <esp_mac.h>

#ifdef CONFIG_ETH_ENABLED
#include <driver/gpio.h>
#include <esp_eth.h>
#include <esp_eth_phy.h>
#include <esp_eth_mac.h>
#include <esp_eth_com.h>
#include <soc/emac_ext_struct.h>
#include <soc/rtc.h>
#include <soc/io_mux_reg.h>
#endif

// 3rdparty lib includes
#include <fmt/core.h>
#include <strutils.h>
#include <delayedconstruction.h>
#include <wrappers/event_group.h>
#include <wrappers/queue.h>
#include <tickchrono.h>
#include <cpputils.h>
#include <cleanuphelper.h>

using namespace std::chrono_literals;

namespace wifi_stack {
namespace {
constexpr const char * const TAG = "WIFI_STACK";

std::optional<WiFiStaStatus> lastStatus;

WiFiState _wifiState;

std::optional<espchrono::millis_clock::time_point> _lastStaSwitchedFromConnected;
std::optional<espchrono::millis_clock::time_point> _lastStaSwitchedToConnected;

std::optional<espchrono::millis_clock::time_point> lastScanStarted;
espchrono::millis_clock::time_point _lastConnect;

std::optional<static_ip_config> last_sta_static_ip;
static_dns_config last_sta_static_dns;
std::optional<ap_config> last_ap_config;

#ifdef CONFIG_ETH_ENABLED
std::optional<static_ip_config> last_eth_static_ip;
static_dns_config last_eth_static_dns;
#endif

std::string lastWifisChecksum;

bool wasReallyScanning{};

constexpr auto AP_STARTED_BIT    = BIT0;
constexpr auto AP_HAS_IP6_BIT    = BIT1;
constexpr auto AP_HAS_CLIENT_BIT = BIT2;
constexpr auto STA_STARTED_BIT   = BIT3;
constexpr auto STA_CONNECTED_BIT = BIT4;
constexpr auto STA_HAS_IP_BIT    = BIT5;
constexpr auto STA_HAS_IP6_BIT   = BIT6;
#ifdef CONFIG_ETH_ENABLED
constexpr auto ETH_STARTED_BIT   = BIT7;
constexpr auto ETH_CONNECTED_BIT = BIT8;
constexpr auto ETH_HAS_IP_BIT    = BIT9;
constexpr auto ETH_HAS_IP6_BIT   = BIT10;
#endif
constexpr auto WIFI_SCANNING_BIT = BIT11;
constexpr auto WIFI_SCAN_DONE_BIT= BIT12;
constexpr auto WIFI_DNS_IDLE_BIT = BIT13;
constexpr auto WIFI_DNS_DONE_BIT = BIT14;

// generic
cpputils::DelayedConstruction<espcpputils::queue> wifi_event_queue;
cpputils::DelayedConstruction<espcpputils::event_group> wifi_event_group;
bool defaultEventLoopCreated{};
bool wifiEventRegistered{};
bool ipEventRegistered{};
#ifdef SMARTCONFIG
bool scEventRegistered{};
#endif
#ifdef CONFIG_ETH_ENABLED
bool ethEventRegistered{};
#endif
#ifdef PROVISIONING
bool wifiProvEventRegistered{};
#endif
} // namespace

esp_netif_t* esp_netifs[ESP_IF_MAX] = {NULL, NULL, NULL};
namespace {
bool _lowLevelInitDone = false;
bool _esp_wifi_started = false;
wifi_ps_type_t _sleepEnabled = WIFI_PS_MIN_MODEM;

// sta
std::atomic<WiFiStaStatus> _sta_status{WiFiStaStatus::NO_SHIELD};
std::optional<espchrono::millis_clock::time_point> _wifiConnectFailFlag;
uint8_t _wifiConnectFailCounter{};

std::optional<StaError> _last_sta_error;
std::string _last_sta_error_message;
std::optional<espchrono::millis_clock::time_point> _last_wifi_connect_failed;

// scan
std::optional<espchrono::millis_clock::time_point> scanStarted;
espchrono::milliseconds32 scanTimeout = 10s;
std::optional<scan_result> _scanResult;
bool scanResultChangedFlag{};

std::string _connectPlanWifisChecksum;
std::vector<mac_t> _pastConnectPlan;
mac_t _currentConnectPlanEntry;
std::vector<mac_t> _connectPlan;

#ifdef CONFIG_ETH_ENABLED
std::optional<tl::expected<void, std::string>> _eth_init_status;
#endif

} // namespace

const WiFiState &wifiStateMachineState{_wifiState};
cpputils::Signal<> scanResultChanged{};
const std::optional<espchrono::millis_clock::time_point> &lastStaSwitchedFromConnected{_lastStaSwitchedFromConnected};
const std::optional<espchrono::millis_clock::time_point> &lastStaSwitchedToConnected{_lastStaSwitchedToConnected};
const bool &esp_wifi_started{_esp_wifi_started};
const uint8_t &sta_error_count{_wifiConnectFailCounter};
const std::string &last_sta_error_message{_last_sta_error_message};
const std::optional<espchrono::millis_clock::time_point> &last_wifi_connect_failed{_last_wifi_connect_failed};
const std::optional<StaError> &last_sta_error{_last_sta_error};
const std::vector<mac_t> &pastConnectPlan{_pastConnectPlan};
const mac_t &currentConnectPlanEntry{_currentConnectPlanEntry};
const std::vector<mac_t> &connectPlan{_connectPlan};

#ifdef CONFIG_ETH_ENABLED
const std::optional<tl::expected<void, std::string>> &eth_init_status{_eth_init_status};
#endif

namespace {
#define WifiEventIdValues(x) \
    x(WIFI_READY) \
    x(WIFI_SCAN_DONE) \
    x(WIFI_STA_START) \
    x(WIFI_STA_STOP) \
    x(WIFI_STA_CONNECTED) \
    x(WIFI_STA_DISCONNECTED) \
    x(WIFI_STA_AUTHMODE_CHANGE) \
    x(WIFI_STA_GOT_IP) \
    x(WIFI_STA_GOT_IP6) \
    x(WIFI_STA_LOST_IP) \
    x(WIFI_AP_START) \
    x(WIFI_AP_STOP) \
    x(WIFI_AP_STACONNECTED) \
    x(WIFI_AP_STADISCONNECTED) \
    x(WIFI_AP_STAIPASSIGNED) \
    x(WIFI_AP_PROBEREQRECVED) \
    x(WIFI_AP_GOT_IP6) \
    x(ETH_START) \
    x(ETH_STOP) \
    x(ETH_CONNECTED) \
    x(ETH_DISCONNECTED) \
    x(ETH_GOT_IP) \
    x(ETH_GOT_IP6) \
    x(WPS_ER_SUCCESS) \
    x(WPS_ER_FAILED) \
    x(WPS_ER_TIMEOUT) \
    x(WPS_ER_PIN) \
    x(WPS_ER_PBC_OVERLAP) \
    x(MAX)
//#ifdef SMARTCONFIG
//    x(SC_SCAN_DONE)
//    x(SC_FOUND_CHANNEL)
//    x(SC_GOT_SSID_PSWD)
//    x(SC_SEND_ACK_DONE)
//#endif
//#ifdef PROVISIONING
//    x(PROV_INIT)
//    x(PROV_DEINIT)
//    x(PROV_START)
//    x(PROV_END)
//    x(PROV_CRED_RECV)
//    x(PROV_CRED_FAIL)
//    x(PROV_CRED_SUCCESS)
//#endif

DECLARE_TYPESAFE_ENUM(WifiEventId, : int32_t, WifiEventIdValues)

static_assert(sizeof(WifiEventId) == 4);

struct WifiEvent
{
    WifiEventId event_id;
    union
    {
        wifi_event_sta_scan_done_t wifi_scan_done;
        wifi_event_sta_authmode_change_t wifi_sta_authmode_change;
        wifi_event_sta_connected_t wifi_sta_connected;
        wifi_event_sta_disconnected_t wifi_sta_disconnected;
        wifi_event_sta_wps_er_pin_t wps_er_pin;
        wifi_event_sta_wps_fail_reason_t wps_fail_reason;
        wifi_event_ap_probe_req_rx_t wifi_ap_probereqrecved;
        wifi_event_ap_staconnected_t wifi_ap_staconnected;
        wifi_event_ap_stadisconnected_t wifi_ap_stadisconnected;
        ip_event_ap_staipassigned_t wifi_ap_staipassigned;
        ip_event_got_ip_t got_ip;
        ip_event_got_ip6_t got_ip6;
#ifdef SMARTCONFIG
        smartconfig_event_got_ssid_pswd_t sc_got_ssid_pswd;
#endif
#ifdef CONFIG_ETH_ENABLED
        esp_eth_handle_t eth_connected;
#endif
        wifi_sta_config_t prov_cred_recv;
#ifdef PROVISIONING
        wifi_prov_sta_fail_reason_t prov_fail_reason;
#endif
    };
};

#ifdef CONFIG_ETH_ENABLED
//#define ETH_PHY_IP101 ETH_PHY_TLK110

bool eth_initialized{};
bool eth_started{};
esp_eth_handle_t eth_handle{};
#endif

int wifi_set_status_bits(int bits);
int wifi_clear_status_bits(int bits);
int wifi_get_status_bits();
int wifi_wait_status_bits(int bits, espcpputils::ticks timeout);
esp_err_t wifi_set_esp_interface_ip(esp_interface_t interface, const std::optional<static_ip_config> &ip);
esp_err_t wifi_set_esp_interface_dns(esp_interface_t interface, const static_dns_config &dns);
esp_err_t wifi_sync_mode(const config &config);
template<size_t LENGTH>
size_t copyStrToBuf(uint8_t (&buf)[LENGTH], std::string_view str);
wifi_config_t make_ap_config(const ap_config &ap_config);
esp_err_t wifi_set_ap_config(const ap_config &ap_config);
void set_sta_status(WiFiStaStatus status);
void wifi_scan_done();
esp_err_t wifi_sta_disconnect(const config &config, bool eraseap = false);
esp_err_t wifi_sta_begin(const config &config, const sta_config &sta_config, std::string_view ssid,
                         const wifi_entry &wifi_entry, int32_t channel = 0, std::optional<mac_t> bssid = {}, bool connect = true);
esp_err_t wifi_sta_restart(const config &config);
void wifi_event_callback(const config &config, const WifiEvent &event);
esp_err_t wifi_post_event(std::unique_ptr<const WifiEvent> event);
void wifi_event_cb(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
esp_err_t wifi_start_network_event_task(const config &config);
esp_err_t wifi_low_level_init(const config &config);
esp_err_t wifi_start();
esp_err_t wifi_low_level_deinit();
esp_err_t wifi_stop();
tl::expected<void, std::string> applyBaseMac(const mac_t &mac);
tl::expected<mac_t, std::string> expectedBaseMac(const config &config);
esp_err_t wifi_set_ap_ip(const config &config, const static_ip_config &ip);
wifi_config_t make_sta_config(std::string_view ssid, std::string_view password, int8_t min_rssi,
                              std::optional<mac_t> bssid, uint8_t channel);
std::string calculateWifisChecksum(const sta_config &sta_config);
void setWifiState(WiFiState newWifiState);
bool buildConnectPlan(const config &config, const sta_config &sta_config);
bool buildConnectPlan(const config &config, const sta_config &sta_config, const scan_result &scanResult);
bool nextConnectPlanItem(const config &config, const sta_config &sta_config);
bool nextConnectPlanItem(const config &config, const sta_config &sta_config, const scan_result &scanResult);
void handleWifiEvents(const config &config, TickType_t xTicksToWait);
#ifdef CONFIG_ETH_ENABLED
tl::expected<void, std::string> eth_begin(const config &config, const eth_config &eth);
#endif
} // namespace

void init(const config &config)
{
    if (const auto mac = expectedBaseMac(config))
    {
        if (const auto result = applyBaseMac(*mac); result)
            ESP_LOGI(TAG, "applyBaseMac() %s succeeded", toString(*mac).c_str());
        else
            ESP_LOGE(TAG, "applyBaseMac() %s failed: %.*s", toString(*mac).c_str(), result.error().size(), result.error().data());
    }
    else
        ESP_LOGE(TAG, "expectedBaseMac() failed: %.*s", mac.error().size(), mac.error().data());

    if (const auto result = esp_netif_init(); result != ESP_OK)
        ESP_LOGE(TAG, "esp_netif_init() failed with %s", esp_err_to_name(result));

    if (const auto result = wifi_start_network_event_task(config); result != ESP_OK)
        ESP_LOGE(TAG, "wifi_start_network_event_task() failed with %s", esp_err_to_name(result));

    if (const auto result = wifi_sync_mode(config); result != ESP_OK)
        ESP_LOGE(TAG, "wifi_sync_mode() failed with %s", esp_err_to_name(result));

#ifdef CONFIG_ETH_ENABLED
    if (config.eth)
    {
        auto result = eth_begin(config, *config.eth);
        if (!result)
            ESP_LOGE(TAG, "eth_begin() failed with %.*s", result.error().size(), result.error().data());
        _eth_init_status = std::move(result);
    }
#endif

    if (config.ap)
    {
        ESP_LOGI(TAG, "AccessPoint %.*s", config.ap->ssid.size(), config.ap->ssid.data());

        if (const auto result = wifi_set_ap_ip(config, config.ap->static_ip); result != ESP_OK)
            ESP_LOGE(TAG, "wifi_set_ap_ip() failed with %s", esp_err_to_name(result));

        if (const auto result = wifi_set_ap_config(*config.ap); result != ESP_OK)
            ESP_LOGE(TAG, "wifi_set_ap_config() failed with %s", esp_err_to_name(result));
    }

#ifdef CONFIG_WIFI_DUAL_ANT
    if (config.dual_ant)
    {
        {
            const wifi_ant_gpio_config_t cfg {
                .gpio_cfg {
                    wifi_ant_gpio_t { .gpio_select = 1, .gpio_num = (uint8_t)config.dual_ant->selectPin0 },
                    wifi_ant_gpio_t { .gpio_select = 1, .gpio_num = (uint8_t)config.dual_ant->selectPin1 }
                }
            };

            if (const auto result = esp_wifi_set_ant_gpio(&cfg); result != ESP_OK)
                ESP_LOGE(TAG, "esp_wifi_set_ant_gpio() failed with %s", esp_err_to_name(result));
        }

        {
            wifi_ant_config_t config {
                .rx_ant_mode = WIFI_ANT_MODE_AUTO,
                .rx_ant_default = WIFI_ANT_ANT0,
                .tx_ant_mode = WIFI_ANT_MODE_AUTO,
                .enabled_ant0 = 0b0001,
                .enabled_ant1 = 0b0010
            };

            if (const auto result = esp_wifi_set_ant(&config); result != ESP_OK)
                ESP_LOGE(TAG, "esp_wifi_set_ant() failed with %s", esp_err_to_name(result));
        }
    }
#endif

    last_ap_config = config.ap;

    _wifiState = WiFiState::None;

    if (config.sta)
    {
        lastWifisChecksum = calculateWifisChecksum(*config.sta);
        if (const auto result = begin_scan(*config.sta); !result)
            ESP_LOGE(TAG, "begin_scan() failed with: %.*s", result.error().size(), result.error().data());
    }
    else
        ESP_LOGW(TAG, "not scanning, because wifi is not enabled");
}

void update(const config &config)
{
    handleWifiEvents(config, 0);

    if (const auto expected = expectedBaseMac(config))
    {
        if (const auto actual = get_base_mac_addr())
        {
            if (*expected != *actual)
            {
                if (const auto result = applyBaseMac(*expected))
                    ESP_LOGI(TAG, "changed base mac from %s to %s", toString(*actual).c_str(), toString(*expected).c_str());
                else
                    ESP_LOGE(TAG, "applyBaseMac() %s failed: %.*s", toString(*expected).c_str(), result.error().size(), result.error().data());
            }
        }
        else
            ESP_LOGE(TAG, "get_base_mac_addr() failed: %.*s", actual.error().size(), actual.error().data());
    }
    else
        ESP_LOGE(TAG, "expectedBaseMac() failed: %.*s", expected.error().size(), expected.error().data());

    if (const auto result = wifi_sync_mode(config); result != ESP_OK)
        ESP_LOGE(TAG, "wifi_sync_mode() failed with %s", esp_err_to_name(result));

    if (last_ap_config != config.ap)
    {
        if (last_ap_config && config.ap)
        {
            ESP_LOGI(TAG, "AP settings changed, applying new config...");

            if (config.ap->ssid != last_ap_config->ssid)
                ESP_LOGI(TAG, "new ap ssid=\"%s\" old ap ssid=\"%s\"", config.ap->ssid.c_str(), last_ap_config->ssid.c_str());

            if (config.ap->key != last_ap_config->key)
                ESP_LOGI(TAG, "new ap key=\"%s\" old ap key=\"%s\"", config.ap->key.c_str(), last_ap_config->key.c_str());

            if (const auto result = wifi_set_ap_ip(config, config.ap->static_ip); result != ESP_OK)
                ESP_LOGE(TAG, "wifi_set_ap_ip() failed with %s", esp_err_to_name(result));

            if (const auto result = wifi_set_ap_config(*config.ap); result != ESP_OK)
                ESP_LOGE(TAG, "wifi_set_ap_config() failed with %s", esp_err_to_name(result));
        }
        else if (!last_ap_config && config.ap)
        {
            ESP_LOGI(TAG, "AP enabled");

            if (const auto result = wifi_set_ap_ip(config, config.ap->static_ip); result != ESP_OK)
                ESP_LOGE(TAG, "wifi_set_ap_ip() failed with %s", esp_err_to_name(result));

            if (const auto result = wifi_set_ap_config(*config.ap); result != ESP_OK)
                ESP_LOGE(TAG, "wifi_set_ap_config() failed with %s", esp_err_to_name(result));
        }
        else if (last_ap_config && !config.ap)
        {
            ESP_LOGI(TAG, "AP disabled");
        }

        last_ap_config = config.ap;
    }

    if (scanResultChangedFlag)
    {
        scanResultChangedFlag = false;
        scanResultChanged();
    }

    if (config.sta)
    {
        if (_wifiState == WiFiState::None)
        {
            if (get_sta_status() == WiFiStaStatus::CONNECTED)
            {
                ESP_LOGI(TAG, "Unexpected connected!");

                setWifiState(WiFiState::Connected);
            }
            else
            {
                const auto anyWifiConfigured = std::any_of(std::begin(config.sta->wifis), std::end(config.sta->wifis),
                                                           [](const auto &entry){ return !entry.ssid.empty(); });
                if (anyWifiConfigured)
                {
                    if (!lastScanStarted || (config.sta->scan.interval && espchrono::ago(*lastScanStarted) >= *config.sta->scan.interval))
                    {
                        if (const auto result = begin_scan(*config.sta); !result)
                            ESP_LOGE(TAG, "begin_scan() failed: %.*s", result.error().size(), result.error().data());
                    }
                    else if (auto newChecksum = calculateWifisChecksum(*config.sta); newChecksum != lastWifisChecksum)
                    {
                        ESP_LOGI(TAG, "old wifis config: %s", lastWifisChecksum.c_str());
                        ESP_LOGI(TAG, "new wifis config: %s", newChecksum.c_str());
                        lastWifisChecksum = std::move(newChecksum);

                        if (get_scan_status() == WiFiScanStatus::Finished)
                        {
                            if (const auto &scanResult = get_scan_result(); scanResult && !scanResult->entries.empty())
                            {
                                ESP_LOGI(TAG, "wifi configs changed, building connect plan...");
                                buildConnectPlan(config, *config.sta, *scanResult);
                            }
                            else
                                goto scanAnyways;
                        }
                        else
                        {
                            scanAnyways:
                            ESP_LOGI(TAG, "wifi configs changed, triggering a new scan");
                            if (const auto result = begin_scan(*config.sta); !result)
                                ESP_LOGE(TAG, "begin_scan() failed: %.*s", result.error().size(), result.error().data());
                        }
                    }
                }
            }
        }

        if (_wifiState == WiFiState::Scanning)
        {
            if (const auto result = get_scan_status(); result != WiFiScanStatus::Scanning)
            {
                if (wasReallyScanning)
                {
                    wasReallyScanning = false;
                }

                if (result == WiFiScanStatus::Finished)
                {
                    if (const auto &scanResult = get_scan_result(); !scanResult)
                    {
                        ESP_LOGE(TAG, "unexpected no scan result");
                        setWifiState(WiFiState::None);
                        lastStatus = std::nullopt;
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Finished scan with %zd results", scanResult->entries.size());

                        if (get_sta_status() != WiFiStaStatus::CONNECTED)
                        {
                            ESP_LOGI(TAG, "Not connected after scan, building connect plan...");
                            buildConnectPlan(config, *config.sta, *scanResult);
                        }
                        else
                        {
                            ESP_LOGI(TAG, "connected after scan, nothing to do...");
                            setWifiState(WiFiState::Connected);
                            lastStatus = std::nullopt;
                        }
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "scan failed with %s", toString(result).c_str());
                    //setWifiState(WiFiState::None); // results in another scan
                }
            }
        }

        if (_wifiState == WiFiState::Connecting)
        {
            const auto status = get_sta_status();
            if (status == WiFiStaStatus::CONNECTED)
            {
                if (const auto interf = esp_netifs[ESP_IF_WIFI_STA])
                {
                    esp_netif_ip_info_t ip;
                    if (const auto result = esp_netif_get_ip_info(interf, &ip); result == ESP_OK)
                    {
                        ESP_LOGI(TAG, "successfully connected WiFi! %s", toString(ip.ip).c_str());
                    }
                    else
                    {
                        ESP_LOGI(TAG, "successfully connected WiFi!");
                        ESP_LOGW(TAG, "esp_netif_get_ip_info() for STA failed with %s", esp_err_to_name(result));
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "successfully connected WiFi!");
                    ESP_LOGW(TAG, "STA interface is null");
                }

                setWifiState(WiFiState::Connected);
                lastStatus = std::nullopt;
            }
            else if (_wifiConnectFailFlag && espchrono::ago(*_wifiConnectFailFlag) < 5s)
            {
                ESP_LOGD(TAG, "clearing connect fail flag");
                _wifiConnectFailFlag = std::nullopt;

                if (auto newConnectPlanWifisChecksum = calculateWifisChecksum(*config.sta);
                    _connectPlanWifisChecksum != newConnectPlanWifisChecksum)
                {
                    ESP_LOGI(TAG, "wifi configs changed, building new connect plan...");

                    if (const auto result = wifi_sta_disconnect(config, true); result != ESP_OK)
                        ESP_LOGE(TAG, "wifi_sta_disconnect() failed with %s", esp_err_to_name(result));

                    setWifiState(WiFiState::None); // results in another scan

                    buildConnectPlan(config, *config.sta);
                }
                else if (_wifiConnectFailCounter++ >= 10)
                {
                    ESP_LOGE(TAG, "fail flag was set and fail count exceeded limit");

                    if (const auto result = wifi_sta_disconnect(config, true); result != ESP_OK)
                        ESP_LOGE(TAG, "wifi_sta_disconnect() failed with %s", esp_err_to_name(result));

                    setWifiState(WiFiState::None); // results in another scan
                }
                else
                {
                    ESP_LOGW(TAG, "fail flag was set, trying again %hhu", _wifiConnectFailCounter);

                    _lastConnect = espchrono::millis_clock::now();

                    if (const auto result = wifi_sta_disconnect(config); result != ESP_OK)
                        ESP_LOGE(TAG, "wifi_sta_disconnect() failed with %s", esp_err_to_name(result));

                    if (const auto result = wifi_sta_restart(config); result != ESP_OK)
                        ESP_LOGE(TAG, "wifi_sta_restart() failed with %s", esp_err_to_name(result));
                }
            }
            else
            {
                if (espchrono::ago(_lastConnect) >= 20s)
                {
                    if (_wifiConnectFailCounter++ >= 10)
                    {
                        ESP_LOGE(TAG, "connecting timed out, fail flag was set and fail count exceeded limit");

                        if (const auto result = wifi_sta_disconnect(config, true); result != ESP_OK)
                            ESP_LOGE(TAG, "wifi_sta_disconnect() failed with %s", esp_err_to_name(result));

                        setWifiState(WiFiState::None); // results in another scan
                    }
                    else
                    {
                        ESP_LOGW(TAG, "connecting timed out, building new connect plan... %s %hhu", toString(status).c_str(), _wifiConnectFailCounter);

                        if (const auto result = wifi_sta_disconnect(config, true); result != ESP_OK)
                            ESP_LOGE(TAG, "wifi_sta_disconnect() failed with %s", esp_err_to_name(result));

                        buildConnectPlan(config, *config.sta);

                        lastStatus = std::nullopt;
                    }
                }
                else if (!lastStatus || *lastStatus != status)
                {
                    ESP_LOGI(TAG, "connecting: %s", toString(status).c_str());
                    lastStatus = status;
                }
            }
        }

        if (_wifiState == WiFiState::Connected)
        {
            const auto status = get_sta_status();
            if (status != WiFiStaStatus::CONNECTED)
            {
                ESP_LOGW(TAG, "lost connection: %s", toString(status).c_str());
                if (const auto result = wifi_sta_disconnect(config, true); result != ESP_OK)
                    ESP_LOGE(TAG, "wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
                lastWifisChecksum.clear();
                setWifiState(WiFiState::None); // results in another scan
            }
            else if (const auto sta_info = get_sta_ap_info())
            {
                const std::string_view connectedSSID{reinterpret_cast<const char *>(sta_info->ssid)};
                const auto iter = std::find_if(std::cbegin(config.sta->wifis), std::cend(config.sta->wifis),
                                              [&connectedSSID](const wifi_entry &entry){
                                                  return cpputils::stringEqualsIgnoreCase(entry.ssid, connectedSSID);
                                              });

                if (iter == std::cend(config.sta->wifis))
                {
                    ESP_LOGI(TAG, "disconnecting, because cannot find ssid in config anymore");
                    lastWifisChecksum.clear();
                    if (const auto result = wifi_sta_disconnect(config, true); result != ESP_OK)
                        ESP_LOGE(TAG, "wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
                    ESP_LOGI(TAG, "status after disconnect: %s", toString(get_sta_status()).c_str());

                    setWifiState(WiFiState::None); // results in another scan
                }
                else
                {
                    if (last_sta_static_ip != iter->static_ip ||
                        last_sta_static_dns != iter->static_dns)
                    {
                        ESP_LOGI(TAG, "STA static ip/dns config changed, applying new config");

                        if (const auto result = wifi_set_esp_interface_ip(ESP_IF_WIFI_STA, iter->static_ip); result != ESP_OK)
                        {
                            ESP_LOGE(TAG, "wifi_set_esp_interface_ip() for STA failed with %s", esp_err_to_name(result));
                            //return result;
                        }
                        else
                        {
                            last_sta_static_ip = iter->static_ip;

                            if (const auto result = wifi_set_esp_interface_dns(ESP_IF_WIFI_STA, iter->static_dns); result != ESP_OK)
                            {
                                ESP_LOGE(TAG, "wifi_set_esp_interface_dns() for STA failed with %s", esp_err_to_name(result));
                                //return result;
                            }
                            else
                                last_sta_static_dns = iter->static_dns;
                        }
                    }
                }
            }
            else
                ESP_LOGE(TAG, "get_sta_ap_info() failed with %.*s", sta_info.error().size(), sta_info.error().data());
        }
    }
    else
    {
        if (_wifiState == WiFiState::Scanning)
        {
            if (get_scan_status() != WiFiScanStatus::Scanning)
                setWifiState(WiFiState::None);
        }

        if (!cpputils::is_in(get_sta_status(), WiFiStaStatus::NO_SHIELD, WiFiStaStatus::IDLE_STATUS, WiFiStaStatus::DISCONNECTED) ||
            !cpputils::is_in(_wifiState, WiFiState::None, WiFiState::Scanning))
        {
            ESP_LOGI(TAG, "disconnecting, because wifi_enabled is false");

            if (const auto result = wifi_sta_disconnect(config, true); result != ESP_OK)
                ESP_LOGE(TAG, "wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
            lastWifisChecksum.clear();
            setWifiState(WiFiState::None);
        }
    }

#ifdef CONFIG_ETH_ENABLED
    if (eth_initialized)
    {
        bool justStarted{};
        if (config.eth && !eth_started)
        {
            if (const auto result = esp_eth_start(eth_handle); result != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_eth_start() failed with %s", esp_err_to_name(result));
                // return result;
            }
            else
            {
                eth_started = true;
                justStarted = true;
            }
        }
        else if (!config.eth && eth_started)
        {
            if (const auto result = esp_eth_stop(eth_handle); result != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_eth_stop() failed with %s", esp_err_to_name(result));
                // return result;
            }
            else
            {
                eth_started = false;
            }
        }

        if (config.eth && eth_started && (
                    justStarted ||
                    last_eth_static_ip != config.eth->static_ip ||
                    last_eth_static_dns != config.eth->static_dns
                ))
        {
            if (!justStarted)
                ESP_LOGI(TAG, "ETH static ip/dns config changed, applying new config");

            if (const auto result = wifi_set_esp_interface_ip(ESP_IF_ETH, config.eth->static_ip); result != ESP_OK)
            {
                ESP_LOGE(TAG, "wifi_set_esp_interface_ip() for ETH failed with %s", esp_err_to_name(result));
                //return result;
            }
            else
            {
                last_eth_static_ip = config.eth->static_ip;

                if (const auto result = wifi_set_esp_interface_dns(ESP_IF_ETH, config.eth->static_dns); result != ESP_OK)
                {
                    ESP_LOGE(TAG, "wifi_set_esp_interface_dns() for ETH failed with %s", esp_err_to_name(result));
                    //return result;
                }
                else
                    last_eth_static_dns = config.eth->static_dns;
            }
        }
    }
#endif
}

wifi_mode_t get_wifi_mode()
{
    if (!_lowLevelInitDone || !_esp_wifi_started)
        return WIFI_MODE_NULL;

    wifi_mode_t mode;
    if (const auto result = esp_wifi_get_mode(&mode); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_get_mode() returned %s", esp_err_to_name(result));
        return WIFI_MODE_NULL;
    }

    return mode;
}

WiFiStaStatus get_sta_status()
{
    return _sta_status.load();
}

tl::expected<void, std::string> begin_scan(const sta_config &sta_config)
{
    if (!(get_wifi_mode() & WIFI_MODE_STA))
        return tl::make_unexpected("STA mode missing");

    if (wifi_get_status_bits() & WIFI_SCANNING_BIT)
        return tl::make_unexpected("already scanning");

    delete_scan_result();

    wifi_scan_config_t scan_config;
    scan_config.ssid = 0;
    scan_config.bssid = 0;
    scan_config.channel = sta_config.scan.channel;
    scan_config.show_hidden = sta_config.scan.show_hidden;

    if (std::holds_alternative<sta_active_scan_config>(sta_config.scan.time))
    {
        scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;

        const auto &time = std::get<sta_active_scan_config>(sta_config.scan.time);
        scan_config.scan_time.active.min = time.min_per_chan.count();
        scan_config.scan_time.active.max = time.max_per_chan.count();
        scanTimeout = time.max_per_chan * 20;
    }
    else if (std::holds_alternative<sta_passive_scan_config>(sta_config.scan.time))
    {
        scan_config.scan_type = WIFI_SCAN_TYPE_PASSIVE;

        const auto &time = std::get<sta_passive_scan_config>(sta_config.scan.time);
        scan_config.scan_time.passive = time.max_per_chan.count();
        scanTimeout = time.max_per_chan * 20;
    }
    else
        return tl::make_unexpected("invalid scan settings (not active nor passive)!");

    if (const auto result = esp_wifi_scan_start(&scan_config, false) != ESP_OK)
        return tl::make_unexpected(fmt::format("esp_wifi_scan_start() failed with: {}", esp_err_to_name(result)));

    scanStarted = espchrono::millis_clock::now();

    wifi_clear_status_bits(WIFI_SCAN_DONE_BIT);
    wifi_set_status_bits(WIFI_SCANNING_BIT);

    lastScanStarted = espchrono::millis_clock::now();
    if (_wifiState != WiFiState::Connected)
        setWifiState(WiFiState::Scanning);
    wasReallyScanning = true;

    return {};
}

WiFiScanStatus get_scan_status()
{
    //Check is scan was started and if the delay expired, return WIFI_SCAN_FAILED in this case
    if (scanStarted && espchrono::ago(*scanStarted) > scanTimeout)
    {
        wifi_clear_status_bits(WIFI_SCANNING_BIT);
        return WiFiScanStatus::Failed;
    }

    if (wifi_get_status_bits() & WIFI_SCAN_DONE_BIT)
        return WiFiScanStatus::Finished;

    if (wifi_get_status_bits() & WIFI_SCANNING_BIT)
        return WiFiScanStatus::Scanning;

    return WiFiScanStatus::None;
}

const std::optional<scan_result> &get_scan_result()
{
    // TODO construct new container using esp_wifi_scan_get_ap_num() and esp_wifi_scan_get_ap_records()
    return _scanResult;
}

void delete_scan_result()
{
    if (_scanResult)
    {
        _scanResult = std::nullopt;
        scanResultChanged();
    }
}

tl::expected<wifi_ap_record_t, std::string> get_sta_ap_info()
{
    wifi_ap_record_t info;
    if (const auto result = esp_wifi_sta_get_ap_info(&info); result == ESP_OK)
        return info;
    else
    {
        ESP_LOGW(TAG, "esp_wifi_sta_get_ap_info() failed with %s", esp_err_to_name(result));
        return tl::make_unexpected(fmt::format("esp_wifi_sta_get_ap_info() failed with {}", esp_err_to_name(result)));
    }
}

mac_or_error get_mac_addr(wifi_interface_t ifx)
{
    wifi_stack::mac_t mac;
    if (const auto result = esp_wifi_get_mac(ifx, std::begin(mac)); result == ESP_OK)
        return mac;
    else
    {
        ESP_LOGW(TAG, "esp_wifi_get_mac() failed with %s", esp_err_to_name(result));
        return tl::make_unexpected(fmt::format("esp_wifi_get_mac() failed with {}", esp_err_to_name(result)));
    }
}

mac_or_error get_default_mac_addr()
{
    static const mac_or_error cachedResult = []() -> mac_or_error {
        mac_t mac{};
        if (const auto result = esp_efuse_mac_get_default(std::begin(mac)); result == ESP_OK)
            return mac;
        else
        {
            //ESP_LOGE(TAG, "esp_efuse_mac_get_default() failed with %s", esp_err_to_name(result));
            return tl::make_unexpected(fmt::format("esp_efuse_mac_get_default() failed with {}", esp_err_to_name(result)));
        }
    }();

    return cachedResult;
}

mac_or_error get_custom_mac_addr()
{
    static const mac_or_error cachedResult = []() -> mac_or_error {
        mac_t mac{};
        if (const auto result = esp_efuse_mac_get_custom(std::begin(mac)); result == ESP_OK)
            return mac;
        else
        {
            //ESP_LOGE(TAG, "esp_efuse_mac_get_custom() failed with %s", esp_err_to_name(result));
            return tl::make_unexpected(fmt::format("esp_efuse_mac_get_custom() failed with {}", esp_err_to_name(result)));
        }
    }();

    return cachedResult;
}

mac_or_error get_base_mac_addr()
{
    mac_t mac{};
    if (const auto result = esp_base_mac_addr_get(std::begin(mac)); result == ESP_OK)
        return mac;
    else
    {
        ESP_LOGE(TAG, "esp_base_mac_addr_get() failed with %s", esp_err_to_name(result));
        return tl::make_unexpected(fmt::format("esp_base_mac_addr_get() failed with {}", esp_err_to_name(result)));
    }
}

tl::expected<void, std::string> set_base_mac_addr(mac_t mac_addr)
{
    if (const auto result = esp_base_mac_addr_set(std::cbegin(mac_addr)); result == ESP_OK)
        return {};
    else
    {
        ESP_LOGE(TAG, "esp_base_mac_addr_set() failed with %s", esp_err_to_name(result));
        return tl::make_unexpected(fmt::format("esp_base_mac_addr_set() failed with {}", esp_err_to_name(result)));
    }
}

tl::expected<esp_netif_ip_info_t, std::string> get_ip_info(esp_netif_t *esp_netif)
{
    esp_netif_ip_info_t ip;
    if (const auto result = esp_netif_get_ip_info(esp_netif, &ip); result == ESP_OK)
        return ip;
    else
    {
        ESP_LOGE(TAG, "esp_netif_get_ip_info() failed with %s", esp_err_to_name(result));
        return tl::make_unexpected(fmt::format("tcpip_adapter_get_ip_info() failed with {}", esp_err_to_name(result)));
    }
}

tl::expected<std::string_view, std::string> get_hostname_for_interface(esp_interface_t interf)
{
    if (const auto netif = esp_netifs[interf])
        return get_hostname_for_interface(netif);
    else
        return tl::make_unexpected(fmt::format("netif for {} is invalid", std::to_underlying(interf)));
}

tl::expected<std::string_view, std::string> get_hostname_for_interface(esp_netif_t *esp_netif)
{
    const char *hostname{};
    if (const auto result = esp_netif_get_hostname(esp_netif, &hostname))
        return tl::make_unexpected(fmt::format("esp_netif_get_hostname() failed with {}", esp_err_to_name(result)));

    if (!hostname)
        return tl::make_unexpected("esp_netif_get_hostname() returned a nullptr string");

    return std::string_view{hostname};
}

#ifdef CONFIG_ETH_ENABLED
esp_eth_handle_t getEthHandle()
{
    return eth_handle;
}

bool get_eth_connected()
{
    return wifi_get_status_bits() & ETH_CONNECTED_BIT;
}

bool get_eth_has_ip()
{
    return wifi_get_status_bits() & ETH_HAS_IP_BIT;
}
#endif

namespace {
int wifi_set_status_bits(int bits)
{
    if (!wifi_event_group.constructed())
        return 0;

    return wifi_event_group->setBits(bits);
}

int wifi_clear_status_bits(int bits)
{
    if (!wifi_event_group.constructed())
        return 0;

    return wifi_event_group->clearBits(bits);
}

int wifi_get_status_bits()
{
    if (!wifi_event_group.constructed())
        return 0;

    return wifi_event_group->getBits();
}

int wifi_wait_status_bits(int bits, espcpputils::ticks timeout)
{
    if (!wifi_event_group.constructed())
        return 0;

    return wifi_event_group->waitBits(
        bits,  // The bits within the event group to wait for.
        pdFALSE,         // BIT_0 and BIT_4 should be cleared before returning.
        pdTRUE,        // Don't wait for both bits, either bit will do.
        timeout.count()) & bits; // Wait a maximum of 100ms for either bit to be set.
}

esp_err_t wifi_set_esp_interface_ip(esp_interface_t interface, const std::optional<static_ip_config> &ip)
{
    using wifi_stack::toString;

    if (ip)
        ESP_LOGI(TAG, "%s set STATIC ip=%s subnet=%s gateway=%s", toString(interface).c_str(),
                 toString(ip->ip).c_str(), toString(ip->subnet).c_str(), toString(ip->gateway).c_str());
    else
        ESP_LOGI(TAG, "%s set DYNAMIC", toString(interface).c_str());

    if (interface == ESP_IF_WIFI_AP)
    {
        ESP_LOGE(TAG, "setting IP for AP must be done with wifi_set_ap_ip() instead!");
        return ESP_FAIL;
    }

    esp_netif_t * const esp_netif = esp_netifs[interface];
    if (!esp_netif)
    {
        ESP_LOGE(TAG, "netif for %i is invalid", std::to_underlying(interface));
        return ESP_FAIL;
    }

    {
        esp_netif_dhcp_status_t status;
        if (const auto result = esp_netif_dhcpc_get_status(esp_netif, &status); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_dhcpc_get_status() failed with %s", esp_err_to_name(result));
            return result;
        }
        ESP_LOGI(TAG, "esp_netif_dhcpc_get_status() resulted in %s", toString(status).c_str());
    }

    if (const auto result = esp_netif_dhcpc_stop(esp_netif); !cpputils::is_in(result, ESP_OK, ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED))
    {
        ESP_LOGE(TAG, "esp_netif_dhcpc_stop() failed with %s", esp_err_to_name(result));
        return result;
    }

    {
        esp_netif_ip_info_t info;
        info.ip.addr = ip ? ip->ip.value() : 0;
        info.netmask.addr = ip ? ip->subnet.value() : 0;
        info.gw.addr = ip ? ip->gateway.value() : 0;

        if (const auto result = esp_netif_set_ip_info(esp_netif, &info); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_set_ip_info() failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    if (!ip)
    {
        ESP_LOGI(TAG, "starting dhcpc");

        if (const auto result = esp_netif_dhcpc_start(esp_netif); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_dhcpc_start() failed with %s", esp_err_to_name(result));
            return result;
        }

#if LWIP_IPV6 && LWIP_IPV6_DHCP6_STATELESS
        if (const auto result = dhcp6_enable_stateless(esp_netif_get_netif_impl(esp_netif)); result != ESP_OK)
        {
            ESP_LOGE(TAG, "dhcp6_enable_stateless() failed with %s", esp_err_to_name(result));
            return result;
        }
#endif
    }

    return ESP_OK;
}

esp_err_t wifi_set_esp_interface_dns(esp_interface_t interface, const static_dns_config &dns)
{
    esp_netif_t *esp_netif = esp_netifs[interface];
    if (!esp_netif)
    {
        ESP_LOGE(TAG, "netif for %i is invalid", std::to_underlying(interface));
        return ESP_FAIL;
    }

    esp_netif_dns_info_t dns_info;
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;

    if (dns.main)
    {
        dns_info.ip.u_addr.ip4.addr = dns.main->value();
        if (const auto result = esp_netif_set_dns_info(esp_netif, ESP_NETIF_DNS_MAIN, &dns_info); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_set_dns_info() for MAIN failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    if (dns.backup)
    {
        if (interface != ESP_IF_WIFI_AP)
        {
            dns_info.ip.u_addr.ip4.addr = dns.backup->value();
            if (const auto result = esp_netif_set_dns_info(esp_netif, ESP_NETIF_DNS_BACKUP, &dns_info); result != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_netif_set_dns_info() for BACKUP failed with %s", esp_err_to_name(result));
                return result;
            }
        }
        else
            ESP_LOGE(TAG, "setting static backup DNS for AP is not allowed!");
    }

    if (dns.fallback)
    {
        if (interface != ESP_IF_WIFI_AP)
        {
            dns_info.ip.u_addr.ip4.addr = dns.fallback->value();
            if (const auto result = esp_netif_set_dns_info(esp_netif, ESP_NETIF_DNS_FALLBACK, &dns_info); result != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_netif_set_dns_info() for FALLBACK failed with %s", esp_err_to_name(result));
                return result;
            }
        }
        else
            ESP_LOGE(TAG, "setting static fallback DNS for AP is not allowed!");
    }

    return ESP_OK;
}

template<size_t LENGTH>
size_t copyStrToBuf(uint8_t (&buf)[LENGTH], std::string_view str)
{
    size_t cutLength = std::min(LENGTH, str.size());
    std::copy(std::begin(str), std::begin(str) + cutLength, buf);
    if (str.size() < LENGTH)
        buf[str.size()] = '\0';
    return cutLength;
}

wifi_config_t make_ap_config(const ap_config &ap_config)
{
    wifi_config_t wifi_config;
    wifi_config.ap.channel = ap_config.channel;
    wifi_config.ap.max_connection = ap_config.max_connection;
    wifi_config.ap.beacon_interval = ap_config.beacon_interval;
    wifi_config.ap.ssid_hidden = ap_config.ssid_hidden;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    wifi_config.ap.ssid_len = 0;
    wifi_config.ap.ssid[0] = 0;
    wifi_config.ap.password[0] = 0;
    if (!ap_config.ssid.empty())
    {
        auto ssidCutLength = copyStrToBuf(wifi_config.ap.ssid, ap_config.ssid);
        wifi_config.ap.ssid_len = ssidCutLength;

        if (!ap_config.key.empty() && ap_config.authmode != WIFI_AUTH_OPEN)
        {
            wifi_config.ap.authmode = ap_config.authmode;
            copyStrToBuf(wifi_config.ap.password, ap_config.key);
        }
    }
    return wifi_config;
}

esp_err_t wifi_set_ap_config(const ap_config &ap_config)
{
    if (ap_config.ssid.empty())
    {
        ESP_LOGE(TAG, "SSID missing!");
        return ESP_FAIL;
    }

    if (ap_config.ssid.size() > 32)
    {
        ESP_LOGE(TAG, "SSID too long! (size=%zd)", ap_config.ssid.size());
        return ESP_FAIL;
    }

    if (!ap_config.key.empty())
    {
        if (ap_config.key.size() < 8)
        {
            ESP_LOGE(TAG, "passphrase too short! (size=%zd)", ap_config.key.size());
            return ESP_FAIL;
        }
        if (ap_config.key.size() > 64)
        {
            ESP_LOGE(TAG, "passphrase too long! (size=%zd)", ap_config.key.size());
            return ESP_FAIL;
        }
    }

    wifi_config_t conf = make_ap_config(ap_config);

    wifi_config_t conf_current;
    if (const auto result = esp_wifi_get_config(WIFI_IF_AP, &conf_current); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_get_config() for AP failed with %s", esp_err_to_name(result));
        return result;
    }

    if (!wifi_ap_config_equal(conf.ap, conf_current.ap))
    {
        if (const auto result = esp_wifi_set_config(WIFI_IF_AP, &conf); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_set_config() for AP failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    return ESP_OK;
}

void set_sta_status(WiFiStaStatus status)
{
    WiFiStaStatus oldStatus = _sta_status.exchange(status);

    ESP_LOGI(TAG, "%s (from %s)", toString(status).c_str(), toString(oldStatus).c_str());

    if (oldStatus == WiFiStaStatus::CONNECTED && status != WiFiStaStatus::CONNECTED)
        _lastStaSwitchedFromConnected = espchrono::millis_clock::now();
    else if (oldStatus != WiFiStaStatus::CONNECTED && status == WiFiStaStatus::CONNECTED)
        _lastStaSwitchedToConnected = espchrono::millis_clock::now();
}

void wifi_scan_done()
{
    _scanResult = std::nullopt; // free memory before allocating the new vector below

    uint16_t scanCount;
    if (const auto result = esp_wifi_scan_get_ap_num(&scanCount); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_scan_get_ap_num() failed with %s", esp_err_to_name(result));
        goto cleanup;
    }

    if (scanCount)
    {
        scan_result newResult;
        newResult.entries.resize(scanCount);

        if (const auto result = esp_wifi_scan_get_ap_records(&scanCount, &(*std::begin(newResult.entries))); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_scan_get_ap_records() failed with %s", esp_err_to_name(result));
            goto cleanup;
        }

        std::sort(std::begin(newResult.entries), std::end(newResult.entries), [](const wifi_ap_record_t &l, const wifi_ap_record_t &r){
            return l.rssi > r.rssi;
        });

        newResult.finished = espchrono::millis_clock::now();

        _scanResult = std::move(newResult);
    }

cleanup:
    scanResultChangedFlag = true;
    scanStarted = std::nullopt; //Reset after a scan is completed for normal behavior

    wifi_set_status_bits(WIFI_SCAN_DONE_BIT);
    wifi_clear_status_bits(WIFI_SCANNING_BIT);
}

void wifi_event_callback(const config &config, const WifiEvent &event)
{
    ESP_LOGD(TAG, "%d %s", int(event.event_id), toString(event.event_id).c_str());

    switch (event.event_id)
    {
    case WifiEventId::WIFI_SCAN_DONE:
        wifi_scan_done();
        break;
    case WifiEventId::WIFI_STA_START:
        set_sta_status(WiFiStaStatus::IDLE_STATUS);
        wifi_set_status_bits(STA_STARTED_BIT);
        if (const auto result = esp_wifi_set_ps(_sleepEnabled); result != ESP_OK)
            ESP_LOGE(TAG, "esp_wifi_set_ps() failed with %s", esp_err_to_name(result));
        break;
    case WifiEventId::WIFI_STA_STOP:
        set_sta_status(WiFiStaStatus::NO_SHIELD);
        wifi_clear_status_bits(STA_STARTED_BIT | STA_CONNECTED_BIT | STA_HAS_IP_BIT | STA_HAS_IP6_BIT);
        break;
    case WifiEventId::WIFI_STA_CONNECTED:
        set_sta_status(WiFiStaStatus::WAITING_FOR_IP);
        wifi_set_status_bits(STA_CONNECTED_BIT);
        esp_netif_create_ip6_linklocal(esp_netifs[ESP_IF_WIFI_STA]);
        break;
    case WifiEventId::WIFI_STA_DISCONNECTED:
    {
        const std::string_view ssid {
            (const char *)event.wifi_sta_disconnected.ssid,
            event.wifi_sta_disconnected.ssid_len
        };

        const auto reason = wifi_err_reason_t(event.wifi_sta_disconnected.reason);
        {
            const mac_t bssid{event.wifi_sta_disconnected.bssid};

            _last_sta_error = StaError {
                .ssid = std::string{ssid},
                .bssid = bssid,
                .reason = reason
            };

            auto msg = fmt::format("{} WIFI_STA_DISCONNECTED ssid=\"{}\" bssid={} reason={}({})",
                                   espchrono::millis_clock::now().time_since_epoch().count(),
                                   ssid, toString(bssid),
                                   std::to_underlying(reason), wifi_stack::toString(reason));
            ESP_LOGW(TAG, "%.*s", msg.size(), msg.data());
            _last_sta_error_message += msg;
            _last_sta_error_message += '\n';
            if (_last_sta_error_message.size() > 512)
                _last_sta_error_message = _last_sta_error_message.substr(_last_sta_error_message.size() - 512, 512);
        }

        switch (reason)
        {
        case WIFI_REASON_AUTH_EXPIRE:
//            break;
        case WIFI_REASON_NO_AP_FOUND:
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_ASSOC_FAIL:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_BEACON_TIMEOUT:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
        default:
        {
            const auto sta_status = get_sta_status();
            if (sta_status != WiFiStaStatus::DISCONNECTING)
            {
                ESP_LOGD(TAG, "setting fail flag");
                _last_wifi_connect_failed = espchrono::millis_clock::now();
                _wifiConnectFailFlag = espchrono::millis_clock::now();
            }
            switch (sta_status)
            {
            case WiFiStaStatus::CONNECTED: set_sta_status(WiFiStaStatus::CONNECTION_LOST); break;
            case WiFiStaStatus::CONNECTING:
            case WiFiStaStatus::WAITING_FOR_IP: set_sta_status(WiFiStaStatus::CONNECT_FAILED); break;
            case WiFiStaStatus::DISCONNECTING:
            default:
                set_sta_status(WiFiStaStatus::DISCONNECTED);
                break;
            }
            break;
        }
        }

        wifi_clear_status_bits(STA_CONNECTED_BIT | STA_HAS_IP_BIT | STA_HAS_IP6_BIT);
        //if (((reason == WIFI_REASON_AUTH_EXPIRE) ||
        //    (reason >= WIFI_REASON_BEACON_TIMEOUT && reason != WIFI_REASON_AUTH_FAIL)))
        //{
        //    if (const auto result = wifi_sta_disconnect(config); result != ESP_OK)
        //        ESP_LOGE(TAG, "wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
        //    if (const auto result = wifi_sta_restart(config); result != ESP_OK)
        //        ESP_LOGE(TAG, "wifi_sta_restart() failed with %s", esp_err_to_name(result));
        //}

        break;
    }
    case WifiEventId::WIFI_STA_GOT_IP:
        ESP_LOGI(TAG, "WIFI_STA_GOT_IP ip=%s netmask=%s gw=%s",
                 wifi_stack::toString(event.got_ip.ip_info.ip).c_str(),
                 wifi_stack::toString(event.got_ip.ip_info.netmask).c_str(),
                 wifi_stack::toString(event.got_ip.ip_info.gw).c_str()
        );
        set_sta_status(WiFiStaStatus::CONNECTED);
        wifi_set_status_bits(STA_HAS_IP_BIT | STA_CONNECTED_BIT);
        break;
    case WifiEventId::WIFI_STA_LOST_IP:
        ESP_LOGW(TAG, "WIFI_STA_LOST_IP");
        set_sta_status(WiFiStaStatus::IDLE_STATUS);
        wifi_clear_status_bits(STA_HAS_IP_BIT);
        break;
    case WifiEventId::WIFI_AP_START:
        wifi_set_status_bits(AP_STARTED_BIT);
        break;
    case WifiEventId::WIFI_AP_STOP:
        wifi_clear_status_bits(AP_STARTED_BIT | AP_HAS_CLIENT_BIT);
        break;
    case WifiEventId::WIFI_AP_STACONNECTED:
        wifi_set_status_bits(AP_HAS_CLIENT_BIT);
        break;
    case WifiEventId::WIFI_AP_STADISCONNECTED:
        wifi_sta_list_t clients;
        if (esp_wifi_ap_get_sta_list(&clients) != ESP_OK || !clients.num)
            wifi_clear_status_bits(AP_HAS_CLIENT_BIT);
        break;
    case WifiEventId::ETH_START:
#ifdef CONFIG_ETH_ENABLED
        wifi_set_status_bits(ETH_STARTED_BIT);
#endif
        break;
    case WifiEventId::ETH_STOP:
#ifdef CONFIG_ETH_ENABLED
        wifi_clear_status_bits(ETH_STARTED_BIT | ETH_CONNECTED_BIT | ETH_HAS_IP_BIT | ETH_HAS_IP6_BIT);
#endif
        break;
    case WifiEventId::ETH_CONNECTED:
#ifdef CONFIG_ETH_ENABLED
        wifi_set_status_bits(ETH_CONNECTED_BIT);
        esp_netif_create_ip6_linklocal(esp_netifs[ESP_IF_ETH]);
#endif
        break;
    case WifiEventId::ETH_DISCONNECTED:
#ifdef CONFIG_ETH_ENABLED
        wifi_clear_status_bits(ETH_CONNECTED_BIT | ETH_HAS_IP_BIT | ETH_HAS_IP6_BIT);
#endif
        break;
    case WifiEventId::ETH_GOT_IP:
#ifdef CONFIG_ETH_ENABLED
        ESP_LOGI(TAG, "ETH_GOT_IP ip=%s netmask=%s gw=%s",
                 wifi_stack::toString(event.got_ip.ip_info.ip).c_str(),
                 wifi_stack::toString(event.got_ip.ip_info.netmask).c_str(),
                 wifi_stack::toString(event.got_ip.ip_info.gw).c_str()
        );
        wifi_set_status_bits(ETH_CONNECTED_BIT | ETH_HAS_IP_BIT);
#endif
        break;
    case WifiEventId::WIFI_STA_GOT_IP6:
        ESP_LOGI(TAG, "WIFI_STA_GOT_IP6 index=%d zone=%d ip=" IPV6STR,
                 event.got_ip6.ip_index, event.got_ip6.ip6_info.ip.zone, IPV62STR(event.got_ip6.ip6_info.ip));
        wifi_set_status_bits(STA_CONNECTED_BIT | STA_HAS_IP6_BIT);
        break;
    case WifiEventId::WIFI_AP_GOT_IP6:
        ESP_LOGI(TAG, "WIFI_AP_GOT_IP6 index=%d zone=%d ip=" IPV6STR,
                 event.got_ip6.ip_index, event.got_ip6.ip6_info.ip.zone, IPV62STR(event.got_ip6.ip6_info.ip));
        wifi_set_status_bits(AP_HAS_IP6_BIT);
        break;
    case WifiEventId::ETH_GOT_IP6:
#ifdef CONFIG_ETH_ENABLED
        ESP_LOGI(TAG, "ETH_GOT_IP6 index=%d zone=%d ip=" IPV6STR,
                 event.got_ip6.ip_index, event.got_ip6.ip6_info.ip.zone, IPV62STR(event.got_ip6.ip6_info.ip));
        wifi_set_status_bits(ETH_CONNECTED_BIT | ETH_HAS_IP6_BIT);
#endif
        break;
#ifdef SMARTCONFIG
    case WifiEventId::SC_GOT_SSID_PSWD:
        wifi_sta_begin(
            (const char *)event.sc_got_ssid_pswd.ssid,
            (const char *)event.sc_got_ssid_pswd.password,
            0,
            ((event.sc_got_ssid_pswd.bssid_set == true)?event.sc_got_ssid_pswd.bssid:NULL)
        );
        break;
    case WifiEventId::SC_SEND_ACK_DONE:
        esp_smartconfig_stop();
        _smartConfigDone = true;
        break;
#endif
    default:;
    }
}

esp_err_t wifi_post_event(std::unique_ptr<const WifiEvent> event)
{
    if (!event)
    {
        ESP_LOGE(TAG, "invalid event");
        return ESP_FAIL;
    }

    const auto ptr = event.get();
    if (const auto result = wifi_event_queue->send(&ptr, portMAX_DELAY); result != pdTRUE)
    {
        ESP_LOGE(TAG, "wifi_event_queue->send() failed with %i", result);
        return ESP_FAIL;
    }

    event.release(); // ownership taken over by wifi_event_queue

    return ESP_OK;
}

void wifi_event_cb(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    auto wifi_event = std::make_unique<WifiEvent>();
    wifi_event->event_id = WifiEventId::MAX;

    /*
     * STA
     * */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_START");
        wifi_event->event_id = WifiEventId::WIFI_STA_START;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_STOP)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_STOP");
        wifi_event->event_id = WifiEventId::WIFI_STA_STOP;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_AUTHMODE_CHANGE)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_AUTHMODE_CHANGE");

        const wifi_event_sta_authmode_change_t &event = *(const wifi_event_sta_authmode_change_t *)event_data;
        ESP_LOGI(TAG, "STA Auth Mode Changed: From: %s, To: %s", wifi_stack::toString(event.old_mode).c_str(), wifi_stack::toString(event.new_mode).c_str());

        wifi_event->event_id = WifiEventId::WIFI_STA_AUTHMODE_CHANGE;
        wifi_event->wifi_sta_authmode_change = event;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_CONNECTED");

        const wifi_event_sta_connected_t &event = *(const wifi_event_sta_connected_t *)event_data;
        ESP_LOGI(TAG, "STA Connected: SSID: %s, BSSID: " MACSTR ", Channel: %u, Auth: %s", event.ssid, MAC2STR(event.bssid), event.channel, wifi_stack::toString(event.authmode).c_str());

        wifi_event->event_id = WifiEventId::WIFI_STA_CONNECTED;
        wifi_event->wifi_sta_connected = event;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_DISCONNECTED");

        const wifi_event_sta_disconnected_t &event = *(const wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGI(TAG, "STA Disconnected: SSID: %s, BSSID: " MACSTR ", Reason: %u", event.ssid, MAC2STR(event.bssid), event.reason);

        wifi_event->event_id = WifiEventId::WIFI_STA_DISCONNECTED;
        wifi_event->wifi_sta_disconnected = event;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "IP_EVENT_STA_GOT_IP");

        const ip_event_got_ip_t &event = *(const ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "STA Got %sIP:" IPSTR, event.ip_changed?"New ":"Same ", IP2STR(&event.ip_info.ip));

        wifi_event->event_id = WifiEventId::WIFI_STA_GOT_IP;
        wifi_event->got_ip = event;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "IP_EVENT_STA_LOST_IP");
        wifi_event->event_id = WifiEventId::WIFI_STA_LOST_IP;
    }

    /*
     * SCAN
     * */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_SCAN_DONE");

        const wifi_event_sta_scan_done_t &event = *(const wifi_event_sta_scan_done_t *)event_data;
        ESP_LOGI(TAG, "SCAN Done: ID: %i, Status: %lu, Results: %i", event.scan_id, event.status, event.number);

        wifi_event->event_id = WifiEventId::WIFI_SCAN_DONE;
        wifi_event->wifi_scan_done = event;
    }

    /*
     * AP
     * */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_AP_START");
        wifi_event->event_id = WifiEventId::WIFI_AP_START;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_AP_STOP");
        wifi_event->event_id = WifiEventId::WIFI_AP_STOP;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_PROBEREQRECVED)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_AP_PROBEREQRECVED");

        const wifi_event_ap_probe_req_rx_t &event = *(const wifi_event_ap_probe_req_rx_t *)event_data;
        ESP_LOGI(TAG, "AP Probe Request: RSSI: %d, MAC: " MACSTR, event.rssi, MAC2STR(event.mac));

        wifi_event->event_id = WifiEventId::WIFI_AP_PROBEREQRECVED;
        wifi_event->wifi_ap_probereqrecved = event;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_AP_STACONNECTED");

        const wifi_event_ap_staconnected_t &event = *(const wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "AP Station Connected: MAC: " MACSTR ", AID: %d", MAC2STR(event.mac), event.aid);

        wifi_event->event_id = WifiEventId::WIFI_AP_STACONNECTED;
        wifi_event->wifi_ap_staconnected = event;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_AP_STADISCONNECTED");

        const wifi_event_ap_stadisconnected_t &event = *(const wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "AP Station Disconnected: MAC: " MACSTR ", AID: %d", MAC2STR(event.mac), event.aid);

        wifi_event->event_id = WifiEventId::WIFI_AP_STADISCONNECTED;
        wifi_event->wifi_ap_stadisconnected = event;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "IP_EVENT_AP_STAIPASSIGNED");

        const ip_event_ap_staipassigned_t &event = *(const ip_event_ap_staipassigned_t *)event_data;
        ESP_LOGI(TAG, "AP Station IP Assigned:" IPSTR, IP2STR(&event.ip));

        wifi_event->event_id = WifiEventId::WIFI_AP_STAIPASSIGNED;
        wifi_event->wifi_ap_staipassigned = event;
    }

#ifdef CONFIG_ETH_ENABLED
    /*
     * ETH
     * */
    else if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_CONNECTED)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "ETHERNET_EVENT_CONNECTED");

        esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

        wifi_event->event_id = WifiEventId::ETH_CONNECTED;
        wifi_event->eth_connected = eth_handle;
    }
    else if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_DISCONNECTED)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "ETHERNET_EVENT_DISCONNECTED");
        wifi_event->event_id = WifiEventId::ETH_DISCONNECTED;
    }
    else if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_START)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "ETHERNET_EVENT_START");
        wifi_event->event_id = WifiEventId::ETH_START;
    }
    else if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_STOP)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "ETHERNET_EVENT_STOP");
        wifi_event->event_id = WifiEventId::ETH_STOP;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "IP_EVENT_ETH_GOT_IP");

        const ip_event_got_ip_t &event = *(const ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Ethernet got %sip:" IPSTR, event.ip_changed?"new":"", IP2STR(&event.ip_info.ip));

        wifi_event->event_id = WifiEventId::ETH_GOT_IP;
        wifi_event->got_ip = event;
    }
#endif

    /*
     * IPv6
     * */
    else if (event_base == IP_EVENT && event_id == IP_EVENT_GOT_IP6)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "IP_EVENT_GOT_IP6");

        ip_event_got_ip6_t * event = (ip_event_got_ip6_t*)event_data;

        esp_interface_t iface = ESP_IF_MAX;
        for (int i = 0; i < ESP_IF_MAX; i++)
            if (esp_netifs[i] &&
                esp_netifs[i] == event->esp_netif)
            {
                iface = (esp_interface_t)i;
                break;
            }

        ESP_LOGV(TAG, "IF[%d] Got IPv6: IP Index: %d, Zone: %d, " IPV6STR, iface, event->ip_index, event->ip6_info.ip.zone, IPV62STR(event->ip6_info.ip));
        switch (iface)
        {
        case ESP_IF_WIFI_STA:
            wifi_event->event_id = WifiEventId::WIFI_STA_GOT_IP6;
            break;
        case ESP_IF_WIFI_AP:
            wifi_event->event_id = WifiEventId::WIFI_AP_GOT_IP6;
            break;
        case ESP_IF_ETH:
#ifdef CONFIG_ETH_ENABLED
            wifi_event->event_id = WifiEventId::ETH_GOT_IP6;
#endif
            break;
        default:;
        }
        std::memcpy(&wifi_event->got_ip6, event_data, sizeof(ip_event_got_ip6_t)); // TODO replace with operator=
    }

    /*
     * WPS
     * */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_SUCCESS)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_WPS_ER_SUCCESS");
        wifi_event->event_id = WifiEventId::WPS_ER_SUCCESS;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_FAILED)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_WPS_ER_FAILED");

        wifi_event_sta_wps_fail_reason_t * event = (wifi_event_sta_wps_fail_reason_t*)event_data;
        CPP_UNUSED(event)

        wifi_event->event_id = WifiEventId::WPS_ER_FAILED;
        std::memcpy(&wifi_event->wps_fail_reason, event_data, sizeof(wifi_event_sta_wps_fail_reason_t)); // TODO replace with operator=
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_TIMEOUT)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_WPS_ER_TIMEOUT");
        wifi_event->event_id = WifiEventId::WPS_ER_TIMEOUT;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_PIN)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_WPS_ER_PIN");

        wifi_event_sta_wps_er_pin_t * event = (wifi_event_sta_wps_er_pin_t*)event_data;
        CPP_UNUSED(event)

        wifi_event->event_id = WifiEventId::WPS_ER_PIN;
        std::memcpy(&wifi_event->wps_er_pin, event_data, sizeof(wifi_event_sta_wps_er_pin_t)); // TODO replace with operator=
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP)
    {
        ESP_LOGI(TAG, "event_base=%s event_id=%s", event_base, "WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP");
        wifi_event->event_id = WifiEventId::WPS_ER_PBC_OVERLAP;
    }

#ifdef SMARTCONFIG
    /*
     * SMART CONFIG
     * */
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE)
    {
        ESP_LOGV(TAG, "SC Scan Done");
        wifi_event->event_id = WifiEventId::SC_SCAN_DONE;
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL)
    {
        ESP_LOGV(TAG, "SC Found Channel");
        wifi_event->event_id = WifiEventId::SC_FOUND_CHANNEL;
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
    {
        smartconfig_event_got_ssid_pswd_t *event = (smartconfig_event_got_ssid_pswd_t *)event_data;
        ESP_LOGV(TAG, "SC: SSID: %s, Password: %s", (const char *)event->ssid, (const char *)event->password);
        wifi_event->event_id = WifiEventId::SC_GOT_SSID_PSWD;
        std::memcpy(&wifi_event->sc_got_ssid_pswd, event_data, sizeof(smartconfig_event_got_ssid_pswd_t)); // TODO replace with operator=

    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE)
    {
        ESP_LOGV(TAG, "SC Send Ack Done");
        wifi_event->event_id = WifiEventId::SC_SEND_ACK_DONE;
    }
#endif

#ifdef PROVISIONING
    /*
     * Provisioning
     * */
    else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_INIT)
    {
        ESP_LOGV(TAG, "Provisioning Initialized!");
        wifi_event->event_id = WifiEventId::PROV_INIT;
    }
    else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_DEINIT)
    {
        ESP_LOGV(TAG, "Provisioning Uninitialized!");
        wifi_event->event_id = WifiEventId::PROV_DEINIT;
    }
    else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_START)
    {
        ESP_LOGV(TAG, "Provisioning Start!");
        wifi_event->event_id = WifiEventId::PROV_START;
    }
    else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_END)
    {
        ESP_LOGV(TAG, "Provisioning End!");
        wifi_prov_mgr_deinit();
        wifi_event->event_id = WifiEventId::PROV_END;
    }
    else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_CRED_RECV)
    {
        wifi_sta_config_t *event = (wifi_sta_config_t *)event_data;
        ESP_LOGV(TAG, "Provisioned Credentials: SSID: %s, Password: %s", (const char *) event->ssid, (const char *) event->password);
        wifi_event->event_id = WifiEventId::PROV_CRED_RECV;
        std::memcpy(&wifi_event->prov_cred_recv, event_data, sizeof(wifi_sta_config_t)); // TODO replace with operator=
    }
    else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_CRED_FAIL)
    {
        wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
        ESP_LOGE(TAG, "Provisioning Failed: Reason : %s", (*reason == WIFI_PROV_STA_AUTH_ERROR)?"Authentication Failed":"AP Not Found");
        wifi_event->event_id = WifiEventId::PROV_CRED_FAIL;
        std::memcpy(&wifi_event->prov_fail_reason, event_data, sizeof(wifi_prov_sta_fail_reason_t)); // TODO replace with operator=
    }
    else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_CRED_SUCCESS)
    {
        ESP_LOGV(TAG, "Provisioning Success!");
        wifi_event->event_id = WifiEventId::PROV_CRED_SUCCESS;
    }
#endif
    else
        ESP_LOGW(TAG, "event_base=%s event_id=%li", event_base, event_id);

    if (wifi_event->event_id < WifiEventId::MAX)
        wifi_post_event(std::move(wifi_event));
}

esp_err_t wifi_start_network_event_task(const config &config)
{
    if (!wifi_event_group.constructed())
    {
        wifi_event_group.construct();
        if (!wifi_event_group->handle)
        {
            wifi_event_group.destruct();
            ESP_LOGE(TAG, "Network Event Group Create Failed!");
            return ESP_FAIL;
        }

        wifi_event_group->setBits(WIFI_DNS_IDLE_BIT);
    }

    if (!wifi_event_queue.constructed())
    {
        wifi_event_queue.construct(UBaseType_t{32}, sizeof(WifiEvent*));
        if (!wifi_event_queue->handle)
        {
            wifi_event_queue.destruct();
            ESP_LOGE(TAG, "Network Event Queue Create Failed!");
            return ESP_FAIL;
        }
    }

    if (!defaultEventLoopCreated)
    {
        if (const auto result = esp_event_loop_create_default(); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_event_loop_create_default() failed with %s", esp_err_to_name(result));
            return result;
        }
        defaultEventLoopCreated = true;
    }

    if (!wifiEventRegistered)
    {
        if (const auto result = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_cb, NULL, NULL); result != ESP_OK)
        {
            ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "WIFI_EVENT", esp_err_to_name(result));
            return result;
        }
        wifiEventRegistered = true;
    }

    if (!ipEventRegistered)
    {
        if (const auto result = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_cb, NULL, NULL); result != ESP_OK)
        {
            ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "IP_EVENT", esp_err_to_name(result));
            return result;
        }
        ipEventRegistered = true;
    }

#ifdef SMARTCONFIG
    if (!scEventRegistered)
    {
        if (const auto result = esp_event_handler_instance_register(SC_EVENT, ESP_EVENT_ANY_ID, &wifi_event_cb, NULL, NULL); result != ESP_OK)
        {
            ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "SC_EVENT", esp_err_to_name(result));
            return result;
        }
        scEventRegistered = true;
    }
#endif

#ifdef CONFIG_ETH_ENABLED
    if (!ethEventRegistered)
    {
        if (const auto result = esp_event_handler_instance_register(ETH_EVENT, ESP_EVENT_ANY_ID, &wifi_event_cb, NULL, NULL); result != ESP_OK)
        {
            ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "ETH_EVENT", esp_err_to_name(result));
            return result;
        }
        ethEventRegistered = true;
    }
#endif

#ifdef PROVISIONING
    if (!wifiProvEventRegistered)
    {
        if (const auto result = esp_event_handler_instance_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &wifi_event_cb, NULL, NULL); result != ESP_OK)
        {
            ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "WIFI_PROV_EVENT", esp_err_to_name(result));
            return result;
        }
        wifiProvEventRegistered = true;
    }
#endif

    return ESP_OK;
}

esp_err_t wifi_low_level_init(const config &config)
{
    if (_lowLevelInitDone)
    {
        ESP_LOGW(TAG, "already called");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "called");

    if (!esp_netifs[ESP_IF_WIFI_AP])
    {
        esp_netifs[ESP_IF_WIFI_AP] = esp_netif_create_default_wifi_ap();
        if (!esp_netifs[ESP_IF_WIFI_AP])
        {
            ESP_LOGE(TAG, "esp_netif_create_default_wifi_ap() returned null");
            return ESP_FAIL;
        }
    }

    if (!esp_netifs[ESP_IF_WIFI_STA])
    {
        esp_netifs[ESP_IF_WIFI_STA] = esp_netif_create_default_wifi_sta();
        if (!esp_netifs[ESP_IF_WIFI_STA])
        {
            ESP_LOGE(TAG, "esp_netif_create_default_wifi_sta() returned null");
            return ESP_FAIL;
        }
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (const auto result = esp_wifi_init(&cfg); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_init() failed with %s", esp_err_to_name(result));
        return result;
    }

    if (const auto result = esp_wifi_set_storage(WIFI_STORAGE_RAM); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_storage() failed with %s", esp_err_to_name(result));
        return result;
    }

    _lowLevelInitDone = true;
    return ESP_OK;
}

esp_err_t wifi_start()
{
    if (_esp_wifi_started)
    {
        //ESP_LOGW(TAG, "already called");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "called");

    if (const auto result = esp_wifi_start(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_start() failed with %s", esp_err_to_name(result));
        return result;
    }

    _esp_wifi_started = true;

    return ESP_OK;
}

esp_err_t wifi_low_level_deinit()
{
    if (!_lowLevelInitDone)
    {
        ESP_LOGW(TAG, "already called");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "called");

    if (const auto result = esp_wifi_deinit(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_deinit() failed with %s", esp_err_to_name(result));
        return result;
    }

    _lowLevelInitDone = false;
    return ESP_OK;
}

esp_err_t wifi_stop()
{
    if (!_esp_wifi_started)
    {
        ESP_LOGW(TAG, "already called");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "called");

    if (const auto result = esp_wifi_stop(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_stop() failed with %s", esp_err_to_name(result));
        return result;
    }

    _esp_wifi_started = false;

    if (const auto result = wifi_low_level_deinit(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "wifi_low_level_deinit() failed with %s", esp_err_to_name(result));
        return result;
    }

    return ESP_OK;
}

tl::expected<void, std::string> applyBaseMac(const mac_t &mac)
{
    if (const auto result = set_base_mac_addr(mac); result)
        return {};
    else
    {
        const auto msg = fmt::format("set_base_mac_addr() {} failed: {}", toString(mac), result.error());
        ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
        return tl::make_unexpected(msg);
    }
}

tl::expected<mac_t, std::string> expectedBaseMac(const config &config)
{
    if (config.base_mac_override)
    {
        if (*config.base_mac_override != mac_t{})
            return *config.base_mac_override;
        else
            ESP_LOGW(TAG, "invalid base_mac_override %s", toString(*config.base_mac_override).c_str());
    }

    if (const auto mac = get_custom_mac_addr())
        return *mac;
    //else
        //ESP_LOGW(TAG, "get_custom_mac_addr() failed %.*s", mac.error().size(), mac.error().data());

    if (const auto mac = get_default_mac_addr())
        return *mac;
    else
    {
        const auto msg = fmt::format("no base mac fuse or override set and get_default_mac_addr() failed: {}", mac.error());
        ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
        return tl::make_unexpected(msg);
    }
}

esp_err_t wifi_sync_mode(const config &config)
{
    const wifi_mode_t newMode = wifi_mode_t(
        (config.sta ? WIFI_MODE_STA : WIFI_MODE_NULL) |
        (config.ap ? WIFI_MODE_AP : WIFI_MODE_NULL));

    const wifi_mode_t oldMode = get_wifi_mode();

    if (oldMode == newMode)
        return ESP_OK;

    if (!oldMode && newMode)
    {
        if (const auto result = wifi_low_level_init(config); result != ESP_OK)
        {
            ESP_LOGE(TAG, "wifi_low_level_init() failed with %s", esp_err_to_name(result));
            return result;
        }
    }
    else if (oldMode && !newMode)
    {
        if (const auto result = wifi_stop(); result != ESP_OK)
        {
            ESP_LOGE(TAG, "wifi_stop() failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    if (config.sta)
    {
        if (esp_netifs[ESP_IF_WIFI_STA])
        {
            if (const auto result = esp_netif_set_hostname(esp_netifs[ESP_IF_WIFI_STA], config.sta->hostname.c_str()); result != ESP_OK)
                ESP_LOGE(TAG, "esp_netif_set_hostname() STA \"%s\" failed with %s", config.sta->hostname.c_str(), esp_err_to_name(result));
        }
        else
            ESP_LOGE(TAG, "netif for STA is invalid");
    }

    if (config.ap)
    {
        if (esp_netifs[ESP_IF_WIFI_AP])
        {
            if (const auto result = esp_netif_set_hostname(esp_netifs[ESP_IF_WIFI_AP], config.ap->hostname.c_str()); result != ESP_OK)
                ESP_LOGE(TAG, "esp_netif_set_hostname() AP \"%s\" failed with %s", config.ap->hostname.c_str(), esp_err_to_name(result));
        }
        else
            ESP_LOGE(TAG, "netif for AP is invalid");
    }

    if (const auto result = esp_wifi_set_mode(newMode); result != ESP_OK)
        ESP_LOGE(TAG, "esp_wifi_set_mode() failed with %s", esp_err_to_name(result));

    if (newMode)
    {
        if (config.sta && config.sta->long_range)
            if (const auto result = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR); result != ESP_OK)
                ESP_LOGE(TAG, "esp_wifi_set_protocol() for STA long range failed with %s", esp_err_to_name(result));

        if (config.ap && config.ap->long_range)
            if (const auto result = esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR); result != ESP_OK)
                ESP_LOGE(TAG, "esp_wifi_set_protocol() for AP long range failed with %s", esp_err_to_name(result));

        if (config.country)
            if (const auto result = esp_wifi_set_country(&*config.country); result != ESP_OK)
                ESP_LOGE(TAG, "esp_wifi_set_country_code() failed with %s", esp_err_to_name(result));

        if (const auto result = wifi_start(); result != ESP_OK)
        {
            ESP_LOGE(TAG, "wifi_start() failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    return ESP_OK;
}

esp_err_t wifi_set_ap_ip(const config &config, const static_ip_config &ip)
{
    using wifi_stack::toString;

    ESP_LOGI(TAG, "ip=%s subnet=%s gateway=%s", toString(ip.ip).c_str(), toString(ip.subnet).c_str(), toString(ip.gateway).c_str());

    esp_netif_t * const esp_netif = esp_netifs[ESP_IF_WIFI_AP];
    if (!esp_netif)
    {
        ESP_LOGE(TAG, "netif for AP is invalid");
        return ESP_FAIL;
    }

    {
        esp_netif_dhcp_status_t status;
        if (const auto result = esp_netif_dhcps_get_status(esp_netif, &status); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_dhcps_get_status() failed with %s", esp_err_to_name(result));
            return result;
        }
        ESP_LOGI(TAG, "esp_netif_dhcps_get_status() resulted in %s", toString(status).c_str());
    }

    for (int i = 0; ; i++)
    {
        if (const auto result = esp_netif_dhcps_stop(esp_netif); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_dhcps_stop() failed with %s", esp_err_to_name(result));
            if (result != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)
                return result;
        }

        esp_netif_ip_info_t info;
        info.ip.addr = ip.ip.value();
        info.netmask.addr = ip.subnet.value();
        info.gw.addr = ip.gateway.value();

        if (const auto result = esp_netif_set_ip_info(esp_netif, &info); result == ESP_OK)
            break;
        else
        {
            ESP_LOGE(TAG, "esp_netif_set_ip_info() for AP failed with %s", esp_err_to_name(result));
            if (result == ESP_ERR_ESP_NETIF_DHCP_NOT_STOPPED)
            {
                if (i < 20)
                {
                    espcpputils::delay(50ms);
                    continue;
                }
                else
                    ESP_LOGE(TAG, "esp_netif_set_ip_info() giving up after 20 tries");
            }
            return result;
        }
    }

    {
        dhcps_lease_t lease;
        lease.enable = true;
        lease.start_ip.addr = ip.ip.value() + (1 << 24);
        lease.end_ip.addr = ip.ip.value() + (11 << 24);

        if (const auto result = esp_netif_dhcps_option(esp_netifs[ESP_IF_WIFI_AP],
                                                       ESP_NETIF_OP_SET,
                                                       ESP_NETIF_REQUESTED_IP_ADDRESS,
                                                       &lease, sizeof(dhcps_lease_t)); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_dhcps_option() REQUESTED_IP_ADDRESS failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    {
        dhcps_offer_t dhcps_flag_false = 0;

        if (const auto result = esp_netif_dhcps_option(esp_netifs[ESP_IF_WIFI_AP],
                                                       ESP_NETIF_OP_SET,
                                                       ESP_NETIF_ROUTER_SOLICITATION_ADDRESS,
                                                       &dhcps_flag_false, sizeof(dhcps_flag_false)); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_dhcps_option() ROUTER_SOLICITATION_ADDRESS failed with %s", esp_err_to_name(result));
            return result;
        }

        if (const auto result = esp_netif_dhcps_option(esp_netifs[ESP_IF_WIFI_AP],
                                                       ESP_NETIF_OP_SET,
                                                       ESP_NETIF_DOMAIN_NAME_SERVER,
                                                       &dhcps_flag_false, sizeof(dhcps_flag_false)); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_dhcps_option() DOMAIN_NAME_SERVER failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    if (const auto result = esp_netif_dhcps_start(esp_netif); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_netif_dhcps_start() failed with %s", esp_err_to_name(result));
        return result;
    }

    return ESP_OK;
}

wifi_config_t make_sta_config(std::string_view ssid, std::string_view password, int8_t min_rssi,
                              std::optional<mac_t> bssid, uint8_t channel)
{
    wifi_config_t wifi_config;

    wifi_config.sta.channel = channel;
    wifi_config.sta.listen_interval = 0;
    wifi_config.sta.scan_method = WIFI_FAST_SCAN; //WIFI_ALL_CHANNEL_SCAN or WIFI_FAST_SCAN
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL; //WIFI_CONNECT_AP_BY_SIGNAL or WIFI_CONNECT_AP_BY_SECURITY
    wifi_config.sta.threshold.rssi = min_rssi;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    wifi_config.sta.bssid_set = 0;
    std::fill(std::begin(wifi_config.sta.bssid), std::end(wifi_config.sta.bssid), 0);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.ssid[0] = 0;
    wifi_config.sta.password[0] = 0;

    if (!ssid.empty())
    {
        copyStrToBuf(wifi_config.sta.ssid, ssid);

        if (!password.empty())
        {
            wifi_config.sta.threshold.authmode = WIFI_AUTH_WEP;
            copyStrToBuf(wifi_config.sta.password, password);
        }

        if (bssid)
        {
            wifi_config.sta.bssid_set = 1;
            bssid->copyTo(wifi_config.sta.bssid);
        }
    }

    return wifi_config;
}

esp_err_t wifi_sta_begin(const config &config, const sta_config &sta_config, std::string_view ssid,
                         const wifi_entry &wifi_entry, int32_t channel, std::optional<mac_t> bssid, bool connect)
{
    if (!(get_wifi_mode() & WIFI_MODE_STA))
    {
        ESP_LOGE(TAG, "STA mode missing");
        return ESP_FAIL;
    }

    if (wifi_entry.ssid.empty())
    {
        ESP_LOGE(TAG, "SSID missing!");
        return ESP_FAIL;
    }

    if (wifi_entry.ssid.size() > 32)
    {
        ESP_LOGE(TAG, "SSID too long! (size=%zd)", wifi_entry.ssid.size());
        return ESP_FAIL;
    }

    if (!wifi_entry.key.empty())
    {
        if (wifi_entry.key.size() < 8)
        {
            ESP_LOGE(TAG, "key too short! (size=%zd)", wifi_entry.key.size());
            return ESP_FAIL;
        }

        if (wifi_entry.key.size() > 64)
        {
            ESP_LOGE(TAG, "key too long! (size=%zd)", wifi_entry.key.size());
            return ESP_FAIL;
        }
    }

    wifi_config_t conf = make_sta_config(ssid, wifi_entry.key, sta_config.min_rssi, bssid, channel);

    wifi_config_t current_conf;
    if (const auto result = esp_wifi_get_config(WIFI_IF_STA, &current_conf); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_get_config() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    if (!wifi_sta_config_equal(current_conf.sta, conf.sta))
    {
        if (const auto result = esp_wifi_disconnect(); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_disconnect() failed with %s", esp_err_to_name(result));
            return result;
        }

        if (const auto result = esp_wifi_set_config(WIFI_IF_STA, &conf); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_set_config() for STA failed with %s", esp_err_to_name(result));
            return result;
        }
    }
    else if (get_sta_status() == WiFiStaStatus::CONNECTED)
    {
        ESP_LOGW(TAG, "when already connected?!");
        return ESP_OK;
    }
    else
    {
        if (const auto result = esp_wifi_set_config(WIFI_IF_STA, &conf); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_set_config() for STA failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    if (const auto result = wifi_set_esp_interface_ip(ESP_IF_WIFI_STA, wifi_entry.static_ip); result != ESP_OK)
    {
        ESP_LOGE(TAG, "wifi_set_esp_interface_ip() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    last_sta_static_ip = wifi_entry.static_ip;

    if (const auto result = wifi_set_esp_interface_dns(ESP_IF_WIFI_STA, wifi_entry.static_dns); result != ESP_OK)
    {
        ESP_LOGE(TAG, "wifi_set_esp_interface_dns() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    last_sta_static_dns = wifi_entry.static_dns;

    if (connect)
    {
        ESP_LOGI(TAG, "clearing connect fail flag");
        _wifiConnectFailFlag = std::nullopt;

        if (const auto result = esp_wifi_connect(); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_connect() failed with %s", esp_err_to_name(result));
            return result;
        }

        set_sta_status(WiFiStaStatus::CONNECTING);
    }

    return ESP_OK;
}

esp_err_t wifi_sta_restart(const config &config)
{
    if (!(get_wifi_mode() & WIFI_MODE_STA))
    {
        ESP_LOGE(TAG, "STA mode missing");
        return ESP_FAIL;
    }

    wifi_config_t current_conf;
    if (const auto result = esp_wifi_get_config(WIFI_IF_STA, &current_conf); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_get_config() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    if (const auto result = esp_wifi_set_config(WIFI_IF_STA, &current_conf); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_config() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    // TODO

    if (const auto result = wifi_set_esp_interface_ip(ESP_IF_WIFI_STA, /*config.sta.static_ip*/last_sta_static_ip); result != ESP_OK)
    {
        ESP_LOGE(TAG, "wifi_set_esp_interface_ip() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    //last_sta_static_ip = config.sta.static_ip;

    if (const auto result = wifi_set_esp_interface_dns(ESP_IF_WIFI_STA, /*config.sta.static_dns*/last_sta_static_dns); result != ESP_OK)
    {
        ESP_LOGE(TAG, "wifi_set_esp_interface_dns() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    //last_sta_static_dns = config.sta.static_dns;

    if (get_sta_status() != WiFiStaStatus::CONNECTED)
    {
        ESP_LOGI(TAG, "clearing connect fail flag");
        _wifiConnectFailFlag = std::nullopt;

        if (const auto result = esp_wifi_connect(); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_connect() failed with %s", esp_err_to_name(result));
            return result;
        }

        set_sta_status(WiFiStaStatus::CONNECTING);
    }
    else
        ESP_LOGW(TAG, "when already connected?!");

    return ESP_OK;
}

esp_err_t wifi_sta_disconnect(const config &config, bool eraseap)
{
    ESP_LOGI(TAG, "eraseap=%s", eraseap ? "true" : "false");

    if (!(get_wifi_mode() & WIFI_MODE_STA))
    {
        ESP_LOGE(TAG, "mode STA not set");
        return ESP_FAIL;
    }

    wifi_config_t conf = make_sta_config({}, {}, 0, {}, 0);

    if (eraseap)
    {
        if (const auto result = esp_wifi_set_config(WIFI_IF_STA, &conf); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_set_config() failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    set_sta_status(WiFiStaStatus::DISCONNECTING);

    if (const auto result = esp_wifi_disconnect(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_disconnect() failed with %s", esp_err_to_name(result));
        return result;
    }

    return ESP_OK;
}

std::string calculateWifisChecksum(const sta_config &sta_config)
{
    std::string result;
    for (const auto &wifi : sta_config.wifis)
    {
        result += wifi.ssid;
        result += ",";
        result += wifi.key;
        result += "|";
    }
    return result;
}

void setWifiState(WiFiState newWifiState)
{
    if (newWifiState != _wifiState)
    {
        ESP_LOGI(TAG, "state machine state changed from %s to %s", toString(_wifiState).c_str(), toString(newWifiState).c_str());
        _wifiState = newWifiState;
    }
}

bool buildConnectPlan(const config &config, const sta_config &sta_config)
{
    const auto &scanResult = get_scan_result();
    if (!scanResult)
    {
        ESP_LOGE(TAG, "no scan result available!");
        return false;
    }

    return buildConnectPlan(config, sta_config, *scanResult);
}

bool buildConnectPlan(const config &config, const sta_config &sta_config, const scan_result &scanResult)
{
    _connectPlanWifisChecksum = calculateWifisChecksum(sta_config);

    _pastConnectPlan.clear();
    _currentConnectPlanEntry = mac_t{};
    _connectPlan.clear();

    for (const auto &entry : scanResult.entries)
    {
        std::string_view scanSSID{(const char *)entry.ssid};

        const auto iter = std::find_if(std::begin(sta_config.wifis), std::end(sta_config.wifis),
                                       [&scanSSID](const auto &entry){ return cpputils::stringEqualsIgnoreCase(entry.ssid, scanSSID); });
        if (iter != std::end(sta_config.wifis))
        {
            _connectPlan.push_back(mac_t{entry.bssid});
        }
    }

    return nextConnectPlanItem(config, sta_config, scanResult);
}

bool nextConnectPlanItem(const config &config, const sta_config &sta_config)
{
    const auto &scanResult = get_scan_result();
    if (!scanResult)
    {
        ESP_LOGE(TAG, "no scan result available!");
        return false;
    }

    return nextConnectPlanItem(config, sta_config, *scanResult);
}

bool nextConnectPlanItem(const config &config, const sta_config &sta_config, const scan_result &scanResult)
{
    if (_connectPlan.empty())
    {
        ESP_LOGW(TAG, "no (more) configured ssid found");

        {
            std::string foundSsids;
            for (const auto &entry : scanResult.entries)
            {
                std::string_view scanSSID{(const char *)entry.ssid};
                if (!foundSsids.empty())
                    foundSsids += ", ";
                foundSsids += scanSSID;
            }
            ESP_LOGW(TAG, "found ssids: %s", foundSsids.c_str());
        }

        {
            std::string configuredSsids;
            for (const auto &config : sta_config.wifis)
            {
                if (!configuredSsids.empty())
                    configuredSsids += ", ";
                configuredSsids += config.ssid;
            }
            ESP_LOGW(TAG, "configured ssids: %s", configuredSsids.c_str());
        }

        setWifiState(WiFiState::None);

        return false;
    }

    using wifi_stack::toString;

    _currentConnectPlanEntry = _connectPlan.front();
    _connectPlan.erase(std::begin(_connectPlan));

    const auto scanResultIter = std::find_if(std::begin(scanResult.entries), std::end(scanResult.entries), [&](const wifi_ap_record_t &entry){
        return mac_t{entry.bssid} == _currentConnectPlanEntry;
    });

    if (scanResultIter == std::end(scanResult.entries))
    {
        ESP_LOGW(TAG, "could not find bssid from connect plan in scan result %s", toString(_currentConnectPlanEntry).c_str());
        return nextConnectPlanItem(config, sta_config, scanResult);
    }

    const std::string_view ssid{reinterpret_cast<const char *>(scanResultIter->ssid)};

    const auto configIter = std::find_if(std::begin(sta_config.wifis), std::end(sta_config.wifis),
                                         [ssid](const wifi_entry &entry){ return cpputils::stringEqualsIgnoreCase(entry.ssid, ssid); });

    if (configIter == std::end(sta_config.wifis))
    {
        ESP_LOGW(TAG, "could not find config for ssid %.*s", ssid.size(), ssid.data());
        return nextConnectPlanItem(config, sta_config, scanResult);
    }

    ESP_LOGI(TAG, "connecting to %.*s (auth=%s, key=%.*s, channel=%i, rssi=%i, bssid=%s)",
             ssid.size(), ssid.data(),
             toString(scanResultIter->authmode).c_str(),
             configIter->key.size(), configIter->key.data(),
             scanResultIter->primary,
             scanResultIter->rssi,
             toString(mac_t{scanResultIter->bssid}).c_str()
    );
    _lastConnect = espchrono::millis_clock::now();

    ESP_LOGI(TAG, "resetting wifi connect fail counter");
    _wifiConnectFailCounter = 0;
    if (const auto result = wifi_sta_begin(config, sta_config, ssid, *configIter, scanResultIter->primary, mac_t{scanResultIter->bssid}); result != ESP_OK)
        ESP_LOGE(TAG, "wifi_sta_begin() failed with %s", esp_err_to_name(result));
    setWifiState(WiFiState::Connecting);

    return true;
}

void handleWifiEvents(const config &config, TickType_t xTicksToWait)
{
    const WifiEvent *data{};
    if (const auto result = wifi_event_queue->receive(&data, xTicksToWait); result != pdTRUE)
    {
        if (result != pdFALSE)
            ESP_LOGE(TAG, "wifi_event_queue->receive() failed with %i", result);
        return;
    }

    if (!data)
    {
        ESP_LOGE(TAG, "received nullptr event");
        return;
    }

    std::unique_ptr<const WifiEvent> event{data};
    wifi_event_callback(config, *event);
}

#ifdef CONFIG_ETH_ENABLED
tl::expected<void, std::string> eth_begin(const config &config, const eth_config &eth)
{
    esp_netif_config_t cfg ESP_NETIF_DEFAULT_ETH();
    esp_netif_inherent_config_t newBase = *cfg.base;
    newBase.route_prio = 150;
    cfg.base = &newBase;

    esp_netifs[ESP_IF_ETH] = esp_netif_new(&cfg);
    if (!esp_netifs[ESP_IF_ETH])
    {
        auto msg = std::string{"esp_netif_new() failed"};
        ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
        return tl::make_unexpected(std::move(msg));
    }

    esp_eth_mac_t *eth_mac{};

    eth_esp32_emac_config_t emac_config ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_mdc_gpio_num = 23;
    emac_config.smi_mdio_gpio_num = 18;
    emac_config.interface = EMAC_DATA_INTERFACE_RMII;

    eth_mac_config_t mac_config ETH_MAC_DEFAULT_CONFIG();
    mac_config.sw_reset_timeout_ms = 1000;

    eth_mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (!eth_mac)
    {
        auto msg = std::string{"esp_eth_mac_new_esp32() failed"};
        ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
        return tl::make_unexpected(std::move(msg));
    }

    eth_phy_config_t phy_config ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;
    phy_config.reset_gpio_num = 16;

    esp_eth_phy_t *eth_phy{};

    if constexpr (true)
    {
        eth_phy = esp_eth_phy_new_lan87xx(&phy_config);
        if (!eth_phy)
        {
            auto msg = std::string{"esp_eth_phy_new_lan8720() failed"};
            ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
            return tl::make_unexpected(std::move(msg));
        }
    }
    else
    {
        eth_phy = esp_eth_phy_new_ksz80xx(&phy_config);
        if (!eth_phy)
        {
            auto msg = std::string{"esp_eth_phy_new_ksz8041() failed"};
            ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
            return tl::make_unexpected(std::move(msg));
        }
    }

    eth_handle = {};

    esp_eth_config_t eth_config ETH_DEFAULT_CONFIG(eth_mac, eth_phy);

    if (const auto result = esp_eth_driver_install(&eth_config, &eth_handle); result != ESP_OK)
    {
        auto msg = fmt::format("esp_eth_driver_install() failed with {}", esp_err_to_name(result));
        ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
        return tl::make_unexpected(std::move(msg));
    }

    if (!eth_handle)
    {
        auto msg = std::string{"esp_eth_driver_install() invalid eth_handle"};
        ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
        return tl::make_unexpected(std::move(msg));
    }

    /* attach Ethernet driver to TCP/IP stack */
    auto ptr = esp_eth_new_netif_glue(eth_handle);
    if (!ptr)
    {
        auto msg = std::string{"esp_eth_new_netif_glue() failed"};
        ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
        return tl::make_unexpected(std::move(msg));
    }

    if (const auto result = esp_netif_attach(esp_netifs[ESP_IF_ETH], ptr); result != ESP_OK)
    {
        auto msg = fmt::format("esp_netif_attach() failed with {}", esp_err_to_name(result));
        ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
        return tl::make_unexpected(std::move(msg));
    }

    eth_initialized = true;

    if (config.eth)
    {
        if (const auto result = esp_eth_start(eth_handle); result != ESP_OK)
        {
            auto msg = fmt::format("esp_eth_start() failed with {}", esp_err_to_name(result));
            ESP_LOGE(TAG, "%.*s", msg.size(), msg.data());
            return tl::make_unexpected(std::move(msg));
        }

        eth_started = true;

        if (const auto result = wifi_set_esp_interface_ip(ESP_IF_ETH, config.eth->static_ip); result != ESP_OK)
        {
            ESP_LOGE(TAG, "wifi_set_esp_interface_ip() for ETH failed with %s", esp_err_to_name(result));
            //return result;
        }
        else
        {
            last_eth_static_ip = config.eth->static_ip;

            if (const auto result = wifi_set_esp_interface_dns(ESP_IF_ETH, config.eth->static_dns); result != ESP_OK)
            {
                ESP_LOGE(TAG, "wifi_set_esp_interface_dns() for ETH failed with %s", esp_err_to_name(result));
                //return result;
            }
            else
                last_eth_static_dns = config.eth->static_dns;
        }
    }

    return {};
}
#endif
} // namespace
} // namespace wifi_stack
