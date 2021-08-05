#include "espwifistack.h"

#include "sdkconfig.h"
#define LOG_LOCAL_LEVEL CONFIG_LOG_LOCAL_LEVEL_WIFI_STACK

// system includes
#include <optional>
#include <string>
#include <queue>
#include <functional>
#include <atomic>

// esp-idf includes
#include <esp_log.h>
#include <dhcpserver/dhcpserver_options.h>
#include <lwip/dns.h>
#if LWIP_IPV6 && LWIP_IPV6_DHCP6_STATELESS
#include <lwip/dhcp6.h>
#include <esp_netif_net_stack.h>
#endif

// 3rdparty lib includes
#include <fmt/core.h>

// local includes
#include "strutils.h"
#include "delayedconstruction.h"
#include "wrappers/event_group.h"
#include "wrappers/queue.h"
#include "strutils.h"
#include "tickchrono.h"
#include "cpputils.h"
#include "cleanuphelper.h"

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
ap_config last_ap_config;

std::string lastWifisChecksum;

bool wasReallyScanning{};

constexpr auto AP_STARTED_BIT    = BIT0;
constexpr auto AP_HAS_IP6_BIT    = BIT1;
constexpr auto AP_HAS_CLIENT_BIT = BIT2;
constexpr auto STA_STARTED_BIT   = BIT3;
constexpr auto STA_CONNECTED_BIT = BIT4;
constexpr auto STA_HAS_IP_BIT    = BIT5;
constexpr auto STA_HAS_IP6_BIT   = BIT6;
constexpr auto ETH_STARTED_BIT   = BIT7;
constexpr auto ETH_CONNECTED_BIT = BIT8;
constexpr auto ETH_HAS_IP_BIT    = BIT9;
constexpr auto ETH_HAS_IP6_BIT   = BIT10;
constexpr auto WIFI_SCANNING_BIT = BIT11;
constexpr auto WIFI_SCAN_DONE_BIT= BIT12;
constexpr auto WIFI_DNS_IDLE_BIT = BIT13;
constexpr auto WIFI_DNS_DONE_BIT = BIT14;

// generic
cpputils::DelayedConstruction<espcpputils::queue> goe_wifi_event_queue;
cpputils::DelayedConstruction<espcpputils::event_group> goe_wifi_event_group;
} // namespace
esp_netif_t* esp_netifs[ESP_IF_MAX] = {NULL, NULL, NULL};
namespace {
bool lowLevelInitDone = false;
bool _esp_wifi_started = false;
bool _long_range = false;
wifi_ps_type_t _sleepEnabled = WIFI_PS_MIN_MODEM;

// sta
std::atomic<WiFiStaStatus> _sta_status{WiFiStaStatus::WL_NO_SHIELD};
std::optional<espchrono::millis_clock::time_point> _wifiConnectFailFlag;
uint8_t _wifiConnectFailCounter{};

// scan
std::optional<espchrono::millis_clock::time_point> scanStarted;
espchrono::milliseconds32 scanTimeout = 10s;
std::optional<scan_result> _scanResult;
bool scanResultChangedFlag{};

struct ConnectPlanItem
{
    wifi_entry config;
    uint8_t channel;
    wifi_auth_mode_t authmode;
    int8_t rssi;
    mac_t bssid;
};
std::vector<ConnectPlanItem> connectPlan;

} // namespace

const WiFiState &wifiStateMachineState{_wifiState};
cpputils::Signal<> scanResultChanged{};
const std::optional<espchrono::millis_clock::time_point> &lastStaSwitchedFromConnected{_lastStaSwitchedFromConnected};
const std::optional<espchrono::millis_clock::time_point> &lastStaSwitchedToConnected{_lastStaSwitchedToConnected};

namespace {

#define GoeWifiEventIdValues(x) \
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

DECLARE_TYPESAFE_ENUM(GoeWifiEventId, : int32_t, GoeWifiEventIdValues)
IMPLEMENT_TYPESAFE_ENUM(GoeWifiEventId, : int32_t, GoeWifiEventIdValues)

static_assert(sizeof(GoeWifiEventId) == 4);

struct goe_wifi_event_t {
    GoeWifiEventId event_id;
    union {
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
#ifdef ETHERNET
        esp_eth_handle_t eth_connected;
#endif
        wifi_sta_config_t prov_cred_recv;
#ifdef PROVISIONING
        wifi_prov_sta_fail_reason_t prov_fail_reason;
#endif
    };
};

const char * system_event_reasons[] = { "UNSPECIFIED", "AUTH_EXPIRE", "AUTH_LEAVE", "ASSOC_EXPIRE", "ASSOC_TOOMANY", "NOT_AUTHED", "NOT_ASSOCED", "ASSOC_LEAVE", "ASSOC_NOT_AUTHED", "DISASSOC_PWRCAP_BAD", "DISASSOC_SUPCHAN_BAD", "UNSPECIFIED", "IE_INVALID", "MIC_FAILURE", "4WAY_HANDSHAKE_TIMEOUT", "GROUP_KEY_UPDATE_TIMEOUT", "IE_IN_4WAY_DIFFERS", "GROUP_CIPHER_INVALID", "PAIRWISE_CIPHER_INVALID", "AKMP_INVALID", "UNSUPP_RSN_IE_VERSION", "INVALID_RSN_IE_CAP", "802_1X_AUTH_FAILED", "CIPHER_SUITE_REJECTED", "BEACON_TIMEOUT", "NO_AP_FOUND", "AUTH_FAIL", "ASSOC_FAIL", "HANDSHAKE_TIMEOUT", "CONNECTION_FAIL" };
#define reason2str(r) ((r>176)?system_event_reasons[r-176]:system_event_reasons[r-1])

int goe_wifi_set_status_bits(int bits)
{
    if (!goe_wifi_event_group.constructed())
        return 0;

    return goe_wifi_event_group->setBits(bits);
}

int goe_wifi_clear_status_bits(int bits)
{
    if (!goe_wifi_event_group.constructed())
        return 0;

    return goe_wifi_event_group->clearBits(bits);
}

int goe_wifi_get_status_bits()
{
    if (!goe_wifi_event_group.constructed())
        return 0;

    return goe_wifi_event_group->getBits();
}

int goe_wifi_wait_status_bits(int bits, espcpputils::ticks timeout)
{
    if (!goe_wifi_event_group.constructed())
        return 0;

    return goe_wifi_event_group->waitBits(
        bits,  // The bits within the event group to wait for.
        pdFALSE,         // BIT_0 and BIT_4 should be cleared before returning.
        pdTRUE,        // Don't wait for both bits, either bit will do.
        timeout.count()) & bits; // Wait a maximum of 100ms for either bit to be set.
}

esp_err_t goe_wifi_set_esp_interface_ip(esp_interface_t interface, const std::optional<static_ip_config> &ip)
{
    using wifi_stack::toString;

    if (ip)
        ESP_LOGI(TAG, "%s set STATIC ip=%s subnet=%s gateway=%s", toString(interface).c_str(),
                 toString(ip->ip).c_str(), toString(ip->subnet).c_str(), toString(ip->gateway).c_str());
    else
        ESP_LOGI(TAG, "%s set DYNAMIC", toString(interface).c_str());

    if (interface == ESP_IF_WIFI_AP)
    {
        ESP_LOGE(TAG, "setting IP for AP must be done with goe_wifi_set_ap_ip() instead!");
        return ESP_FAIL;
    }

    esp_netif_t * const esp_netif = esp_netifs[interface];

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

esp_err_t goe_wifi_set_esp_interface_dns(esp_interface_t interface, const static_dns_config &dns)
{
    esp_netif_t *esp_netif = esp_netifs[interface];

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

wifi_mode_t goe_wifi_get_mode();
esp_err_t goe_wifi_set_mode(wifi_mode_t m, const config &config);

esp_err_t goe_wifi_enable_sta(bool enable, const config &config)
{
    wifi_mode_t currentMode = goe_wifi_get_mode();
    bool isEnabled = currentMode & WIFI_MODE_STA;

    if (isEnabled == enable)
        return ESP_OK;

    const wifi_mode_t mode = enable ?
                (wifi_mode_t)(currentMode | WIFI_MODE_STA) :
                (wifi_mode_t)(currentMode & (~WIFI_MODE_STA));

    if (const auto result = goe_wifi_set_mode(mode, config); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_set_mode() failed with %s", esp_err_to_name(result));
        return result;
    }

    return ESP_OK;
}

esp_err_t goe_wifi_enable_ap(bool enable, const config &config)
{
    wifi_mode_t currentMode = goe_wifi_get_mode();
    bool isEnabled = currentMode & WIFI_MODE_AP;

    if (isEnabled == enable)
        return ESP_OK;

    esp_err_t result;
    if (enable)
        result = goe_wifi_set_mode((wifi_mode_t)(currentMode | WIFI_MODE_AP), config);
    else
        result = goe_wifi_set_mode((wifi_mode_t)(currentMode & (~WIFI_MODE_AP)), config);

    if (result != ESP_OK)
        ESP_LOGE(TAG, "goe_wifi_set_mode() failed with %s", esp_err_to_name(result));
    return result;
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
    wifi_config.ap.ssid_hidden = false;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    wifi_config.ap.ssid_len = 0;
    wifi_config.ap.ssid[0] = 0;
    wifi_config.ap.password[0] = 0;
    if (!ap_config.ssid.empty())
    {
        auto ssidCutLength = copyStrToBuf(wifi_config.ap.ssid, ap_config.ssid);
        wifi_config.ap.ssid_len = ssidCutLength;

        if (!ap_config.key.empty())
        {
            wifi_config.ap.authmode = ap_config.authmode;
            copyStrToBuf(wifi_config.ap.password, ap_config.key);
        }
    }
    return wifi_config;
}

esp_err_t goe_wifi_set_ap_config(const config &config, const ap_config &ap_config)
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

    if (!goe_wifi_ap_config_equal(conf.ap, conf_current.ap))
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

    if (oldStatus == WiFiStaStatus::WL_CONNECTED && status != WiFiStaStatus::WL_CONNECTED)
        _lastStaSwitchedFromConnected = espchrono::millis_clock::now();
    else if (oldStatus != WiFiStaStatus::WL_CONNECTED && status == WiFiStaStatus::WL_CONNECTED)
        _lastStaSwitchedToConnected = espchrono::millis_clock::now();
}

void goe_wifi_scan_done()
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

    goe_wifi_set_status_bits(WIFI_SCAN_DONE_BIT);
    goe_wifi_clear_status_bits(WIFI_SCANNING_BIT);
}

esp_err_t goe_wifi_sta_disconnect(const config &config, bool wifioff = false, bool eraseap = false);
esp_err_t goe_wifi_sta_begin(const config &config, const wifi_entry &sta_config,
                             int32_t channel = 0, std::optional<mac_t> bssid = {}, bool connect = true);
esp_err_t goe_wifi_sta_restart(const config &config);

void goe_wifi_event_callback(const config &config, const goe_wifi_event_t &event)
{
    ESP_LOGD(TAG, "%d %s", int(event.event_id), toString(event.event_id).c_str());

    switch (event.event_id)
    {
    case GoeWifiEventId::WIFI_SCAN_DONE:
        goe_wifi_scan_done();
        break;
    case GoeWifiEventId::WIFI_STA_START:
        set_sta_status(WiFiStaStatus::WL_IDLE_STATUS);
        goe_wifi_set_status_bits(STA_STARTED_BIT);
        if (const auto result = esp_wifi_set_ps(_sleepEnabled); result != ESP_OK)
            ESP_LOGE(TAG, "esp_wifi_set_ps() failed with %s", esp_err_to_name(result));
        break;
    case GoeWifiEventId::WIFI_STA_STOP:
        set_sta_status(WiFiStaStatus::WL_NO_SHIELD);
        goe_wifi_clear_status_bits(STA_STARTED_BIT | STA_CONNECTED_BIT | STA_HAS_IP_BIT | STA_HAS_IP6_BIT);
        break;
    case GoeWifiEventId::WIFI_STA_CONNECTED:
        set_sta_status(WiFiStaStatus::WL_IDLE_STATUS);
        goe_wifi_set_status_bits(STA_CONNECTED_BIT);
        //esp_netif_create_ip6_linklocal(esp_netifs[ESP_IF_WIFI_STA]);
        break;
    case GoeWifiEventId::WIFI_STA_DISCONNECTED:
    {
        const std::string_view ssid {
            (const char *)event.wifi_sta_disconnected.ssid,
            event.wifi_sta_disconnected.ssid_len
        };

        const auto reason = event.wifi_sta_disconnected.reason;
        ESP_LOGW(TAG, "WIFI_STA_DISCONNECTED ssid=\"%.*s\" bssid=%s reason=%u(%s)",
                 ssid.size(), ssid.data(),
                 toString(mac_t{event.wifi_sta_disconnected.bssid}).c_str(),
                 reason, reason2str(reason));

        switch (reason)
        {
        case WIFI_REASON_AUTH_EXPIRE:
            break;
        case WIFI_REASON_NO_AP_FOUND:
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_ASSOC_FAIL:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_BEACON_TIMEOUT:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
        default:
        {
            const auto sta_status = get_sta_status();
            if (sta_status != WiFiStaStatus::WL_DISCONNECTING)
            {
                ESP_LOGW(TAG, "setting fail flag");
                _wifiConnectFailFlag = espchrono::millis_clock::now();
            }
            switch (sta_status)
            {
            case WiFiStaStatus::WL_CONNECTED: set_sta_status(WiFiStaStatus::WL_CONNECTION_LOST); break;
            case WiFiStaStatus::WL_CONNECTING: set_sta_status(WiFiStaStatus::WL_CONNECT_FAILED); break;
            case WiFiStaStatus::WL_DISCONNECTING:
            default:
                set_sta_status(WiFiStaStatus::WL_DISCONNECTED);
                break;
            }
            break;
        }
        }

        goe_wifi_clear_status_bits(STA_CONNECTED_BIT | STA_HAS_IP_BIT | STA_HAS_IP6_BIT);
        if (((reason == WIFI_REASON_AUTH_EXPIRE) ||
            (reason >= WIFI_REASON_BEACON_TIMEOUT && reason != WIFI_REASON_AUTH_FAIL)))
        {
            if (const auto result = goe_wifi_sta_disconnect(config); result != ESP_OK)
                ESP_LOGE(TAG, "goe_wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
            if (const auto result = goe_wifi_sta_restart(config); result != ESP_OK)
                ESP_LOGE(TAG, "goe_wifi_sta_restart() failed with %s", esp_err_to_name(result));
        }

        break;
    }
    case GoeWifiEventId::WIFI_STA_GOT_IP:
        ESP_LOGI(TAG, "WIFI_STA_GOT_IP ip=%s netmask=%s gw=%s",
                 wifi_stack::toString(event.got_ip.ip_info.ip).c_str(),
                 wifi_stack::toString(event.got_ip.ip_info.netmask).c_str(),
                 wifi_stack::toString(event.got_ip.ip_info.gw).c_str()
        );
        set_sta_status(WiFiStaStatus::WL_CONNECTED);
        goe_wifi_set_status_bits(STA_HAS_IP_BIT | STA_CONNECTED_BIT);
        break;
    case GoeWifiEventId::WIFI_STA_LOST_IP:
        ESP_LOGW(TAG, "WIFI_STA_LOST_IP");
        set_sta_status(WiFiStaStatus::WL_IDLE_STATUS);
        goe_wifi_clear_status_bits(STA_HAS_IP_BIT);
        break;
    case GoeWifiEventId::WIFI_AP_START:
        goe_wifi_set_status_bits(AP_STARTED_BIT);
        break;
    case GoeWifiEventId::WIFI_AP_STOP:
        goe_wifi_clear_status_bits(AP_STARTED_BIT | AP_HAS_CLIENT_BIT);
        break;
    case GoeWifiEventId::WIFI_AP_STACONNECTED:
        goe_wifi_set_status_bits(AP_HAS_CLIENT_BIT);
        break;
    case GoeWifiEventId::WIFI_AP_STADISCONNECTED:
        wifi_sta_list_t clients;
        if (esp_wifi_ap_get_sta_list(&clients) != ESP_OK || !clients.num)
            goe_wifi_clear_status_bits(AP_HAS_CLIENT_BIT);
        break;
    case GoeWifiEventId::ETH_START:
        goe_wifi_set_status_bits(ETH_STARTED_BIT);
        break;
    case GoeWifiEventId::ETH_STOP:
        goe_wifi_clear_status_bits(ETH_STARTED_BIT | ETH_CONNECTED_BIT | ETH_HAS_IP_BIT | ETH_HAS_IP6_BIT);
        break;
    case GoeWifiEventId::ETH_CONNECTED:
        goe_wifi_set_status_bits(ETH_CONNECTED_BIT);
        break;
    case GoeWifiEventId::ETH_DISCONNECTED:
        goe_wifi_clear_status_bits(ETH_CONNECTED_BIT | ETH_HAS_IP_BIT | ETH_HAS_IP6_BIT);
        break;
    case GoeWifiEventId::ETH_GOT_IP:
        ESP_LOGI(TAG, "ETH_GOT_IP ip=%s netmask=%s gw=%s",
                 wifi_stack::toString(event.got_ip.ip_info.ip).c_str(),
                 wifi_stack::toString(event.got_ip.ip_info.netmask).c_str(),
                 wifi_stack::toString(event.got_ip.ip_info.gw).c_str()
        );
        goe_wifi_set_status_bits(ETH_CONNECTED_BIT | ETH_HAS_IP_BIT);
        break;
    case GoeWifiEventId::WIFI_STA_GOT_IP6:
        ESP_LOGI(TAG, "WIFI_STA_GOT_IP6 index=%d zone=%d ip=" IPV6STR,
                 event.got_ip6.ip_index, event.got_ip6.ip6_info.ip.zone, IPV62STR(event.got_ip6.ip6_info.ip));
        goe_wifi_set_status_bits(STA_CONNECTED_BIT | STA_HAS_IP6_BIT);
        break;
    case GoeWifiEventId::WIFI_AP_GOT_IP6:
        ESP_LOGI(TAG, "WIFI_AP_GOT_IP6 index=%d zone=%d ip=" IPV6STR,
                 event.got_ip6.ip_index, event.got_ip6.ip6_info.ip.zone, IPV62STR(event.got_ip6.ip6_info.ip));
        goe_wifi_set_status_bits(AP_HAS_IP6_BIT);
        break;
    case GoeWifiEventId::ETH_GOT_IP6:
        ESP_LOGI(TAG, "ETH_GOT_IP6 index=%d zone=%d ip=" IPV6STR,
                 event.got_ip6.ip_index, event.got_ip6.ip6_info.ip.zone, IPV62STR(event.got_ip6.ip6_info.ip));
        goe_wifi_set_status_bits(ETH_CONNECTED_BIT | ETH_HAS_IP6_BIT);
        break;
#ifdef SMARTCONFIG
    case GoeWifiEventId::SC_GOT_SSID_PSWD:
        goe_wifi_sta_begin(
            (const char *)event.sc_got_ssid_pswd.ssid,
            (const char *)event.sc_got_ssid_pswd.password,
            0,
            ((event.sc_got_ssid_pswd.bssid_set == true)?event.sc_got_ssid_pswd.bssid:NULL)
        );
        break;
    case GoeWifiEventId::SC_SEND_ACK_DONE:
        esp_smartconfig_stop();
        _smartConfigDone = true;
        break;
#endif
    default:;
    }
}

esp_err_t goe_wifi_post_event(std::unique_ptr<const goe_wifi_event_t> event)
{
    if (!event)
    {
        ESP_LOGE(TAG, "invalid event");
        return ESP_FAIL;
    }

    const auto ptr = event.get();
    if (const auto result = goe_wifi_event_queue->send(&ptr, portMAX_DELAY); result != pdTRUE)
    {
        ESP_LOGE(TAG, "goe_wifi_event_queue->send() failed with %i", result);
        return ESP_FAIL;
    }

    event.release(); // ownership taken over by goe_wifi_event_queue

    return ESP_OK;
}

void goe_wifi_event_cb(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ESP_LOGI(TAG, "%s %i", event_base, event_id);

    auto goe_wifi_event = std::make_unique<goe_wifi_event_t>();
    goe_wifi_event->event_id = GoeWifiEventId::MAX;

    /*
     * STA
     * */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGV(TAG, "STA Started");
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_STA_START;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_STOP) {
        ESP_LOGV(TAG, "STA Stopped");
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_STA_STOP;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_AUTHMODE_CHANGE) {
        wifi_event_sta_authmode_change_t * event = (wifi_event_sta_authmode_change_t*)event_data;
        ESP_LOGV(TAG, "STA Auth Mode Changed: From: %s, To: %s", wifi_stack::toString(event->old_mode).c_str(), wifi_stack::toString(event->new_mode).c_str());
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_STA_AUTHMODE_CHANGE;
        std::memcpy(&goe_wifi_event->wifi_sta_authmode_change, event_data, sizeof(wifi_event_sta_authmode_change_t)); // TODO replace with operator=
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t * event = (wifi_event_sta_connected_t*)event_data;
        ESP_LOGV(TAG, "STA Connected: SSID: %s, BSSID: " MACSTR ", Channel: %u, Auth: %s", event->ssid, MAC2STR(event->bssid), event->channel, wifi_stack::toString(event->authmode).c_str());
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_STA_CONNECTED;
        std::memcpy(&goe_wifi_event->wifi_sta_connected, event_data, sizeof(wifi_event_sta_connected_t)); // TODO replace with operator=
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t * event = (wifi_event_sta_disconnected_t*)event_data;
        ESP_LOGV(TAG, "STA Disconnected: SSID: %s, BSSID: " MACSTR ", Reason: %u", event->ssid, MAC2STR(event->bssid), event->reason);
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_STA_DISCONNECTED;
        std::memcpy(&goe_wifi_event->wifi_sta_disconnected, event_data, sizeof(wifi_event_sta_disconnected_t)); // TODO replace with operator=
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGV(TAG, "STA Got %sIP:" IPSTR, event->ip_changed?"New ":"Same ", IP2STR(&event->ip_info.ip));
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_STA_GOT_IP;
        std::memcpy(&goe_wifi_event->got_ip, event_data, sizeof(ip_event_got_ip_t)); // TODO replace with operator=
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        ESP_LOGV(TAG, "STA IP Lost");
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_STA_LOST_IP;

    /*
     * SCAN
     * */
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        wifi_event_sta_scan_done_t * event = (wifi_event_sta_scan_done_t*)event_data;
        ESP_LOGV(TAG, "SCAN Done: ID: %u, Status: %u, Results: %u", event->scan_id, event->status, event->number);
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_SCAN_DONE;
        std::memcpy(&goe_wifi_event->wifi_scan_done, event_data, sizeof(wifi_event_sta_scan_done_t)); // TODO replace with operator=

    /*
     * AP
     * */
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGV(TAG, "AP Started");
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_AP_START;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP) {
        ESP_LOGV(TAG, "AP Stopped");
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_AP_STOP;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_PROBEREQRECVED) {
        wifi_event_ap_probe_req_rx_t * event = (wifi_event_ap_probe_req_rx_t*)event_data;
        ESP_LOGV(TAG, "AP Probe Request: RSSI: %d, MAC: " MACSTR, event->rssi, MAC2STR(event->mac));
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_AP_PROBEREQRECVED;
        std::memcpy(&goe_wifi_event->wifi_ap_probereqrecved, event_data, sizeof(wifi_event_ap_probe_req_rx_t)); // TODO replace with operator=
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGV(TAG, "AP Station Connected: MAC: " MACSTR ", AID: %d", MAC2STR(event->mac), event->aid);
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_AP_STACONNECTED;
        std::memcpy(&goe_wifi_event->wifi_ap_staconnected, event_data, sizeof(wifi_event_ap_staconnected_t)); // TODO replace with operator=
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGV(TAG, "AP Station Disconnected: MAC: " MACSTR ", AID: %d", MAC2STR(event->mac), event->aid);
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_AP_STADISCONNECTED;
        std::memcpy(&goe_wifi_event->wifi_ap_stadisconnected, event_data, sizeof(wifi_event_ap_stadisconnected_t)); // TODO replace with operator=
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t * event = (ip_event_ap_staipassigned_t*)event_data;
        ESP_LOGV(TAG, "AP Station IP Assigned:" IPSTR, IP2STR(&event->ip));
        goe_wifi_event->event_id = GoeWifiEventId::WIFI_AP_STAIPASSIGNED;
        std::memcpy(&goe_wifi_event->wifi_ap_staipassigned, event_data, sizeof(ip_event_ap_staipassigned_t)); // TODO replace with operator=

#ifdef ETHERNET
    /*
     * ETH
     * */
    } else if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGV(TAG, "Ethernet Link Up");
        esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
        goe_wifi_event->event_id = GoeWifiEventId::ETH_CONNECTED;
        std::memcpy(&goe_wifi_event->eth_connected, event_data, sizeof(esp_eth_handle_t)); // TODO replace with operator=
    } else if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGV(TAG, "Ethernet Link Down");
        goe_wifi_event->event_id = GoeWifiEventId::ETH_DISCONNECTED;
    } else if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_START) {
        ESP_LOGV(TAG, "Ethernet Started");
        goe_wifi_event->event_id = GoeWifiEventId::ETH_START;
    } else if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_STOP) {
        ESP_LOGV(TAG, "Ethernet Stopped");
        goe_wifi_event->event_id = GoeWifiEventId::ETH_STOP;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGV(TAG, "Ethernet got %sip:" IPSTR, event->ip_changed?"new":"", IP2STR(&event->ip_info.ip));
        goe_wifi_event->event_id = GoeWifiEventId::ETH_GOT_IP;
        std::memcpy(&goe_wifi_event->got_ip, event_data, sizeof(ip_event_got_ip_t)); // TODO replace with operator=
#endif

    /*
     * IPv6
     * */
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_GOT_IP6) {
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
        case ESP_IF_WIFI_STA: goe_wifi_event->event_id = GoeWifiEventId::WIFI_STA_GOT_IP6; break;
        case ESP_IF_WIFI_AP:  goe_wifi_event->event_id = GoeWifiEventId::WIFI_AP_GOT_IP6;  break;
        case ESP_IF_ETH:      goe_wifi_event->event_id = GoeWifiEventId::ETH_GOT_IP6;      break;
        default:;
        }
        std::memcpy(&goe_wifi_event->got_ip6, event_data, sizeof(ip_event_got_ip6_t)); // TODO replace with operator=

    /*
     * WPS
     * */
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_SUCCESS) {
        goe_wifi_event->event_id = GoeWifiEventId::WPS_ER_SUCCESS;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_FAILED) {
        wifi_event_sta_wps_fail_reason_t * event = (wifi_event_sta_wps_fail_reason_t*)event_data;
        CPP_UNUSED(event)
        goe_wifi_event->event_id = GoeWifiEventId::WPS_ER_FAILED;
        std::memcpy(&goe_wifi_event->wps_fail_reason, event_data, sizeof(wifi_event_sta_wps_fail_reason_t)); // TODO replace with operator=
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_TIMEOUT) {
        goe_wifi_event->event_id = GoeWifiEventId::WPS_ER_TIMEOUT;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_PIN) {
        wifi_event_sta_wps_er_pin_t * event = (wifi_event_sta_wps_er_pin_t*)event_data;
        CPP_UNUSED(event)
        goe_wifi_event->event_id = GoeWifiEventId::WPS_ER_PIN;
        std::memcpy(&goe_wifi_event->wps_er_pin, event_data, sizeof(wifi_event_sta_wps_er_pin_t)); // TODO replace with operator=
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP) {
        goe_wifi_event->event_id = GoeWifiEventId::WPS_ER_PBC_OVERLAP;


#ifdef SMARTCONFIG
    /*
     * SMART CONFIG
     * */
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGV(TAG, "SC Scan Done");
        goe_wifi_event->event_id = GoeWifiEventId::SC_SCAN_DONE;
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGV(TAG, "SC Found Channel");
        goe_wifi_event->event_id = GoeWifiEventId::SC_FOUND_CHANNEL;
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        smartconfig_event_got_ssid_pswd_t *event = (smartconfig_event_got_ssid_pswd_t *)event_data;
        ESP_LOGV(TAG, "SC: SSID: %s, Password: %s", (const char *)event->ssid, (const char *)event->password);
        goe_wifi_event->event_id = GoeWifiEventId::SC_GOT_SSID_PSWD;
        std::memcpy(&goe_wifi_event->sc_got_ssid_pswd, event_data, sizeof(smartconfig_event_got_ssid_pswd_t)); // TODO replace with operator=

    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        ESP_LOGV(TAG, "SC Send Ack Done");
        goe_wifi_event->event_id = GoeWifiEventId::SC_SEND_ACK_DONE;
#endif

#ifdef PROVISIONING
    /*
     * Provisioning
     * */
    } else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_INIT) {
        ESP_LOGV(TAG, "Provisioning Initialized!");
        goe_wifi_event->event_id = GoeWifiEventId::PROV_INIT;
    } else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_DEINIT) {
        ESP_LOGV(TAG, "Provisioning Uninitialized!");
        goe_wifi_event->event_id = GoeWifiEventId::PROV_DEINIT;
    } else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_START) {
        ESP_LOGV(TAG, "Provisioning Start!");
        goe_wifi_event->event_id = GoeWifiEventId::PROV_START;
    } else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_END) {
        ESP_LOGV(TAG, "Provisioning End!");
        wifi_prov_mgr_deinit();
        goe_wifi_event->event_id = GoeWifiEventId::PROV_END;
    } else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_CRED_RECV) {
        wifi_sta_config_t *event = (wifi_sta_config_t *)event_data;
        ESP_LOGV(TAG, "Provisioned Credentials: SSID: %s, Password: %s", (const char *) event->ssid, (const char *) event->password);
        goe_wifi_event->event_id = GoeWifiEventId::PROV_CRED_RECV;
        std::memcpy(&goe_wifi_event->prov_cred_recv, event_data, sizeof(wifi_sta_config_t)); // TODO replace with operator=
    } else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_CRED_FAIL) {
        wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
        ESP_LOGE(TAG, "Provisioning Failed: Reason : %s", (*reason == WIFI_PROV_STA_AUTH_ERROR)?"Authentication Failed":"AP Not Found");
        goe_wifi_event->event_id = GoeWifiEventId::PROV_CRED_FAIL;
        std::memcpy(&goe_wifi_event->prov_fail_reason, event_data, sizeof(wifi_prov_sta_fail_reason_t)); // TODO replace with operator=
    } else if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_CRED_SUCCESS) {
        ESP_LOGV(TAG, "Provisioning Success!");
        goe_wifi_event->event_id = GoeWifiEventId::PROV_CRED_SUCCESS;
#endif
    }

    if (goe_wifi_event->event_id < GoeWifiEventId::MAX)
        goe_wifi_post_event(std::move(goe_wifi_event));
}

esp_err_t goe_wifi_start_network_event_task(const config &config)
{
    if (!goe_wifi_event_group.constructed())
    {
        goe_wifi_event_group.construct();
        if (!goe_wifi_event_group->handle)
        {
            goe_wifi_event_group.destruct();
            ESP_LOGE(TAG, "Network Event Group Create Failed!");
            return ESP_FAIL;
        }

        goe_wifi_event_group->setBits(WIFI_DNS_IDLE_BIT);
    }

    if (!goe_wifi_event_queue.constructed())
    {
        goe_wifi_event_queue.construct(UBaseType_t{32}, sizeof(goe_wifi_event_t*));
        if (!goe_wifi_event_queue->handle)
        {
            goe_wifi_event_queue.destruct();
            ESP_LOGE(TAG, "Network Event Queue Create Failed!");
            return ESP_FAIL;
        }
    }

    if (const auto result = esp_event_loop_create_default(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_event_loop_create_default() failed with %s", esp_err_to_name(result));
        if (result != ESP_ERR_INVALID_STATE)
            return result;
    }

    if (const auto result = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &goe_wifi_event_cb, NULL, NULL); result != ESP_OK)
    {
        ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "WIFI_EVENT", esp_err_to_name(result));
        return result;
    }

    if (const auto result = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &goe_wifi_event_cb, NULL, NULL); result != ESP_OK)
    {
        ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "IP_EVENT", esp_err_to_name(result));
        return result;
    }

#ifdef SMARTCONFIG
    if (const auto result = esp_event_handler_instance_register(SC_EVENT, ESP_EVENT_ANY_ID, &goe_wifi_event_cb, NULL, NULL); result != ESP_OK)
    {
        ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "SC_EVENT", esp_err_to_name(result));
        return result;
    }
#endif

#ifdef ETHERNET
    if (const auto result = esp_event_handler_instance_register(ETH_EVENT, ESP_EVENT_ANY_ID, &goe_wifi_event_cb, NULL, NULL); result != ESP_OK)
    {
        ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "ETH_EVENT", esp_err_to_name(result));
        return result;
    }
#endif

#ifdef PROVISIONING
    if (const auto result = esp_event_handler_instance_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &goe_wifi_event_cb, NULL, NULL); result != ESP_OK)
    {
        ESP_LOGE(TAG, "event_handler_instance_register() for %s failed with %s", "WIFI_PROV_EVENT", esp_err_to_name(result));
        return result;
    }
#endif

    return ESP_OK;
}

esp_err_t goe_wifi_tcpip_init(const config &config)
{
    static bool initialized = false;

    if (initialized)
        return ESP_OK;

#if CONFIG_IDF_TARGET_ESP32
    if (const auto mac = get_default_mac_addr())
    {
        if (const auto result = set_base_mac_addr(*mac))
        {
        }
        else
        {
            ESP_LOGE(TAG, "set_base_mac_addr() failed: %.*s", result.error().size(), result.error().data());
        }
    }
    else
    {
        ESP_LOGE(TAG, "get_default_mac_addr() failed: %.*s", mac.error().size(), mac.error().data());
    }
#endif

    if (const auto result = esp_netif_init(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_netif_init() failed with %s", esp_err_to_name(result));
        return result;
    }

    if (const auto result = goe_wifi_start_network_event_task(config); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_start_network_event_task() failed with %s", esp_err_to_name(result));
        return result;
    }

    initialized = true;
    return ESP_OK;
}

esp_err_t goe_wifi_low_level_init(const config &config)
{
    if (lowLevelInitDone)
        return ESP_OK;

    if (const auto result = goe_wifi_tcpip_init(config); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_tcpip_init() failed with %s", esp_err_to_name(result));
        return result;
    }

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

    lowLevelInitDone = true;
    return ESP_OK;
}

esp_err_t goe_wifi_start()
{
    if (_esp_wifi_started)
        return ESP_OK;

    if (const auto result = esp_wifi_start(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_start() failed with %s", esp_err_to_name(result));
        return result;
    }

    _esp_wifi_started = true;

    return ESP_OK;
}

esp_err_t goe_wifi_low_level_deinit()
{
    if (!lowLevelInitDone)
        return ESP_OK;

    if (const auto result = esp_wifi_deinit(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_deinit() failed with %s", esp_err_to_name(result));
        return result;
    }

    lowLevelInitDone = false;
    return ESP_OK;
}

esp_err_t goe_wifi_stop()
{
    if (!_esp_wifi_started)
        return ESP_OK;

    if (const auto result = esp_wifi_stop(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_stop() failed with %s", esp_err_to_name(result));
        return result;
    }

    _esp_wifi_started = false;

    if (const auto result = goe_wifi_low_level_deinit(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_low_level_deinit() failed with %s", esp_err_to_name(result));
        return result;
    }

    return ESP_OK;
}

wifi_mode_t goe_wifi_get_mode()
{
    if (!lowLevelInitDone || !_esp_wifi_started)
        return WIFI_MODE_NULL;

    wifi_mode_t mode;
    if (const auto result = esp_wifi_get_mode(&mode); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_get_mode() returned %s", esp_err_to_name(result));
        return WIFI_MODE_NULL;
    }

    return mode;
}

esp_err_t goe_wifi_set_mode(wifi_mode_t m, const config &config)
{
    wifi_mode_t cm = goe_wifi_get_mode();
    if (cm == m)
        return ESP_OK;

    if (!cm && m)
    {
        if (const auto result = goe_wifi_low_level_init(config); result != ESP_OK)
        {
            ESP_LOGE(TAG, "goe_wifi_low_level_init() failed with %s", esp_err_to_name(result));
            return result;
        }
    }
    else if (cm && !m)
    {
        if (const auto result = goe_wifi_stop(); result != ESP_OK)
        {
            ESP_LOGE(TAG, "goe_wifi_stop() failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    if (m & WIFI_MODE_STA)
    {
        if (const auto result = esp_netif_set_hostname(esp_netifs[ESP_IF_WIFI_STA], config.hostname.c_str()); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_netif_set_hostname() \"%s\" failed with %s", config.hostname.c_str(), esp_err_to_name(result));
            return result;
        }
    }

    if (const auto result = esp_wifi_set_mode(m); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_mode() failed with %s", esp_err_to_name(result));
        return result;
    }

    if (_long_range)
    {
        if (m & WIFI_MODE_STA)
        {
            if (const auto result = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR); result != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_wifi_set_protocol() for STA long range failed with %s", esp_err_to_name(result));
                return result;
            }
        }
        if (m & WIFI_MODE_AP)
        {
            if (const auto result = esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR); result != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_wifi_set_protocol() for AP long range failed with %s", esp_err_to_name(result));
                return result;
            }
        }
    }

    if (const auto result = goe_wifi_start(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_start() failed with %s", esp_err_to_name(result));
        return result;
    }

    return ESP_OK;
}

esp_err_t goe_wifi_set_ap_ip(const config &config, const static_ip_config &ip)
{
    using wifi_stack::toString;

    ESP_LOGI(TAG, "ip=%s subnet=%s gateway=%s", toString(ip.ip).c_str(), toString(ip.subnet).c_str(), toString(ip.gateway).c_str());

    esp_netif_t * const esp_netif = esp_netifs[ESP_IF_WIFI_AP];

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
                    vTaskDelay(std::chrono::ceil<espcpputils::ticks>(50ms).count());
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

        if (const auto result = tcpip_adapter_dhcps_option(ESP_NETIF_OP_SET, ESP_NETIF_REQUESTED_IP_ADDRESS, &lease, sizeof(dhcps_lease_t)); result != ESP_OK)
        {
            ESP_LOGE(TAG, "tcpip_adapter_dhcps_option() failed with %s", esp_err_to_name(result));
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

esp_err_t goe_wifi_sta_begin(const config &config, const wifi_entry &sta_config,
                             int32_t channel, std::optional<mac_t> bssid, bool connect)
{
    if (const auto result = goe_wifi_enable_sta(true, config); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_enable_sta() failed with %s", esp_err_to_name(result));
        return result;
    }

    if (sta_config.ssid.empty())
    {
        ESP_LOGE(TAG, "SSID missing!");
        return ESP_FAIL;
    }

    if (sta_config.ssid.size() > 32)
    {
        ESP_LOGE(TAG, "SSID too long! (size=%zd)", sta_config.ssid.size());
        return ESP_FAIL;
    }

    if (!sta_config.key.empty())
    {
        if (sta_config.key.size() < 8)
        {
            ESP_LOGE(TAG, "key too short! (size=%zd)", sta_config.key.size());
            return ESP_FAIL;
        }

        if (sta_config.key.size() > 64)
        {
            ESP_LOGE(TAG, "key too long! (size=%zd)", sta_config.key.size());
            return ESP_FAIL;
        }
    }

    wifi_config_t conf = make_sta_config(sta_config.ssid, sta_config.key, config.sta.min_rssi, bssid, channel);

    wifi_config_t current_conf;
    if (const auto result = esp_wifi_get_config(WIFI_IF_STA, &current_conf); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_get_config() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    if (!goe_wifi_sta_config_equal(current_conf.sta, conf.sta))
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
    else if (get_sta_status() == WiFiStaStatus::WL_CONNECTED)
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

    if (const auto result = goe_wifi_set_esp_interface_ip(ESP_IF_WIFI_STA, sta_config.static_ip); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_set_esp_interface_ip() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    last_sta_static_ip = sta_config.static_ip;

    if (const auto result = goe_wifi_set_esp_interface_dns(ESP_IF_WIFI_STA, sta_config.static_dns); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_set_esp_interface_dns() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    last_sta_static_dns = sta_config.static_dns;

    if (connect)
    {
        _wifiConnectFailFlag = std::nullopt;

        if (const auto result = esp_wifi_connect(); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_connect() failed with %s", esp_err_to_name(result));
            return result;
        }

        set_sta_status(WiFiStaStatus::WL_CONNECTING);
    }

    return ESP_OK;
}

esp_err_t goe_wifi_sta_restart(const config &config)
{
    if (const auto result = goe_wifi_enable_sta(true, config); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_enable_sta() failed with %s", esp_err_to_name(result));
        return result;
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

    if (const auto result = goe_wifi_set_esp_interface_ip(ESP_IF_WIFI_STA, /*config.sta.static_ip*/last_sta_static_ip); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_set_esp_interface_ip() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    //last_sta_static_ip = config.sta.static_ip;

    if (const auto result = goe_wifi_set_esp_interface_dns(ESP_IF_WIFI_STA, /*config.sta.static_dns*/last_sta_static_dns); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_set_esp_interface_dns() for STA failed with %s", esp_err_to_name(result));
        return result;
    }

    //last_sta_static_dns = config.sta.static_dns;

    if (get_sta_status() != WiFiStaStatus::WL_CONNECTED)
    {
        _wifiConnectFailFlag = std::nullopt;

        if (const auto result = esp_wifi_connect(); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_connect() failed with %s", esp_err_to_name(result));
            return result;
        }

        set_sta_status(WiFiStaStatus::WL_CONNECTING);
    }
    else
        ESP_LOGW(TAG, "when already connected?!");

    return ESP_OK;
}

esp_err_t goe_wifi_sta_disconnect(const config &config, bool wifioff, bool eraseap)
{
    ESP_LOGI(TAG, "wifioff=%s eraseap=%s", wifioff?"true":"false", eraseap?"true":"false");

    wifi_config_t conf = make_sta_config({}, {}, 0, {}, 0);

    if (!(goe_wifi_get_mode() & WIFI_MODE_STA))
        return ESP_FAIL;

    if (eraseap)
    {
        if (const auto result = esp_wifi_set_config(WIFI_IF_STA, &conf); result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_wifi_set_config() failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    set_sta_status(WiFiStaStatus::WL_DISCONNECTING);

    if (const auto result = esp_wifi_disconnect(); result != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_disconnect() failed with %s", esp_err_to_name(result));
        return result;
    }

    if (wifioff)
    {
        if (const auto result = goe_wifi_enable_sta(false, config); result != ESP_OK)
        {
            ESP_LOGE(TAG, "goe_wifi_enable_sta() failed with %s", esp_err_to_name(result));
            return result;
        }
    }

    return ESP_OK;
}

std::string calculateWifisChecksum(const config &config)
{
    std::string result;
    for (const auto &wifi : config.sta.wifis)
    {
        result += wifi.ssid;
        result += ",";
        result += wifi.key;
        result += "|";
    }
    return result;
}

esp_err_t goe_wifi_begin_scan(const config &config, bool show_hidden = false, bool passive = false, espchrono::milliseconds32 max_ms_per_chan = 300ms, uint8_t channel = 0)
{
    if (goe_wifi_get_status_bits() & WIFI_SCANNING_BIT)
    {
        ESP_LOGE(TAG, "while scan was still running");
        return ESP_FAIL;
    }

    scanTimeout = max_ms_per_chan * 20;

    if (const auto result = goe_wifi_enable_sta(true, config); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_enable_sta() failed with %s", esp_err_to_name(result));
        return result;
    }

    delete_scan_result();

    wifi_scan_config_t scan_config;
    scan_config.ssid = 0;
    scan_config.bssid = 0;
    scan_config.channel = channel;
    scan_config.show_hidden = show_hidden;

    if (passive)
    {
        scan_config.scan_type = WIFI_SCAN_TYPE_PASSIVE;
        scan_config.scan_time.passive = espchrono::milliseconds32{max_ms_per_chan}.count();
    }
    else
    {
        scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
        scan_config.scan_time.active.min = 100;
        scan_config.scan_time.active.max = espchrono::milliseconds32{max_ms_per_chan}.count();
    }

    if (const auto result = esp_wifi_scan_start(&scan_config, false) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_scan_start() failed with %s", esp_err_to_name(result));
        return result;
    }

    scanStarted = espchrono::millis_clock::now();

    goe_wifi_clear_status_bits(WIFI_SCAN_DONE_BIT);
    goe_wifi_set_status_bits(WIFI_SCANNING_BIT);

    return ESP_OK;
}

void setWifiState(WiFiState newWifiState)
{
    if (newWifiState != _wifiState)
    {
        ESP_LOGI(TAG, "state machine state changed from %s to %s", toString(_wifiState).c_str(), toString(newWifiState).c_str());
        _wifiState = newWifiState;
    }
}

bool buildConnectPlan(const config &config, const scan_result &scanResult);

bool buildConnectPlan(const config &config)
{
    const auto &scanResult = get_scan_result();
    if (!scanResult)
    {
        ESP_LOGE(TAG, "no scan result available!");
        return false;
    }

    return buildConnectPlan(config, *scanResult);
}

bool buildConnectPlan(const config &config, const scan_result &scanResult)
{
    connectPlan.clear();

    std::string foundSsids;

    for (const auto &entry : scanResult.entries)
    {
        std::string_view scanSSID{(const char *)entry.ssid};

        // avoid duplicates
        if (std::any_of(std::begin(connectPlan), std::end(connectPlan), [&scanSSID](const auto &entry){
                        return cpputils::stringEqualsIgnoreCase(entry.config.ssid, scanSSID); }))
            continue;

        const auto iter = std::find_if(std::begin(config.sta.wifis), std::end(config.sta.wifis),
                                       [&scanSSID](const auto &entry){ return cpputils::stringEqualsIgnoreCase(entry.ssid, scanSSID); });
        if (iter != std::end(config.sta.wifis))
        {
            connectPlan.emplace_back(ConnectPlanItem{
                                         .config = *iter,
                                         .channel = entry.primary,
                                         .authmode = entry.authmode,
                                         .rssi = entry.rssi,
                                         .bssid = mac_t{entry.bssid}
                                     });
        }

        if (!foundSsids.empty())
            foundSsids += ", ";
        foundSsids += scanSSID;
    }

    if (!connectPlan.empty())
    {
        using wifi_stack::toString;

        const auto entry = connectPlan.front();
        connectPlan.erase(std::begin(connectPlan));
        ESP_LOGI(TAG, "connecting to %s (auth=%s, key=%s, channel=%i, rssi=%i, bssid=%s)",
                 entry.config.ssid.c_str(),
                 toString(entry.authmode).c_str(),
                 entry.config.key.c_str(),
                 entry.channel,
                 entry.rssi,
                 toString(entry.bssid).c_str()
        );
        _lastConnect = espchrono::millis_clock::now();

        _wifiConnectFailCounter = 0;
        if (const auto result = goe_wifi_sta_begin(config, entry.config, entry.channel, entry.bssid); result != ESP_OK)
            ESP_LOGE(TAG, "goe_wifi_sta_begin() failed with %s", esp_err_to_name(result));
        setWifiState(WiFiState::Connecting);

        return true;
    }
    else
    {
        ESP_LOGW(TAG, "no configured ssid found");
        ESP_LOGW(TAG, "found ssids: %s", foundSsids.c_str());
        std::string configuredSsids;
        for (const auto &config : config.sta.wifis)
        {
            if (!configuredSsids.empty())
                configuredSsids += ", ";
            configuredSsids += config.ssid;
        }
        ESP_LOGW(TAG, "configured ssids: %s", configuredSsids.c_str());
        setWifiState(WiFiState::None);

        return false;
    }
}

void handleWifiEvents(const config &config, TickType_t xTicksToWait)
{
    const goe_wifi_event_t *data{};
    if (const auto result = goe_wifi_event_queue->receive(&data, xTicksToWait); result != pdTRUE)
    {
        if (result != pdFALSE)
            ESP_LOGE(TAG, "goe_wifi_event_queue->receive() failed with %i", result);
        return;
    }

    if (!data)
    {
        ESP_LOGE(TAG, "received nullptr event");
        return;
    }

    std::unique_ptr<const goe_wifi_event_t> event{data};
    goe_wifi_event_callback(config, *event);
}
} // namespace

void init(const config &config)
{
    ESP_LOGI(TAG, "called");

    if (const auto result = goe_wifi_set_mode(WIFI_MODE_APSTA, config); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_set_mode() failed with %s", esp_err_to_name(result));
        return;
    }

    vTaskDelay(std::chrono::ceil<espcpputils::ticks>(100ms).count());

    if (const auto result = goe_wifi_enable_ap(true, config); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_enable_ap() failed with %s", esp_err_to_name(result));
        return;
    }

    if (const auto result = goe_wifi_set_ap_ip(config, config.ap.static_ip); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_set_ap_ip() failed with %s", esp_err_to_name(result));
        return;
    }

    if (const auto result = goe_wifi_set_ap_config(config, config.ap); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_set_ap_config() failed with %s", esp_err_to_name(result));
        return;
    }

    last_ap_config = config.ap;

    _wifiState = WiFiState::None;

    lastWifisChecksum = calculateWifisChecksum(config);
    if (config.wifiEnabled)
    {
        if (const auto result = begin_scan(config); result != ESP_OK)
            ESP_LOGE(TAG, "begin_scan() failed with %s", esp_err_to_name(result));
    }
    else
        ESP_LOGW(TAG, "not connecting, because wifi is not enabled");
}

void update(const config &config)
{
    handleWifiEvents(config, 0);

    if (last_ap_config != config.ap)
    {
        ESP_LOGI(TAG, "AP settings changed, applying new config...");

        if (config.ap.ssid != last_ap_config.ssid)
            ESP_LOGD(TAG, "new ap ssid=\"%s\" old ap ssid=\"%s\"", config.ap.ssid.c_str(), last_ap_config.ssid.c_str());

        if (config.ap.key != last_ap_config.key)
            ESP_LOGD(TAG, "new ap key=\"%s\" old ap key=\"%s\"", config.ap.key.c_str(), last_ap_config.key.c_str());

        if (const auto result = goe_wifi_set_ap_ip(config, config.ap.static_ip); result != ESP_OK)
            ESP_LOGE(TAG, "goe_wifi_set_ap_ip() failed with %s", esp_err_to_name(result));

        if (const auto result = goe_wifi_set_ap_config(config, config.ap); result != ESP_OK)
            ESP_LOGE(TAG, "goe_wifi_set_ap_config() failed with %s", esp_err_to_name(result));

        last_ap_config = config.ap;
    }

    if (scanResultChangedFlag)
    {
        scanResultChangedFlag = false;
        scanResultChanged();
    }

    if (config.wifiEnabled)
    {
        if (_wifiState == WiFiState::None)
        {
            if (get_sta_status() == WiFiStaStatus::WL_CONNECTED)
            {
                ESP_LOGI(TAG, "Unexpected connected!");

                setWifiState(WiFiState::Connected);
            }
            else
            {
                const auto anyWifiConfigured = std::any_of(std::begin(config.sta.wifis), std::end(config.sta.wifis),
                                                           [](const auto &entry){ return !entry.ssid.empty(); });
                if (anyWifiConfigured)
                {
                    if (!lastScanStarted || espchrono::ago(*lastScanStarted) >= 10min)
                    {
                        if (const auto result = begin_scan(config); result != ESP_OK)
                            ESP_LOGE(TAG, "begin_scan() failed with %s", esp_err_to_name(result));
                    }
                    else if (auto newChecksum = calculateWifisChecksum(config); newChecksum != lastWifisChecksum)
                    {
                        ESP_LOGI(TAG, "old wifis config: %s", lastWifisChecksum.c_str());
                        ESP_LOGI(TAG, "new wifis config: %s", newChecksum.c_str());
                        lastWifisChecksum = std::move(newChecksum);

                        if (get_scan_status() == WiFiScanStatus::Finished)
                        {
                            if (const auto &scanResult = get_scan_result(); scanResult && !scanResult->entries.empty())
                            {
                                ESP_LOGI(TAG, "wifi configs changed, building connect plan...");
                                buildConnectPlan(config, *scanResult);
                            }
                            else
                                goto scanAnyways;
                        }
                        else
                        {
                            scanAnyways:
                            ESP_LOGI(TAG, "wifi configs changed, triggering a new scan");
                            if (const auto result = begin_scan(config); result != ESP_OK)
                                ESP_LOGE(TAG, "begin_scan() failed with %s", esp_err_to_name(result));
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

                        if (get_sta_status() != WiFiStaStatus::WL_CONNECTED)
                        {
                            ESP_LOGI(TAG, "Not connected after scan, building connect plan...");
                            buildConnectPlan(config, *scanResult);
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
            if (status == WiFiStaStatus::WL_CONNECTED)
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
            else if (_wifiConnectFailFlag && espchrono::ago(*_wifiConnectFailFlag) >= 5s)
            {
                _wifiConnectFailFlag = std::nullopt;

                if (_wifiConnectFailCounter++ >= 10)
                {
                    ESP_LOGE(TAG, "fail flag was set and fail count exceeded limit");
                    if (const auto result = goe_wifi_sta_disconnect(config, false, true); result != ESP_OK)
                        ESP_LOGE(TAG, "goe_wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
                    setWifiState(WiFiState::None); // results in another scan
                }
                else
                {
                    ESP_LOGW(TAG, "fail flag was set, trying again %hhu", _wifiConnectFailCounter);

                    _lastConnect = espchrono::millis_clock::now();

                    if (const auto result = goe_wifi_sta_disconnect(config); result != ESP_OK)
                        ESP_LOGE(TAG, "goe_wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
                    if (const auto result = goe_wifi_sta_restart(config); result != ESP_OK)
                        ESP_LOGE(TAG, "goe_wifi_sta_restart() failed with %s", esp_err_to_name(result));
                }
            }
            else
            {
                if (espchrono::ago(_lastConnect) >= 20s)
                {
                    ESP_LOGW(TAG, "connecting timed out, building new connect plan... %s", toString(status).c_str());

                    if (const auto result = goe_wifi_sta_disconnect(config, false, true); result != ESP_OK)
                        ESP_LOGE(TAG, "goe_wifi_sta_disconnect() failed with %s", esp_err_to_name(result));

                    buildConnectPlan(config);

                    lastStatus = std::nullopt;
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
            if (status != WiFiStaStatus::WL_CONNECTED)
            {
                ESP_LOGW(TAG, "lost connection: %s", toString(status).c_str());
                if (const auto result = goe_wifi_sta_disconnect(config, false, true); result != ESP_OK)
                    ESP_LOGE(TAG, "goe_wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
                lastWifisChecksum.clear();
                setWifiState(WiFiState::None); // results in another scan
            }
            else if (const auto sta_info = get_sta_ap_info())
            {
                const std::string_view connectedSSID{reinterpret_cast<const char *>(sta_info->ssid)};
                const auto iter = std::find_if(std::cbegin(config.sta.wifis), std::cend(config.sta.wifis),
                                              [&connectedSSID](const wifi_entry &entry){
                                                  return cpputils::stringEqualsIgnoreCase(entry.ssid, connectedSSID);
                                              });

                if (iter == std::cend(config.sta.wifis))
                {
                    ESP_LOGI(TAG, "disconnecting, because cannot find ssid in config anymore");
                    lastWifisChecksum.clear();
                    if (const auto result = goe_wifi_sta_disconnect(config, false, true); result != ESP_OK)
                        ESP_LOGE(TAG, "goe_wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
                    ESP_LOGI(TAG, "status after disconnect: %s", toString(get_sta_status()).c_str());

                    setWifiState(WiFiState::None); // results in another scan
                }
                else
                {
                    if (last_sta_static_ip != iter->static_ip ||
                        last_sta_static_dns != iter->static_dns)
                    {
                        ESP_LOGI(TAG, "wifi static ip/dns config changed, applying new config");

                        if (const auto result = goe_wifi_set_esp_interface_ip(ESP_IF_WIFI_STA, iter->static_ip); result != ESP_OK)
                        {
                            ESP_LOGE(TAG, "goe_wifi_set_esp_interface_ip() for STA failed with %s", esp_err_to_name(result));
                            //return result;
                        }
                        else
                        {
                            last_sta_static_ip = iter->static_ip;

                            if (const auto result = goe_wifi_set_esp_interface_dns(ESP_IF_WIFI_STA, iter->static_dns); result != ESP_OK)
                            {
                                ESP_LOGE(TAG, "goe_wifi_set_esp_interface_dns() for STA failed with %s", esp_err_to_name(result));
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

        if (!cpputils::is_in(get_sta_status(), WiFiStaStatus::WL_IDLE_STATUS, WiFiStaStatus::WL_DISCONNECTED) ||
            !cpputils::is_in(_wifiState, WiFiState::None, WiFiState::Scanning))
        {
            ESP_LOGI(TAG, "disconnecting, because wifi_enabled is false");

            if (const auto result = goe_wifi_sta_disconnect(config, false, true); result != ESP_OK)
                ESP_LOGE(TAG, "goe_wifi_sta_disconnect() failed with %s", esp_err_to_name(result));
            lastWifisChecksum.clear();
            setWifiState(WiFiState::None);
        }
    }
}

WiFiStaStatus get_sta_status()
{
    return _sta_status.load();
}

esp_err_t begin_scan(const config &config)
{
    if (const auto result = goe_wifi_begin_scan(config); result != ESP_OK)
    {
        ESP_LOGE(TAG, "goe_wifi_begin_scan() failed with %s", esp_err_to_name(result));
        return result;
    }

    lastScanStarted = espchrono::millis_clock::now();
    setWifiState(WiFiState::Scanning);
    wasReallyScanning = true;

    return ESP_OK;
}

WiFiScanStatus get_scan_status()
{
    //Check is scan was started and if the delay expired, return WIFI_SCAN_FAILED in this case
    if (scanStarted && espchrono::ago(*scanStarted) > scanTimeout)
    {
        goe_wifi_clear_status_bits(WIFI_SCANNING_BIT);
        return WiFiScanStatus::Failed;
    }

    if (goe_wifi_get_status_bits() & WIFI_SCAN_DONE_BIT)
        return WiFiScanStatus::Finished;

    if (goe_wifi_get_status_bits() & WIFI_SCANNING_BIT)
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

tl::expected<wifi_stack::mac_t, std::string> get_default_mac_addr()
{
    wifi_stack::mac_t mac{};
    if (const auto result = esp_efuse_mac_get_default(std::begin(mac)); result == ESP_OK)
        return mac;
    else
    {
        ESP_LOGE(TAG, "esp_efuse_mac_get_default() failed with %s", esp_err_to_name(result));
        return tl::make_unexpected(fmt::format("esp_efuse_mac_get_default() failed with {}", esp_err_to_name(result)));
    }
}

tl::expected<wifi_stack::mac_t, std::string> get_base_mac_addr()
{
    wifi_stack::mac_t mac{};
    if (const auto result = esp_base_mac_addr_get(std::begin(mac)); result == ESP_OK)
        return mac;
    else
    {
        ESP_LOGE(TAG, "esp_base_mac_addr_get() failed with %s", esp_err_to_name(result));
        return tl::make_unexpected(fmt::format("esp_base_mac_addr_get() failed with {}", esp_err_to_name(result)));
    }
}

tl::expected<void, std::string> set_base_mac_addr(wifi_stack::mac_t mac_addr)
{
    if (const auto result = esp_base_mac_addr_set(std::cbegin(mac_addr)); result == ESP_OK)
        return {};
    else
    {
        ESP_LOGE(TAG, "esp_base_mac_addr_set() failed with %s", esp_err_to_name(result));
        return tl::make_unexpected(fmt::format("esp_base_mac_addr_set() failed with {}", esp_err_to_name(result)));
    }
}

tl::expected<tcpip_adapter_ip_info_t, std::string> get_ip_info(tcpip_adapter_if_t tcpip_if)
{
    tcpip_adapter_ip_info_t ip;
    if (const auto result = tcpip_adapter_get_ip_info(tcpip_if, &ip); result == ESP_OK)
        return ip;
    else
    {
        ESP_LOGE(TAG, "tcpip_adapter_get_ip_info() failed with %s", esp_err_to_name(result));
        return tl::make_unexpected(fmt::format("tcpip_adapter_get_ip_info() failed with {}", esp_err_to_name(result)));
    }
}

} // namespace wifi_stack
