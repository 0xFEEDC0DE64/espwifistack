#pragma once

// 3rdparty lib includes
#include <cpptypesafeenum.h>

namespace wifi_stack {

#define WiFiStateValues(x) \
    x(None) \
    x(Scanning) \
    x(Connecting) \
    x(Connected)
DECLARE_TYPESAFE_ENUM(WiFiState, : uint8_t, WiFiStateValues)

#define WiFiStaStatusValues(x) \
    x(IDLE_STATUS) \
    x(NO_SSID_AVAIL) \
    x(SCAN_COMPLETED) \
    x(CONNECTED) \
    x(CONNECT_FAILED) \
    x(CONNECTION_LOST) \
    x(DISCONNECTED) \
    x(CONNECTING) \
    x(DISCONNECTING) \
    x(NO_SHIELD)
DECLARE_TYPESAFE_ENUM(WiFiStaStatus, : uint8_t, WiFiStaStatusValues)

#define WiFiScanStatusValues(x) \
    x(None) \
    x(Scanning) \
    x(Finished) \
    x(Failed)
DECLARE_TYPESAFE_ENUM(WiFiScanStatus, : uint8_t, WiFiScanStatusValues)

} // namespace wifi_stack
