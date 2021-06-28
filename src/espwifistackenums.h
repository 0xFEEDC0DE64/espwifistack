#pragma once

// local includes
#include "cpptypesafeenum.h"

namespace wifi_stack {

#define WiFiStateValues(x) \
    x(None) \
    x(Scanning) \
    x(Connecting) \
    x(Connected)
DECLARE_TYPESAFE_ENUM(WiFiState, : uint8_t, WiFiStateValues)

#define WiFiStaStatusValues(x) \
    x(WL_IDLE_STATUS) \
    x(WL_NO_SSID_AVAIL) \
    x(WL_SCAN_COMPLETED) \
    x(WL_CONNECTED) \
    x(WL_CONNECT_FAILED) \
    x(WL_CONNECTION_LOST) \
    x(WL_DISCONNECTED) \
    x(WL_CONNECTING) \
    x(WL_DISCONNECTING) \
    x(WL_NO_SHIELD)
DECLARE_TYPESAFE_ENUM(WiFiStaStatus, : uint8_t, WiFiStaStatusValues)

#define WiFiScanStatusValues(x) \
    x(None) \
    x(Scanning) \
    x(Finished) \
    x(Failed)
DECLARE_TYPESAFE_ENUM(WiFiScanStatus, : uint8_t, WiFiScanStatusValues)

} // namespace wifi_stack
