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
    x(IDLE_STATUS, = 0) \
    x(NO_SSID_AVAIL, = 1) \
    x(SCAN_COMPLETED, = 2) \
    x(CONNECTED, = 3) \
    x(CONNECT_FAILED, = 4) \
    x(CONNECTION_LOST, = 5) \
    x(DISCONNECTED, = 6) \
    x(CONNECTING, = 7) \
    x(DISCONNECTING, = 8) \
    x(NO_SHIELD, = 9) \
    x(WAITING_FOR_IP, = 10)
DECLARE_TYPESAFE_ENUM(WiFiStaStatus, : uint8_t, WiFiStaStatusValues)

#define WiFiScanStatusValues(x) \
    x(None) \
    x(Scanning) \
    x(Finished) \
    x(Failed)
DECLARE_TYPESAFE_ENUM(WiFiScanStatus, : uint8_t, WiFiScanStatusValues)

} // namespace wifi_stack
