#include "espwifistackenums.h"

namespace wifi_stack {

IMPLEMENT_TYPESAFE_ENUM(WiFiState, : uint8_t, WiFiStateValues)
IMPLEMENT_TYPESAFE_ENUM(WiFiStaStatus, : uint8_t, WiFiStaStatusValues)
IMPLEMENT_TYPESAFE_ENUM(WiFiScanStatus, : uint8_t, WiFiScanStatusValues)

} // namespace wifi_stack
