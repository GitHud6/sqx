#pragma once

#include "esp_err.h"

namespace laserctl::network {

// Owns HTTP and WebSocket transport lifetime. Wi-Fi callbacks only notify this
// task; all socket stop/start work runs outside the ESP event loop.
esp_err_t network_lifecycle_start();
bool network_lifecycle_is_started();

// Restarts Wi-Fi and rebuilds its public transports through the same serialized
// path used for a station reconnect.
esp_err_t network_lifecycle_request_wifi_restart();

}  // namespace laserctl::network
