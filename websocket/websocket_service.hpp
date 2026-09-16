#pragma once

#include <cstddef>

#include "esp_err.h"

namespace ray2::websocket {

// Starts the dedicated WebSocket listener on TCP port 8849. The listener
// accepts at most three clients and exposes no command-ingress path.
esp_err_t websocket_service_start();
esp_err_t websocket_service_stop();
bool websocket_service_is_started();

// Broadcasts one protocol text payload to all currently connected clients.
// The payload is copied before this function returns; a saturated network
// work queue drops the newest notification rather than delaying motion work.
esp_err_t websocket_service_publish_text(const char* text, std::size_t length);

}  // namespace ray2::websocket
