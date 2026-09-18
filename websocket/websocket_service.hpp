#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace ray2::websocket {

// Starts the dedicated WebSocket listener on TCP port 8849. The listener
// keeps one active portal client and routes command ingress through network.
esp_err_t websocket_service_start();
esp_err_t websocket_service_stop();
bool websocket_service_is_started();

// Enqueues one text payload for all connected clients. The call never waits
// for socket I/O.
esp_err_t websocket_service_publish_text(const char* text, std::size_t length);

// Enqueues one binary protocol response for all connected clients.
esp_err_t websocket_service_publish_binary(const std::uint8_t* data, std::size_t length);

// Replaces an unsent realtime status with the newest status snapshot.
bool websocket_service_publish_realtime_status(const std::uint8_t* data, std::size_t length);

}  // namespace ray2::websocket
