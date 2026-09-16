#pragma once

#include "esp_err.h"

namespace ray2::httpd {

// Starts the HTTP transport adapter after Wi-Fi is available. Commands remain
// owned by protocol/service; the handler never calls the motion path directly.
esp_err_t httpd_service_start();
esp_err_t httpd_service_stop();
bool httpd_service_is_started();

}  // namespace ray2::httpd
