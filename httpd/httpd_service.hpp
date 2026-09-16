#pragma once

#include "esp_err.h"

namespace ray2::httpd {

// Starts the minimal HTTP service after the Wi-Fi transport is available.
// The service intentionally exposes no motion or configuration mutation APIs.
esp_err_t httpd_service_start();
esp_err_t httpd_service_stop();
bool httpd_service_is_started();

}  // namespace ray2::httpd
