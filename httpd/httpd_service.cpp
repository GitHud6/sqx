#include "httpd_service.hpp"

#include "esp_http_server.h"
#include "esp_log.h"

#include "wifi_service.hpp"

namespace ray2::httpd {
namespace {

constexpr char kLogTag[] = "httpd";

httpd_handle_t s_server = nullptr;

esp_err_t send_text(httpd_req_t* const request, const char* const body,
                    const char* const content_type) {
    if (request == nullptr || body == nullptr || content_type == nullptr) return ESP_ERR_INVALID_ARG;
    esp_err_t result = httpd_resp_set_type(request, content_type);
    if (result != ESP_OK) return result;
    result = httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (result != ESP_OK) return result;
    return httpd_resp_sendstr(request, body);
}

esp_err_t root_get_handler(httpd_req_t* const request) {
    return send_text(request, "Ray2-IDF HTTP service\n", "text/plain");
}

esp_err_t health_get_handler(httpd_req_t* const request) {
    return send_text(request, "{\"service\":\"ray2-idf\",\"status\":\"ok\"}\n",
                     "application/json");
}

const httpd_uri_t kRootUri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = nullptr,
};

const httpd_uri_t kHealthUri = {
    .uri = "/health",
    .method = HTTP_GET,
    .handler = health_get_handler,
    .user_ctx = nullptr,
};

}  // namespace

esp_err_t httpd_service_start() {
    if (s_server != nullptr) return ESP_OK;
    if (!wifi::wifi_is_started()) return ESP_ERR_INVALID_STATE;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 4096;
    config.task_priority = 4;

    httpd_handle_t server = nullptr;
    esp_err_t result = ::httpd_start(&server, &config);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "start failed: %s", esp_err_to_name(result));
        return result;
    }
    result = httpd_register_uri_handler(server, &kRootUri);
    if (result == ESP_OK) result = httpd_register_uri_handler(server, &kHealthUri);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "route registration failed: %s", esp_err_to_name(result));
        (void)::httpd_stop(server);
        return result;
    }

    s_server = server;
    ESP_LOGI(kLogTag, "started port=%u routes=/,/health",
             static_cast<unsigned>(config.server_port));
    return ESP_OK;
}

esp_err_t httpd_service_stop() {
    if (s_server == nullptr) return ESP_OK;
    const esp_err_t result = ::httpd_stop(s_server);
    if (result == ESP_OK) {
        s_server = nullptr;
        ESP_LOGI(kLogTag, "stopped");
    }
    return result;
}

bool httpd_service_is_started() {
    return s_server != nullptr;
}

}  // namespace ray2::httpd
