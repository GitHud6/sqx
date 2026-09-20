#include "websocket_service.hpp"

#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "mbedtls/base64.h"
#include "mbedtls/sha1.h"

#include "command_dispatch.hpp"
#include "network_command_adapter.hpp"
#include "runtime_affinity.hpp"
#include "wifi_service.hpp"

namespace laserctl::websocket {
namespace {

constexpr char kLogTag[] = "websocket";
constexpr std::uint16_t kServerPort = 8849;
constexpr std::size_t kMaximumInboundFrameBytes = 1024;
constexpr std::size_t kMaximumOutboundMessageBytes = 1600;
constexpr std::size_t kOutboundQueueLength = 32;
constexpr std::size_t kStatusMessageCapacity = 256;
constexpr std::size_t kHandshakeCapacity = 2048;
constexpr std::size_t kReceiveCapacity = kMaximumInboundFrameBytes + 32;
constexpr std::uint32_t kAnySession = 0;

enum : std::uint8_t {
    kText = 0x1,
    kBinary = 0x2,
    kClose = 0x8,
    kPing = 0x9,
    kPong = 0xA,
};

struct OutboundMessage {
    std::uint32_t target_session = kAnySession;
    std::uint8_t opcode = kBinary;
    std::size_t length = 0;
    std::uint8_t data[kMaximumOutboundMessageBytes] = {};
};

struct PendingStatus {
    std::size_t length = 0;
    std::uint8_t data[kStatusMessageCapacity] = {};
    bool pending = false;
};

struct TransmitState {
    OutboundMessage* message = nullptr;
    std::uint8_t header[10] = {};
    std::size_t header_length = 0;
    std::size_t header_sent = 0;
    std::size_t payload_sent = 0;
};

// These buffers belong exclusively to the WebSocket task. Keeping them in
// static storage avoids consuming the task stack during the HTTP upgrade and
// the first build-info response (which formats floating-point values).
struct SessionBuffers {
    std::array<char, kHandshakeCapacity> handshake = {};
    std::array<char, 256> handshake_response = {};
    std::array<std::uint8_t, kReceiveCapacity> receive = {};
    std::array<char, kMaximumInboundFrameBytes + 1U> command = {};
};

QueueHandle_t s_free_messages = nullptr;
QueueHandle_t s_ready_messages = nullptr;
TaskHandle_t s_task = nullptr;
std::atomic<bool> s_started = false;
std::atomic<bool> s_running = false;
std::atomic<bool> s_client_ready = false;
int s_listener_fd = -1;
portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
PendingStatus s_pending_status = {};
OutboundMessage s_messages[kOutboundQueueLength] = {};
SessionBuffers s_session_buffers = {};

bool set_nonblocking(const int fd) {
    const int flags = fcntl(fd, F_GETFL, 0);
    return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

int create_listener() {
    const int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (fd < 0) return -1;

    int reuse = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(kServerPort);
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 ||
        listen(fd, 1) != 0 || !set_nonblocking(fd)) {
        close(fd);
        return -1;
    }
    return fd;
}

bool equals_ignore_case(const char* const text, const std::size_t length,
                        const char* const expected) {
    const std::size_t expected_length = std::strlen(expected);
    if (length != expected_length) return false;
    for (std::size_t index = 0; index < length; ++index) {
        if (std::toupper(static_cast<unsigned char>(text[index])) !=
            std::toupper(static_cast<unsigned char>(expected[index]))) return false;
    }
    return true;
}

const char* find_header(const char* const text, const std::size_t length,
                        const char* const name) {
    const std::size_t name_length = std::strlen(name);
    if (name_length == 0 || length < name_length) return nullptr;
    for (std::size_t index = 0; index + name_length <= length; ++index) {
        if (std::memcmp(text + index, name, name_length) == 0) return text + index;
        bool match = true;
        for (std::size_t offset = 0; offset < name_length; ++offset) {
            if (std::tolower(static_cast<unsigned char>(text[index + offset])) !=
                std::tolower(static_cast<unsigned char>(name[offset]))) {
                match = false;
                break;
            }
        }
        if (match) return text + index;
    }
    return nullptr;
}

bool header_value(const char* const request, const std::size_t length, const char* const name,
                  char* const output, const std::size_t capacity) {
    const char* const found = find_header(request, length, name);
    if (found == nullptr) return false;
    const char* cursor = found + std::strlen(name);
    const char* const end = request + length;
    while (cursor < end && (*cursor == ' ' || *cursor == '\t')) ++cursor;
    if (cursor >= end || *cursor != ':') return false;
    ++cursor;
    while (cursor < end && (*cursor == ' ' || *cursor == '\t')) ++cursor;
    const char* value_end = cursor;
    while (value_end < end && *value_end != '\r' && *value_end != '\n') ++value_end;
    while (value_end > cursor && (value_end[-1] == ' ' || value_end[-1] == '\t')) --value_end;
    const std::size_t value_length = static_cast<std::size_t>(value_end - cursor);
    if (value_length == 0 || value_length + 1 > capacity) return false;
    std::memcpy(output, cursor, value_length);
    output[value_length] = '\0';
    return true;
}

bool build_handshake_response(const char* const request, const std::size_t length,
                              char* const response, const std::size_t capacity,
                              std::size_t& response_length) {
    if (find_header(request, length, "Upgrade") == nullptr ||
        find_header(request, length, "Sec-WebSocket-Key") == nullptr) return false;

    char key[128] = {};
    if (!header_value(request, length, "Sec-WebSocket-Key", key, sizeof(key))) return false;
    char input[192] = {};
    const int input_length = std::snprintf(input, sizeof(input),
                                           "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key);
    if (input_length <= 0 || static_cast<std::size_t>(input_length) >= sizeof(input)) return false;

    unsigned char digest[20] = {};
    if (mbedtls_sha1(reinterpret_cast<const unsigned char*>(input),
                     static_cast<std::size_t>(input_length), digest) != 0) return false;
    unsigned char encoded[64] = {};
    std::size_t encoded_length = 0;
    if (mbedtls_base64_encode(encoded, sizeof(encoded), &encoded_length, digest, sizeof(digest)) != 0 ||
        encoded_length + 1 > sizeof(encoded)) return false;
    encoded[encoded_length] = '\0';

    const int written = std::snprintf(response, capacity,
                                      "HTTP/1.1 101 Switching Protocols\r\n"
                                      "Upgrade: websocket\r\n"
                                      "Connection: Upgrade\r\n"
                                      "Sec-WebSocket-Accept: %s\r\n\r\n", encoded);
    if (written <= 0 || static_cast<std::size_t>(written) >= capacity) return false;
    response_length = static_cast<std::size_t>(written);
    return true;
}

void recycle_ready_messages() {
    if (s_ready_messages == nullptr || s_free_messages == nullptr) return;
    OutboundMessage* message = nullptr;
    while (xQueueReceive(s_ready_messages, &message, 0) == pdTRUE) {
        (void)xQueueSend(s_free_messages, &message, 0);
    }
    portENTER_CRITICAL(&s_status_lock);
    s_pending_status.pending = false;
    s_pending_status.length = 0;
    portEXIT_CRITICAL(&s_status_lock);
}

bool queue_message(const std::uint32_t target_session, const std::uint8_t opcode,
                   const std::uint8_t* const data, const std::size_t length) {
    if (!s_client_ready.load(std::memory_order_acquire) || (data == nullptr && length != 0) ||
        length > kMaximumOutboundMessageBytes || s_free_messages == nullptr ||
        s_ready_messages == nullptr) return false;
    OutboundMessage* message = nullptr;
    if (xQueueReceive(s_free_messages, &message, 0) != pdTRUE) return false;
    message->target_session = target_session;
    message->opcode = opcode;
    message->length = length;
    if (length != 0) std::memcpy(message->data, data, length);
    if (xQueueSend(s_ready_messages, &message, 0) != pdTRUE) {
        (void)xQueueSend(s_free_messages, &message, 0);
        return false;
    }
    return true;
}

bool take_pending_status(OutboundMessage& message, const std::uint32_t session) {
    portENTER_CRITICAL(&s_status_lock);
    if (!s_pending_status.pending) {
        portEXIT_CRITICAL(&s_status_lock);
        return false;
    }
    message.target_session = session;
    message.opcode = kBinary;
    message.length = s_pending_status.length;
    std::memcpy(message.data, s_pending_status.data, message.length);
    s_pending_status.pending = false;
    s_pending_status.length = 0;
    portEXIT_CRITICAL(&s_status_lock);
    return true;
}

bool prepare_header(TransmitState& state) {
    if (state.message == nullptr || state.message->length > kMaximumOutboundMessageBytes) return false;
    const std::size_t length = state.message->length;
    state.header[0] = static_cast<std::uint8_t>(0x80U | state.message->opcode);
    if (length < 126) {
        state.header[1] = static_cast<std::uint8_t>(length);
        state.header_length = 2;
    } else {
        state.header[1] = 126;
        state.header[2] = static_cast<std::uint8_t>((length >> 8U) & 0xffU);
        state.header[3] = static_cast<std::uint8_t>(length & 0xffU);
        state.header_length = 4;
    }
    state.header_sent = 0;
    state.payload_sent = 0;
    return true;
}

void release_active(TransmitState& state) {
    if (state.message != nullptr && s_free_messages != nullptr) {
        (void)xQueueSend(s_free_messages, &state.message, 0);
    }
    state = {};
}

bool normalize_command(char* const text, std::size_t& length) {
    if (text == nullptr || length == 0) return false;
    std::size_t first = 0;
    while (first < length && std::isspace(static_cast<unsigned char>(text[first])) != 0) ++first;
    while (length > first && std::isspace(static_cast<unsigned char>(text[length - 1])) != 0) --length;
    if (first == length) return false;
    if (first != 0) std::memmove(text, text + first, length - first);
    length -= first;
    text[length] = '\0';
    return true;
}

void send_submit_error(const std::uint32_t session, const network::SubmitResult result) {
    const char* response = "error:network-input-unavailable\r\n";
    if (result == network::SubmitResult::Busy) response = "error:network-input-busy\r\n";
    if (result == network::SubmitResult::Invalid) response = "error:invalid-command\r\n";
    (void)queue_message(session, kBinary, reinterpret_cast<const std::uint8_t*>(response),
                        std::strlen(response));
}

void submit_to_protocol(const std::uint32_t session, const char* const text,
                        const std::size_t length, const bool command_text) {
    const network::SubmitResult result =
        network::network_command_adapter_submit(text, length, command_text);
    if (result != network::SubmitResult::Accepted) send_submit_error(session, result);
}

void process_inbound_command(const std::uint32_t session, char* const command, std::size_t length) {
    if (!normalize_command(command, length)) {
        static constexpr char kError[] = "error:invalid-command\r\n";
        (void)queue_message(session, kBinary, reinterpret_cast<const std::uint8_t*>(kError),
                            sizeof(kError) - 1U);
        return;
    }
    if (length == 1 && command[0] == '?') {
        submit_to_protocol(session, command, length, false);
        return;
    }
    if (equals_ignore_case(command, length, "$I")) {
        submit_to_protocol(session, command, length, false);
        return;
    }
    if (equals_ignore_case(command, length, "req")) {
        static constexpr char kStatus[] = "?";
        static constexpr char kBuildInfo[] = "$I";
        submit_to_protocol(session, kStatus, sizeof(kStatus) - 1U, false);
        submit_to_protocol(session, kBuildInfo, sizeof(kBuildInfo) - 1U, false);
        return;
    }
    if (equals_ignore_case(command, length, "$WiFi/ListAPs")) {
        static constexpr char kResponse[] = "{\"AP_LIST\":[]}ok\r\n";
        (void)queue_message(session, kBinary, reinterpret_cast<const std::uint8_t*>(kResponse),
                            sizeof(kResponse) - 1U);
        return;
    }
    if (length == std::strlen("{\"type\":\"ping\"}") &&
        std::memcmp(command, "{\"type\":\"ping\"}", length) == 0) {
        static constexpr char kPong[] = "{\"type\":\"pong\"}";
        (void)queue_message(session, kText, reinterpret_cast<const std::uint8_t*>(kPong),
                            sizeof(kPong) - 1U);
        return;
    }
    submit_to_protocol(session, command, length, true);
}

void send_client_bootstrap(const std::uint32_t session) {
    char status[256] = {};
    std::size_t status_length = 0;
    if (protocol::command_format_realtime_status(status, sizeof(status), status_length)) {
        (void)queue_message(session, kBinary, reinterpret_cast<const std::uint8_t*>(status),
                            status_length);
    }
    char build_info[512] = {};
    std::size_t build_info_length = 0;
    if (protocol::command_format_build_info(build_info, sizeof(build_info), build_info_length)) {
        (void)queue_message(session, kBinary, reinterpret_cast<const std::uint8_t*>(build_info),
                            build_info_length);
    }
}

bool parse_frame(std::array<std::uint8_t, kReceiveCapacity>& buffer, std::size_t& buffer_length,
                 const std::uint32_t session) {
    if (buffer_length < 2) return true;
    const std::uint8_t first = buffer[0];
    const std::uint8_t second = buffer[1];
    if ((first & 0x80U) == 0 || (second & 0x80U) == 0) return false;
    const std::uint8_t opcode = first & 0x0fU;
    std::size_t header_length = 2;
    std::uint64_t payload_length = second & 0x7fU;
    if (payload_length == 126) {
        if (buffer_length < 4) return true;
        payload_length = (static_cast<std::uint64_t>(buffer[2]) << 8U) | buffer[3];
        header_length = 4;
    } else if (payload_length == 127) {
        return false;
    }
    if (payload_length > kMaximumInboundFrameBytes) return false;
    const std::size_t frame_length = header_length + 4U + static_cast<std::size_t>(payload_length);
    if (buffer_length < frame_length) return true;
    const std::size_t mask_offset = header_length;
    const std::size_t payload_offset = header_length + 4U;
    const std::uint8_t mask[4] = {buffer[mask_offset], buffer[mask_offset + 1],
                                  buffer[mask_offset + 2], buffer[mask_offset + 3]};
    for (std::size_t index = 0; index < payload_length; ++index) {
        buffer[payload_offset + index] ^= mask[index % 4U];
    }
    auto* const payload = buffer.data() + payload_offset;
    switch (opcode) {
        case kText:
        case kBinary:
            if (std::memchr(payload, '\0', static_cast<std::size_t>(payload_length)) == nullptr) {
                const std::size_t command_length = static_cast<std::size_t>(payload_length);
                std::memcpy(s_session_buffers.command.data(), payload, command_length);
                s_session_buffers.command[command_length] = '\0';
                process_inbound_command(session, s_session_buffers.command.data(), command_length);
            }
            break;
        case kPing:
            (void)queue_message(session, kPong, payload, static_cast<std::size_t>(payload_length));
            break;
        case kPong:
            break;
        case kClose:
            return false;
        default:
            return false;
    }
    const std::size_t remaining = buffer_length - frame_length;
    if (remaining != 0) std::memmove(buffer.data(), buffer.data() + frame_length, remaining);
    buffer_length = remaining;
    return true;
}

void websocket_task(void*) {
    int listener_fd = s_listener_fd;
    s_listener_fd = -1;
    int client_fd = -1;
    std::uint32_t session = 0;
    std::uint32_t next_session = 0;
    bool ready = false;
    std::size_t handshake_response_length = 0;
    std::size_t handshake_response_sent = 0;
    std::size_t receive_length = 0;
    TransmitState transmit = {};

    const auto close_client = [&]() {
        if (client_fd >= 0) {
            ESP_LOGI(kLogTag, "session close port=%u fd=%d session=%u", kServerPort, client_fd,
                     static_cast<unsigned>(session));
            shutdown(client_fd, SHUT_RDWR);
            close(client_fd);
        }
        client_fd = -1;
        ready = false;
        s_client_ready.store(false, std::memory_order_release);
        receive_length = 0;
        s_session_buffers.handshake[0] = '\0';
        handshake_response_length = 0;
        handshake_response_sent = 0;
        release_active(transmit);
        recycle_ready_messages();
    };

    while (s_running.load(std::memory_order_acquire)) {
        if (client_fd >= 0 && ready && transmit.message == nullptr) {
            OutboundMessage* message = nullptr;
            while (xQueueReceive(s_ready_messages, &message, 0) == pdTRUE) {
                if (message->target_session != kAnySession && message->target_session != session) {
                    (void)xQueueSend(s_free_messages, &message, 0);
                    continue;
                }
                transmit.message = message;
                if (!prepare_header(transmit)) release_active(transmit);
                break;
            }
            if (transmit.message == nullptr && xQueueReceive(s_free_messages, &message, 0) == pdTRUE) {
                if (!take_pending_status(*message, session)) {
                    (void)xQueueSend(s_free_messages, &message, 0);
                } else {
                    transmit.message = message;
                    (void)prepare_header(transmit);
                }
            }
        }

        fd_set read_set;
        fd_set write_set;
        FD_ZERO(&read_set);
        FD_ZERO(&write_set);
        int maximum_fd = -1;
        if (listener_fd >= 0) {
            FD_SET(listener_fd, &read_set);
            maximum_fd = listener_fd;
        }
        if (client_fd >= 0) {
            FD_SET(client_fd, &read_set);
            if (handshake_response_sent < handshake_response_length || transmit.message != nullptr) {
                FD_SET(client_fd, &write_set);
            }
            if (client_fd > maximum_fd) maximum_fd = client_fd;
        }
        timeval timeout = {};
        timeout.tv_usec = 10000;
        const int selected = select(maximum_fd + 1, &read_set, &write_set, nullptr, &timeout);
        if (selected < 0) {
            if (errno != EINTR) ESP_LOGW(kLogTag, "select failed errno=%d", errno);
            continue;
        }

        if (listener_fd >= 0 && FD_ISSET(listener_fd, &read_set)) {
            close_client();
            sockaddr_storage address = {};
            socklen_t address_length = sizeof(address);
            const int accepted = accept(listener_fd, reinterpret_cast<sockaddr*>(&address),
                                        &address_length);
            if (accepted >= 0 && set_nonblocking(accepted)) {
                client_fd = accepted;
                session = ++next_session;
                ESP_LOGI(kLogTag, "session open port=%u fd=%d session=%u", kServerPort, client_fd,
                         static_cast<unsigned>(session));
            } else if (accepted >= 0) {
                close(accepted);
            }
        }

        if (client_fd < 0) continue;
        if (FD_ISSET(client_fd, &write_set)) {
            if (handshake_response_sent < handshake_response_length) {
                const ssize_t sent = send(client_fd,
                                          s_session_buffers.handshake_response.data() +
                                              handshake_response_sent,
                                          handshake_response_length - handshake_response_sent, 0);
                if (sent > 0) handshake_response_sent += static_cast<std::size_t>(sent);
                else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) close_client();
                if (client_fd >= 0 && handshake_response_sent == handshake_response_length) {
                    ready = true;
                    s_client_ready.store(true, std::memory_order_release);
                    send_client_bootstrap(session);
                    ESP_LOGI(kLogTag, "connected fd=%d session=%u", client_fd,
                             static_cast<unsigned>(session));
                }
            }
            if (client_fd >= 0 && ready && transmit.message != nullptr) {
                while (transmit.header_sent < transmit.header_length) {
                    const ssize_t sent = send(client_fd, transmit.header + transmit.header_sent,
                                              transmit.header_length - transmit.header_sent, 0);
                    if (sent > 0) transmit.header_sent += static_cast<std::size_t>(sent);
                    else if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                    else { close_client(); break; }
                }
                while (client_fd >= 0 && transmit.header_sent == transmit.header_length &&
                       transmit.payload_sent < transmit.message->length) {
                    const ssize_t sent = send(client_fd,
                                              transmit.message->data + transmit.payload_sent,
                                              transmit.message->length - transmit.payload_sent, 0);
                    if (sent > 0) transmit.payload_sent += static_cast<std::size_t>(sent);
                    else if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                    else { close_client(); break; }
                }
                if (client_fd >= 0 && transmit.message != nullptr &&
                    transmit.payload_sent == transmit.message->length) release_active(transmit);
            }
        }

        if (client_fd >= 0 && FD_ISSET(client_fd, &read_set)) {
            const ssize_t received = recv(client_fd, s_session_buffers.receive.data() + receive_length,
                                          s_session_buffers.receive.size() - receive_length, 0);
            if (received == 0 || (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                close_client();
            } else if (received > 0) {
                receive_length += static_cast<std::size_t>(received);
                if (!ready) {
                    if (receive_length >= 4 &&
                        std::strstr(s_session_buffers.handshake.data(), "\r\n\r\n") == nullptr) {
                        const std::size_t copy_length =
                            receive_length < s_session_buffers.handshake.size() - 1U
                                ? receive_length : s_session_buffers.handshake.size() - 1U;
                        std::memcpy(s_session_buffers.handshake.data(),
                                    s_session_buffers.receive.data(), copy_length);
                        s_session_buffers.handshake[copy_length] = '\0';
                    }
                    const char* const terminator =
                        std::strstr(s_session_buffers.handshake.data(), "\r\n\r\n");
                    if (terminator != nullptr) {
                        const std::size_t request_length =
                            static_cast<std::size_t>(terminator - s_session_buffers.handshake.data()) +
                            4U;
                        if (request_length > s_session_buffers.handshake.size() - 1U ||
                            !build_handshake_response(s_session_buffers.handshake.data(), request_length,
                                                      s_session_buffers.handshake_response.data(),
                                                      s_session_buffers.handshake_response.size(),
                                                      handshake_response_length)) {
                            close_client();
                        } else {
                            handshake_response_sent = 0;
                            const std::size_t trailing = receive_length - request_length;
                            if (trailing != 0) {
                                std::memmove(s_session_buffers.receive.data(),
                                             s_session_buffers.receive.data() + request_length, trailing);
                            }
                            receive_length = trailing;
                        }
                    } else if (receive_length >= s_session_buffers.receive.size()) {
                        close_client();
                    }
                } else {
                    bool valid = true;
                    while (receive_length != 0) {
                        const std::size_t previous_length = receive_length;
                        if (!parse_frame(s_session_buffers.receive, receive_length, session)) {
                            valid = false;
                            break;
                        }
                        if (receive_length == previous_length) break;
                    }
                    if (!valid || receive_length > s_session_buffers.receive.size()) {
                        close_client();
                    }
                }
            }
        }
    }

    close_client();
    if (listener_fd >= 0) close(listener_fd);
    s_started.store(false, std::memory_order_release);
    s_task = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace

esp_err_t websocket_service_start() {
    if (s_started.load(std::memory_order_acquire)) return ESP_OK;
    if (s_task != nullptr || s_running.load(std::memory_order_acquire)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!wifi::wifi_is_started()) return ESP_ERR_INVALID_STATE;
    if (s_free_messages == nullptr || s_ready_messages == nullptr) {
        s_free_messages = xQueueCreate(kOutboundQueueLength, sizeof(OutboundMessage*));
        s_ready_messages = xQueueCreate(kOutboundQueueLength, sizeof(OutboundMessage*));
        if (s_free_messages == nullptr || s_ready_messages == nullptr) {
            if (s_ready_messages != nullptr) vQueueDelete(s_ready_messages);
            if (s_free_messages != nullptr) vQueueDelete(s_free_messages);
            s_ready_messages = nullptr;
            s_free_messages = nullptr;
            return ESP_ERR_NO_MEM;
        }
        for (std::size_t index = 0; index < kOutboundQueueLength; ++index) {
            OutboundMessage* message = &s_messages[index];
            (void)xQueueSend(s_free_messages, &message, 0);
        }
    }
    const int listener = create_listener();
    if (listener < 0) {
        ESP_LOGE(kLogTag, "listen port=%u failed errno=%d", kServerPort, errno);
        return ESP_FAIL;
    }
    s_listener_fd = listener;
    s_running.store(true, std::memory_order_release);
    if (xTaskCreatePinnedToCore(websocket_task, "websocket", 8192, nullptr, 4, &s_task,
                                runtime::kNonRealtimeCore) != pdPASS) {
        s_running.store(false, std::memory_order_release);
        close(listener);
        s_listener_fd = -1;
        return ESP_ERR_NO_MEM;
    }
    s_started.store(true, std::memory_order_release);
    ESP_LOGI(kLogTag, "started port=%u backend=socket", kServerPort);
    return ESP_OK;
}

esp_err_t websocket_service_stop() {
    if (!s_started.load(std::memory_order_acquire) && s_task == nullptr) return ESP_OK;
    s_running.store(false, std::memory_order_release);
    if (s_task != nullptr) xTaskNotifyGive(s_task);
    return ESP_OK;
}

bool websocket_service_is_started() {
    return s_started.load(std::memory_order_acquire);
}

esp_err_t websocket_service_publish_text(const char* const text, const std::size_t length) {
    return queue_message(kAnySession, kText, reinterpret_cast<const std::uint8_t*>(text), length)
               ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t websocket_service_publish_binary(const std::uint8_t* const data, const std::size_t length) {
    return queue_message(kAnySession, kBinary, data, length) ? ESP_OK : ESP_ERR_NO_MEM;
}

bool websocket_service_publish_realtime_status(const std::uint8_t* const data,
                                                const std::size_t length) {
    if (!s_client_ready.load(std::memory_order_acquire) || data == nullptr || length == 0 ||
        length > kStatusMessageCapacity) return false;
    portENTER_CRITICAL(&s_status_lock);
    std::memcpy(s_pending_status.data, data, length);
    s_pending_status.length = length;
    s_pending_status.pending = true;
    portEXIT_CRITICAL(&s_status_lock);
    return true;
}

}  // namespace laserctl::websocket
