#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace ray2::protocol {

constexpr std::size_t kProtocolTxMessageCapacity = 1600;

esp_err_t protocol_tx_start();

// Queues a complete normal response in FIFO order. The caller may wait for a
// slot, but does not write UART hardware directly.
bool protocol_tx_enqueue_normal(const std::uint8_t* data, std::size_t length);

// Replaces any unsent realtime status packet with the newest snapshot.
bool protocol_tx_publish_realtime_status(const std::uint8_t* data, std::size_t length);
// Drops unsent ordinary replies and status packets after Abort.
void protocol_tx_abort_flush();

}  // namespace ray2::protocol
