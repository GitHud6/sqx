#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace ray2::protocol {

constexpr std::size_t kProtocolTxMessageCapacity = 1600;

struct ProtocolTxDiagnostics {
    std::size_t normal_pending = 0;
    std::uint64_t normal_enqueued = 0;
    std::uint64_t normal_sent = 0;
    std::uint64_t normal_write_failed = 0;
    std::uint64_t status_published = 0;
    std::uint64_t status_sent = 0;
    std::uint64_t status_write_failed = 0;
    std::uint64_t status_coalesced = 0;
};

esp_err_t protocol_tx_start();

// Queues a complete normal response in FIFO order. The caller may wait for a
// slot, but does not write UART hardware directly.
bool protocol_tx_enqueue_normal(const std::uint8_t* data, std::size_t length);

// Replaces any unsent realtime status packet with the newest snapshot.
bool protocol_tx_publish_realtime_status(const std::uint8_t* data, std::size_t length);
// Drops unsent ordinary replies and status packets after Abort.
void protocol_tx_abort_flush();
ProtocolTxDiagnostics protocol_tx_diagnostics();

}  // namespace ray2::protocol
