#pragma once

#include "freertos/FreeRTOS.h"

namespace laserctl::runtime {

// Timer Group 0 is initialized from app_main on CPU0, and its Level-3 ISR is
// allocated on that same core. All blocking, parsing and planning tasks must
// stay on CPU1 so their critical sections cannot mask the step timer ISR.
constexpr BaseType_t kRealtimeCore = 0;
constexpr BaseType_t kNonRealtimeCore = 1;

}  // namespace laserctl::runtime
