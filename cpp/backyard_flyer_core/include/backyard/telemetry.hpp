// Purpose: standardize the telemetry sample the controller consumes

#pragma once

#include <cstdint>

#include "backyard/types.hpp"

// NOTE: keep it POD-like, timestamp can be "best available";
// monotonic microseconds is fine

namespace backyard {

struct TelemetrySample {
    NED position_ned;
    NED velocity_ned;
    bool armed{false};
    bool guided{false};
    uint64_t t_us{0};
};

} // namespace backyard
