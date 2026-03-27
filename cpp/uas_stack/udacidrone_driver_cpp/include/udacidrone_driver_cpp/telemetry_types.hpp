#pragma once

#include <cstdint>

namespace udacidrone_driver_cpp {

struct UasStateTelemetry {
    double stamp_sec{0.0};
    bool armed{false};
    bool guided{false};
    int32_t status{0};
};

struct UasPositionTelemetry {
    double stamp_sec{0.0};
    double north{0.0};
    double east{0.0};
    double down{0.0};
    double longitude{0.0};
    double latitude{0.0};
    double altitude{0.0};
};

struct UasVelocityTelemetry {
    double stamp_sec{0.0};
    double vn{0.0};
    double ve{0.0};
    double vd{0.0};
};

struct UasStateSnapshot {
    UasStateTelemetry state;
    UasPositionTelemetry position;
    UasVelocityTelemetry velocity;
    bool connected{false};
};

} // namespace udacidrone_driver_cpp
