#pragma once

// Class: Uas State Repository
//
// Responsibility:
//
// 1. Hold latest normalized state
// 2. Apply frame/sign conversion rules (NED, PX4/simulator differences)
// 3. Provide read-only snapshots to publishers and guards

#include <mutex>

#include "udacidrone_driver_cpp/telemetry_types.hpp"

namespace udacidrone_driver_cpp {

class UasStateRepository {
public:
    void setConnected(bool connected);
    void updateState(const UasStateTelemetry &msg);
    void updatePosition(const UasPositionTelemetry &msg);
    void updateVelocity(const UasVelocityTelemetry &msg);
    UasStateSnapshot snapshot() const;

private:
    mutable std::mutex mutex_;
    UasStateSnapshot snapshot_{};
};

} // namespace udacidrone_driver_cpp
