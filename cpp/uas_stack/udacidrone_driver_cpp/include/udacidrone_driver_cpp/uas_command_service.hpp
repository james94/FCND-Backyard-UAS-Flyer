#pragma once

// Class: Uas Command Service

// Responsibility:
//
// 1. Execute high-level operations using connection + translator
// 2. Ensure ordered command flow (control mode, arm, takeoff, etc.)
// 3. Enforce command throttling and retries where needed

#include "udacidrone_driver_cpp/mavlink_translator.hpp"
#include "udacidrone_driver_cpp/simulator_connection.hpp"

namespace udacidrone_driver_cpp {

class UasCommandService {
public:
    UasCommandService(SimulatorConnection &connection, MavlinkTranslator &translator, bool is_px4);
    
    bool arm();
    bool disarm();
    bool takeControl();
    bool releaseControl();
    bool takeoff(float target_altitude_m);
    bool land();
    bool cmdPosition(float n, float e, float d, float heading_rad);

private:
    SimulatorConnection &connection_;
    MavlinkTranslator &translator_;
    bool is_px4_{false};
};

} // namespace udacidrone_driver_cpp
