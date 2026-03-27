#pragma once

// Class: Command Gatekeeeper
//
// Responsibilities:
//
// 1. Decide if a command is allowed based on run-state, safety-state, and timeout-state.
// 2. Keep a list of always-allowed commands such as disarm and emergency-level.
// 3. Centralize command gating policy away from ROS callback glue.

#include <string>

#include "udacidrone_driver_cpp/telemetry_types.hpp"

namespace udacidrone_driver_cpp {

enum class UasCommandType {
    ARM,
    DISARM,
    TAKE_CONTROL,
    RELEASE_CONTROL,
    TAKEOFF,
    LAND,
    CMD_POSITION
};

class CommandGatekeeper {
public:
    bool allow(UasCommandType type, const UasStateSnapshot &snapshot, bool program_running) const;
    std::string denyReason(UasCommandType, const UasStateSnapshot &snapshot, bool program_running) const;
};

} // namespace udacidrone_driver_cpp
