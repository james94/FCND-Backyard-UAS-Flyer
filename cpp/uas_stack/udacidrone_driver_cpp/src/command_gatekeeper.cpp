#include "udacidrone_driver_cpp/command_gatekeeper.hpp"

namespace udacidrone_driver_cpp {

bool CommandGatekeeper::allow(
    UasCommandType type,
    const UasStateSnapshot &snapshot,
    bool program_running
) const {
    // Safety-style exceptions inspired by UR driver behavior:
    // disarm and land remain allowed even if run-state is degraded
    if (type == UasCommandType::DISARM || type == UasCommandType::LAND) {
        return true;
    }

    if (!snapshot.connected) {
        return false;
    }

    if ((type == UasCommandType::TAKEOFF || type == UasCommandType::CMD_POSITION) && !snapshot.state.armed) {
        return false;
    }

    return program_running;
}

std::string CommandGatekeeper::denyReason(
    UasCommandType type,
    const UasStateSnapshot &snapshot,
    bool program_running
) const {
    if (type == UasCommandType::DISARM || type == UasCommandType::LAND) {
        return "";
    }

    if (!snapshot.connected) {
        return "driver not connected";
    }

    if ((type == UasCommandType::TAKEOFF || type == UasCommandType::CMD_POSITION) && !snapshot.state.armed) {
        return "vehicle must be armed first";
    }

    if (!program_running) {
        return "external control program not running";
    }

    return "";
}

} // namespace udacidrone_driver_cpp
