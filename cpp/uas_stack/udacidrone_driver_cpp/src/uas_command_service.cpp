#include "udacidrone_driver_cpp/uas_command_service.hpp"

namespace udacidrone_driver_cpp {

UasCommandService::UasCommandService(
    SimulatorConnection &connection,
    MavlinkTranslator &translator,
    bool is_px4
) : connection_(connection), translator_(translator), is_px4_(is_px4) {

}

bool UasCommandService::arm() {
    return connection_.sendMessage(translator_.encodeArm(true));
}

bool UasCommandService::disarm() {
    return connection_.sendMessage(translator_.encodeArm(false));
}

bool UasCommandService::takeControl() {
    return connection_.sendMessage(translator_.encodeTakeControl());
}

bool UasCommandService::releaseControl() {
    return connection_.sendMessage(translator_.encodeReleaseControl());
}

bool UasCommandService::takeoff(float target_altitude_m) {
    return connection_.sendMessage(translator_.encodeTakeoff(target_altitude_m));
}

bool UasCommandService::land() {
    return connection_.sendMessage(translator_.encodeLand());
}

bool UasCommandService::cmdPosition(float n, float e, float d, float heading_rad) {
    // Keep this behavior aligned with UdaciDrone: simulator may need sign inversion on d.
    if (!is_px4_) {
        d = -d;
    }

    return connection_.sendMessage(translator_.encodeCmdPosition(n, e, d, heading_rad));
}

} // namespace udacidrone_driver_cpp
