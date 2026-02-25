#include "backyard/controller.hpp"
#include "backyard/guards.hpp"

namespace backyard {

BackyardFlyerController::BackyardFlyerController(const Config& cfg, IVehicle& vehicle)
    : cfg_(cfg), vehicle_(vehicle), planner_(cfg) {}

void BackyardFlyerController::StartMission() {
    in_mission_ = true;

    // We keep the state machine event-driven in spirit: Tick() will
    // advance states once the appropriate telemetry conditions are observed
}

void BackyardFlyerController::UpdateTelemetry(const TelemetrySample& sample) {
    last_sample_ = sample;
}

void BackyardFlyerController::Tick() {
    if (!in_mission_) {
        return;
    }

    if (!last_sample_.has_value()) {
        return;
    }

    const auto& s = *last_sample_;

    switch (state_) {
        case FlightState::Manual:
            ArmingTransition();
            break;
        
        case FlightState::Arming:
            // Equivalent of Python state_callback guard
            if (s.armed) {
                TakeoffTransition();
            }
            break;

        case FlightState::Takeoff:
            if (guards::AltitudeReached(s, cfg_)) {
                WaypointTransition();
            }
            break;

        case FlightState::Waypoint:
            if (guards::WaypointReachedXY(s, cfg_, target_)) {
                if (waypoint_index_ < waypoints_.size()) {
                    WaypointTransition();
                }
                else {
                    LandingTransition();
                }
            }
            break;

        case FlightState::Landing:
            if (guards::ReadyToDisarm(s, cfg_)) {
                DisarmingTransition();
            }
            break;
        
        case FlightState::Disarming:
            if (!s.armed) {
                ManualTransition();
            }
            break;
    };
}

void BackyardFlyerController::SetState(FlightState s) {
    state_ = s;
}

void BackyardFlyerController::ArmingTransition() {
    vehicle_.TakeControl();
    vehicle_.Arm();
    SetState(FlightState::Arming);
}

void BackyardFlyerController::TakeoffTransition() {
    vehicle_.Takeoff(cfg_.target_altitude_m);
    SetState(FlightState::Takeoff);
}

// Continue onward
void BackyardController::WaypointTransition() {
    if (!last_sample_.has_value()) {
        return;
    }
}



}