#include "backyard/controller.hpp"
#include "backyard/guards.hpp"

// NOTE: 
// - This controller never sends commands "every tick"; only transitions send commands.
// - The "WaypointTransition()" computes waypoints once, then commands one waypoint per transition.

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
void BackyardFlyerController::WaypointTransition() {
    if (!last_sample_.has_value()) {
        return;
    }

    if(!has_plan_) {
        const auto& s = *last_sample_;
        waypoints_ = planner_.BuildBox(s.position_ned.north, s.position_ned.east);
        waypoint_index_ = 0;
        has_plan_ = true;
    }

    if (waypoint_index_ >= waypoints_.size()) {
        LandingTransition();
        return;
    }

    target_ = waypoints_[waypoint_index_];
    waypoint_index_++;

    vehicle_.CmdPosition(target_, cfg_.heading_rad);
    SetState(FlightState::Waypoint);
}

void BackyardFlyerController::LandingTransition() {
    vehicle_.Land();
    SetState(FlightState::Landing);
}

void BackyardFlyerController::DisarmingTransition() {
    vehicle_.Disarm();
    SetState(FlightState::Disarming);
}

void BackyardFlyerController::ManualTransition() {
    vehicle_.ReleaseControl();
    vehicle_.Stop();
    in_mission_ = false;
    SetState(FlightState::Manual);
}


}