// Controller / FSM (transitions only send commands once)

// We'll implement the same states:

// - Manual -> Arming -> Takeoff -> Waypoint -> Landing -> Disarming -> Manual

#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "backyard/config.hpp"
#include "backyard/planner.hpp"
#include "backyard/telemtry.hpp"
#include "backyard/types.hpp"
#include "backyard/vehicle.hpp"

namespace backyard {

enum class FlightState {
    Manual,
    Arming,
    Takeoff,
    Waypoint,
    Landing,
    Disarming,
};

// Why "Tick()" + "UpdateTelemetry()"?
    // - "UpdateTelemetry()" is called by the control loop after it copies
        // the latest sample.
    // - "Tick()" is where you evaluate guards and do transitions.

// This matches the "telemetry thread writes, fixed-rate loop reads" model

class BackyardFlyerController {
public:
    BackyardFlyerController(const Config& cfg, IVehicle& vehicle);

    void StartMission();
    void UpdateTelemetry(const TelemetrySample& sample);
    void Tick();

    FlightState state() const {
        return state_;
    }

    bool in_mission() const {
        return in_mission_;
    }

private:
    void SetState(FlightState s);

    // Transition (actions)
    void ArmingTransition();
    void TakeoffTransition();
    void WaypointTransition();
    void LandingTransition();
    void DisarmingTransition();
    void ManualTransition();

    const Config& cfg_;
    IVehicle& vehicle_;
    BoxPlanner planner_;

    FlightState state_{FlightState::Manual};
    bool in_mission_{false};

    std::optional<TelemetrySample> last_sample_;

    std::array<NED, 4> waypoints_{};
    std::size_t waypoint_index_{0};
    bool has_plan_{false};
    NED target_{};
};

} // namespace backyard
