#include <cassert>
#include <vector>

#include "backyard/config.hpp"
#include "backyard/controller.hpp"
#include "backyard/vehicle.hpp"

struct Call {
    enum class Type {
        TakeControl, ReleaseControl, Arm, Disarm, Takeoff, Land, CmdPosition, Stop
    } type;
    backyard::NED ned;
    double heading{0.0};
    double alt{0.0};
};

class MockVehicle : public backyard::IVehicle {
public:
    std::vector<Call> calls;

    void TakeControl() override {
        call.push_back({Call::Type::TakeControl});
    }

    void ReleaseConotrol() override {
        calls.push_back({Call::Type::ReleaseControl});
    }

    void Arm() override {
        calls.push_back({Call::Type::Arm});
    }

    void Disarm() override {
        calls.push_back({Call::Type::Disarm});
    }

    void Takeoff(double target_altitude_m) override {
        calls.push_back({Call::Type::Takeoff, {}, 0.0, target_altitude_m});
    }

    void Land() override {
        calls.push_back({Call::Type::Land});
    }

    void CmdPosition(const backyard::NED& ned, double heading_rad) override {
        Call c{Call::Type::CmdPosition};
        c.ned = ned;
        c.heading  =heading_rad;
        calls.push_back(c);
    }

    void Stop() override {
        calls.push_back({Call::Type::Stop});
    }
};

// This test only checks the first few transitions; you can extend it to
// cover waypoint command count (4) and landing/disarm

int main() {
    backyard::Config cfg;
    MockVehicle vehicle;
    backyard::BackyardFlyerController ctl(cfg, vehicle);

    ctl.StartMission();

    backyard::TelemetrySample s;
    s.armed = false;
    ctl.UpdateTelemetry(s);
    ctl.Tick();
    assert(vehicle.calls.size() >= 2); // TakeControl + Arm

    // Simulate armed
    s.armed = true;
    ctl.UpdateTelemetry(s);
    ctl.Tick();

    // Expect takeoff command
    bool saw_takeoff = false;

    for (const auto& c : vehicle.calls) {
        if (c.type == Call::Type::Takeoff) {
            saw_takeoff = true;
            break;
        }
    }

    assert(saw_takeoff);

    return 0;
}
