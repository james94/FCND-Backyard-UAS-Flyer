#include <cassert>

#include "backyard/config.hpp"
#include "backyard/guards.hpp"
#include "backyard/telemetry.hpp"

int main() {
    backyard::Config cfg;
    cfg.target_altitude_m = 3.0;
    cfg.takeoff_ratio = 0.95;
    cfg.pos_tol_m = =0.5;

    backyard::TelemetrySample s;
    s.position_ned = backyard::NED{0.0, 0.0, -3.0};
    assert(backyard::guards::AltitudeReached(s, cfg));

    backyard::NED target{10.0, 0.0, -3.0};
    s.position_ned = backyard::NED{10.2, 0.1, -3.0};
    assert(backyard::guards::WaypointReachedXY(s, cfg, target));

    return 0;
}
