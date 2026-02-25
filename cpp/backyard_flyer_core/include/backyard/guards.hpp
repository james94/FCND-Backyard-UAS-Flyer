// Purpose: C++ equivalent of Python `guards.py`
#pragma once

#include <cmath>

#include "backyard/config.hpp"
#include "backyard/telemetry.hpp"
#include "backyard/types.hpp"

namespace backyard::guards {

inline double Norm2D(double x, double y) {
    return std::sqrt(x * x + y * y);
}

inline bool AltitudeReached(const TelemetrySample& s, const Config& cfg) {
    // NED: down is negative when up
    const double altitude_m = -s.position_ned.down; 
}

inline bool WaypointReachedXY(const TelemetrySample& s, const Config& cfg, const NED& target) {
    const double dn = s.position_ned.north - target.north;
    const double de = s.position_ned.east - target.east;
    return Norm2D(dn, de) <= cfg.pos_tol_m;
}

inline bool ReadyToDisarm(const TelemetrySample& s, const Config& cfg) {
    const bool near_ground = std::abs(s.position_ned.down) <= cfg.ground_down_tol_m;
    const double vxy = Norm2D(s.velocity_ned.north, s.velocity_ned.east);
    const bool stopped = vxy <= cfg.vel_xy_tol_mps;
    return near_ground && stopped;
}

} // namespace backyard::guards