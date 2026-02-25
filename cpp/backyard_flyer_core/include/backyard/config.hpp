// Purpose: keep mission parameters in one place 
// (mirrors the Python 'MissionConfig')

#pragma once

namespace backyard {

struct Config {
    double box_size_m{10.0};
    double target_altitude_m{3.0};

    double pos_tol_m{0.5};
    double vel_xy_tol_mps{0.5};
    double ground_down_tol_m{0.1};
    double takeoff_ratio{0.95};

    double heading_rad{0.0};

    // "More real-time": controller processing cadence
    double control_rate_hz{100.0};
};

} // namespace backyard
