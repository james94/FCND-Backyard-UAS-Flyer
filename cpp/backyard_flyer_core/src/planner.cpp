#include "backyard/planner.hpp"

namespace backyard {

std::array<NED, 4> BoxPlanner::BuildBox(double n0, double e0) const {
    const double d = -cfg_.target_altitude_m;
    const double s = cfg_.box_size_m;

    return {
        NED{n0 + s, e0 + 0.0, d},
        NED{n0 + s, e0 + s, d},
        NED{n0 + 0.0, e0 + s, d},
        NED{n0 + 0.0, e0 + 0.0, d},
    };
}

} // namespace backyard