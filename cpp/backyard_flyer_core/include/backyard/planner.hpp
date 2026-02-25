// Purpose: generate the 4 corners of the box in local coordinates
    // Planner (10m box waypoints)

#pragma once

#include <array>

#include "backyard/config.hpp"
#include "backyard/types.hpp"

namespace backyard {

class BoxPlanner {
public:
    explicit BoxPlanner(const Config& cfg) : cfg_(cfg) {}

    // Origin is current local (north, east). Altitude uses NED down sign.
    std::array<NED, 4> BuildBox(double n0, double e0) const;

private:
    const Config& cfg_;

};

} // namespace backyard
