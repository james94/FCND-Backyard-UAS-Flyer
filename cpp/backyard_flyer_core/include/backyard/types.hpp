// Purpose: explicit coordinate types + small helpers

#pragma once

#include <cstdint>

// NOTE: NED matches Udacity local frame from "README.md"

namespace backyard {

struct NED {
    double north{0.0};
    double east{0.0};
    double down{0.0};
};

inline NED operator+(const NED& a, const NED& b) {
    return NED{a.north + b.north, a.east + b.east, a.down + b.down};
}

inline NED operator-(const NED& a, const NED& b) {
    return NED{a.north - b.north, a.east - b.east, a.down - b.down};
}

} // namespace backyard
