// Vehicle interface (adapter seam)

// In Python, UdaciDrone is your "vehicle API". 
// In C++, you want a seam so that:

    // - The controller can be unit tested without a simulator
    // - Transport/MAVLink/MAVSDK code stays out of your FSM core.

// NOTE: 
    // - This matches the outgoing command list in "README.md" at concept level
    // - We can extend it later for "set home", etc

#pragma once

#include "backyard/types.hpp"

namespace backyard {

class IVehicle {
public:
    virtual ~IVehicle() = default;

    virtual void TakeControl() = 0;
    virtual void ReleaseControl() = 0;

    virtual void Arm() = 0;
    virtual void Disarm() = 0;

    virtual void Takeoff(double target_altitude_m) = 0;
    virtual void Land() = 0;

    virtual void CmdPosition(const NED& ned, double heading_rad) = 0;

    // Optional lifecycle hooks (depends on your transport)
    virtual void Stop();
};

}
