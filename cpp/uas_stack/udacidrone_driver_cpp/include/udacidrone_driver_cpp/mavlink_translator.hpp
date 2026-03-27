#pragma once

#include "udacidrone_driver_cpp/telemetry_types.hpp"

// #include <mavros_msgs/mavlink_convert.hpp>

#include <mavros_msgs/msg/mavlink.hpp>

namespace udacidrone_driver_cpp
{
class MavlinkTranslator
{
public:
    bool decode(const mavros_msgs::msg::Mavlink &msg);
    bool hasState() const;
    UasStateTelemetry takeState();
    bool hasPosition() const;
    UasPositionTelemetry takePosition();
    bool hasVelocity() const;
    UasVelocityTelemetry takeVelocity();

    mavros_msgs::msg::Mavlink encodeArm(bool arm);
    mavros_msgs::msg::Mavlink encodeTakeControl();
    mavros_msgs::msg::Mavlink encodeReleaseControl();
    mavros_msgs::msg::Mavlink encodeTakeoff(float altitude_m);
    mavros_msgs::msg::Mavlink encodeLand();
    mavros_msgs::msg::Mavlink encodeCmdPosition(float n, float e, float d, float yaw);

private:
    bool has_state_{false};
    bool has_position_{false};
    bool has_velocity_{false};
    UasStateTelemetry state_{};
    UasPositionTelemetry position_{};
    UasVelocityTelemetry velocity_{};
};
}  // namespace udacidrone_driver_cpp
