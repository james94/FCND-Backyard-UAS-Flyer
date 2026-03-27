#include "udacidrone_driver_cpp/mavlink_translator.hpp"

#include <cmath>

// This translator implementation mirrors UdaciDrone Python behavior
// while using mavros conversion APIs:
//
// 1. Decodes "HEARTBEAT", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"
    // into normalized state/position/velocity.
// 2. Uses "MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_DO_SET_MODE"
    // for arm/offboard/manual transitions.
// 3. Uses "SET_POSITION_TARGET_LOCAL_NED" masks for takeoff, land,
    // and cmd_position semantics.
// 4. Uses "mavros_msgs::mavlink::convert()" for ROS<->MAVLink
    // conversion boundaries.

#if __has_include(<mavlink/v2.0/mavlink.h>)
extern "C" {
#ifndef MAVLINK_DIALECT
#define MAVLINK_DIALECT common
#endif
#include <mavlink/v2.0/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#elif __has_include(<mavlink/v2.0/common/mavlink.h>)
extern "C" {
#include <mavlink/v2.0/common/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#elif __has_include(<mavconn/mavlink_dialect.hpp>)
#include <mavconn/mavlink_dialect.hpp>
#elif __has_include(<mavlink/v2.0/minimal/mavlink.h>)
extern "C" {
#include <mavlink/v2.0/minimal/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#elif __has_include(<mavlink/common/mavlink.h>)
extern "C" {
#include <mavlink/common/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#elif __has_include(<common/mavlink.h>)
extern "C" {
#include <common/mavlink.h>
}
namespace mavlink {
using mavlink_message_t = ::mavlink_message_t;
}
#else
#error "No MAVLink dialect header found. Install ros-jazzy-mavlink and ros-jazzy-libmavconn."
#endif

// Prevent macro collision with MAVLink C++ headers/types in downstream includes.
#ifdef MAVLINK_VERSION
#undef MAVLINK_VERSION
#endif

#include <mavros_msgs/mavlink_convert.hpp>

namespace udacidrone_driver_cpp {

namespace {

constexpr uint8_t kSysId = 255;
// Avoid pulling mavros_uas.hpp (which introduces tf2_ros compile-time deps).
constexpr uint8_t kCompId = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER
constexpr uint8_t kTargetSys = 1;
constexpr uint8_t kTargetComp = 1;

constexpr float kMainModeManual = 1.0F;
constexpr float kMainModeOffboard = 6.0F;

constexpr uint16_t kMaskIgnorePosition = 0x007;
constexpr uint16_t kMaskIgnoreVelocity = 0x038;
constexpr uint16_t kMaskIgnoreAcceleration = 0x1C0;
constexpr uint16_t kMaskIgnoreYaw = 0x400;
constexpr uint16_t kMaskIgnoreYawRate = 0x800;
constexpr uint16_t kMaskIsTakeoff = 0x1000;
constexpr uint16_t kMaskIsLand = 0x2000;

mavros_msgs::msg::Mavlink toRosMavlink(const mavlink::mavlink_message_t &msg) {
    mavros_msgs::msg::Mavlink out;
    (void)mavros_msgs::mavlink::convert(msg, out);
    return out;
}

mavros_msgs::msg::Mavlink encodeCommandLong(
    uint16_t command,
    float p1,
    float p2 = 0.0F,
    float p3 = 0.0F,
    float p4 = 0.0F,
    float p5 = 0.0F,
    float p6 = 0.0F,
    float p7 = 0.0F
) {
    mavlink::mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        kSysId,
        kCompId,
        &msg,
        kTargetSys,
        kTargetComp,
        command,
        0,
        p1,
        p2,
        p3,
        p4,
        p5,
        p6,
        p7);

    return toRosMavlink(msg);
}

mavros_msgs::msg::Mavlink encodeSetPositionTarget(
    uint16_t mask,
    float n,
    float e,
    float d,
    float vn,
    float ve,
    float vd,
    float yaw,
    float yaw_rate
) {
    mavlink::mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        kSysId,
        kCompId,
        &msg,
        0,
        kTargetSys,
        kTargetComp,
        MAV_FRAME_LOCAL_NED,
        mask,
        n,
        e,
        d,
        vn,
        ve,
        vd,
        0.0F,
        0.0F,
        0.0F,
        yaw,
        yaw_rate
    );

    return toRosMavlink(msg);
}
} // namespace

bool MavlinkTranslator::decode(const mavros_msgs::msg::Mavlink &msg) {
    has_state_ = false;
    has_position_ = false;
    has_velocity_ = false;

    mavlink::mavlink_message_t wire;
    if (!mavros_msgs::mavlink::convert(msg, wire)) {
        return false;
    }

    switch(wire.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&wire, &hb);

            state_.stamp_sec = 0.0;
            state_.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            const uint32_t main_mode = (hb.custom_mode & 0x000F0000U) >> 16U;
            state_.guided = (main_mode == static_cast<uint32_t>(kMainModeOffboard));
            state_.status = static_cast<int32_t>(hb.system_status);
            has_state_ = true;

            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t lp;
            mavlink_msg_local_position_ned_decode(&wire, &lp);
            const double t = static_cast<double>(lp.time_boot_ms) / 1000.0;

            position_.stamp_sec = t;
            position_.north = lp.x;
            position_.east = lp.y;
            position_.down = lp.z;
            has_position_ = true;

            velocity_.stamp_sec = t;
            velocity_.vn = lp.vx;
            velocity_.ve = lp.vy;
            velocity_.vd = lp.vz;
            has_velocity_ = true;
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t gp;
            mavlink_msg_global_position_int_decode(&wire, &gp);
            const double t = static_cast<double>(gp.time_boot_ms) / 1000.0;

            position_.stamp_sec = t;
            position_.latitude = static_cast<double>(gp.lat) / 1e7;
            position_.longitude = static_cast<double>(gp.lon) / 1e7;
            position_.altitude = static_cast<double>(gp.alt) / 1000.0;
            has_position_ = true;

            velocity_.stamp_sec = t;
            velocity_.vn = static_cast<double>(gp.vx) / 100.0;
            velocity_.ve = static_cast<double>(gp.vy) / 100.0;
            velocity_.vd = static_cast<double>(gp.vz) / 100.0;
            has_velocity_ = true;

            break;
        }
        default:
            break;
    }
    
    return true;
}

bool MavlinkTranslator::hasState() const {
    return has_state_;
}

UasStateTelemetry MavlinkTranslator::takeState() {
    has_state_ = false;
    return state_;
}

bool MavlinkTranslator::hasPosition() const {
    return has_position_;
}

UasPositionTelemetry MavlinkTranslator::takePosition() {
    has_position_ = false;
    return position_;
}

bool MavlinkTranslator::hasVelocity() const {
    return has_velocity_;
}

UasVelocityTelemetry MavlinkTranslator::takeVelocity() {
    has_velocity_ = false;
    return velocity_;
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeArm(bool arm) {
    return encodeCommandLong(
        MAV_CMD_COMPONENT_ARM_DISARM, arm ? 1.0F : 0.0F);
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeTakeControl() {
    return encodeCommandLong(
        MAV_CMD_DO_SET_MODE,
        static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        kMainModeOffboard,
        0.0F
    );
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeReleaseControl() {
    return encodeCommandLong(
        MAV_CMD_DO_SET_MODE,
        static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        kMainModeManual,
        0.0F
    );
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeTakeoff(float altitude_m) {
    const float target_d = -std::fabs(altitude_m);
    const uint16_t mask = static_cast<uint16_t>(
        kMaskIsTakeoff | kMaskIgnoreYawRate | kMaskIgnoreYaw | kMaskIgnoreAcceleration | kMaskIgnoreVelocity
    );

    return encodeSetPositionTarget(
        mask, 0.0F, 0.0F, target_d, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F
    );
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeLand() {
    const uint16_t mask = static_cast<uint16_t>(
        kMaskIsLand | kMaskIgnoreYawRate | kMaskIgnoreYaw | kMaskIgnoreAcceleration | kMaskIgnoreVelocity
    );

    return encodeSetPositionTarget(
        mask, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F
    );
}

mavros_msgs::msg::Mavlink MavlinkTranslator::encodeCmdPosition(float n, float e, float d, float yaw) {
    const uint16_t mask = static_cast<uint16_t>(
        kMaskIgnoreYawRate | kMaskIgnoreAcceleration | kMaskIgnoreVelocity
    );

    return encodeSetPositionTarget(
        mask, n, e, d, 0.0F, 0.0F, 0.0F, yaw, 0.0F
    );
}


} // namespace udacidrone_driver_cpp

