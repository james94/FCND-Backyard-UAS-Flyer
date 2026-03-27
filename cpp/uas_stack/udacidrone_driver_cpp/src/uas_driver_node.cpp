#include "udacidrone_driver_cpp/uas_driver_node.hpp"

#include <chrono>
#include <string>

using namespace std::chrono_literals;

// Before executing any command callback, this integrated node
// now does the following:
//
// 1. Pull "UasStateSnapshot" from repository.
// 2. Ask "session_manager_.isProgramRunning()"
// 3. Validate with "gatekeeper_.allow(...)"
// 4. Execute command only when allowed.
// 5. Report result to "health_monitor_.onCommandResult(...)"
// 6. Publish health summary periodically on "/uas/driver_health"

namespace udacidrone_driver_cpp {

UasDriverNode::UasDriverNode() : Node("uas_driver_node") {
    const std::string connection_uri = declare_parameter<std::string>("connection_uri", "tcp:127.0.0.1:5760");
    const bool is_px4 = declare_parameter<bool>("is_px4", false);
    use_mock_sim_ = declare_parameter<bool>("use_mock_sim", false);
    start_external_control_on_boot_ = declare_parameter<bool>("start_external_control_on_boot", false);

    log_telemetry_csv_ = declare_parameter<bool>("log_telemetry_csv", false);
    telemetry_log_csv_path_ = declare_parameter<std::string>("telemetry_log_csv_path", "/tmp/uas_telemetry.csv");

    session_service_timeout_sec_ = declare_parameter<double>("session_service_timeout_sec", 1.5);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 5.0);
    target_altitude_m_ = declare_parameter<double>("target_altitude_m", 3.0);

    command_service_ = std::make_unique<UasCommandService>(connection_, translator_, is_px4);
    connected_ = connection_.connect(connection_uri);
    state_repo_.setConnected(connected_);
    last_rx_time_ = now();

    if (!use_mock_sim_) {
        session_manager_ = std::make_unique<DriverSessionManager>(*this);
        const auto dep_timeout = std::chrono::milliseconds(
            static_cast<int>(session_service_timeout_sec_ * 1000.0)
        );
        const bool deps_ready = session_manager_->waitForDependencies(dep_timeout);

        if (!deps_ready) {
            health_monitor_.markDependencyNotReady();
        }
        if (deps_ready && start_external_control_on_boot_) {
            (void)session_manager_->startExternalControlSession();
        }

        (void)session_manager_->refreshProgramRunning(std::chrono::milliseconds(100));
    }

    armed_pub_ = create_publisher<std_msgs::msg::Bool>("/uas/armed", 10);
    local_position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/uas/local_position", 10);
    local_velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/uas/local_velocity", 10);
    global_position_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/uas/global_position", 10);
    driver_health_pub_ = create_publisher<std_msgs::msg::String>("/uas/driver_health", 10);

    cmd_position_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/uas/cmd_position", 10,
        std::bind(&UasDriverNode::cmdPositionCallback, this, std::placeholders::_1)
    );

    arm_srv_ = create_service<std_srvs::srv::SetBool>(
        "/uas/arm", 
        std::bind(&UasDriverNode::handleArm, this, std::placeholders::_1, std::placeholders::_2)
    );

    takeoff_srv_ = create_service<std_srvs::srv::Trigger>(
        "/uas/takeoff",
        std::bind(&UasDriverNode::handleTakeoff, this, std::placeholders::_1, std::placeholders::_2)
    );

    land_srv_ = create_service<std_srvs::srv::Trigger>(
        "/uas/land",
        std::bind(&UasDriverNode::handleLand, this, std::placeholders::_1, std::placeholders::_2)
    );

    read_timer_ = create_wall_timer(20ms, std::bind(&UasDriverNode::readLoop, this));
    watchdog_timer_ = create_wall_timer(200ms, std::bind(&UasDriverNode::watchdogLoop, this));
}

void UasDriverNode::readLoop() {
    mavros_msgs::msg::Mavlink mav_msg;

    if (!connection_.readMessage(mav_msg)) {
        return;
    }

    const bool decoded_ok = translator_.decode(mav_msg);
    health_monitor_.onRxFrame(decoded_ok, now());

    if (!decoded_ok) {
        return;
    }

    if (translator_.hasState()) {
        state_repo_.updateState(translator_.takeState());
    }
    if (translator_.hasPosition()) {
        state_repo_.updatePosition(translator_.takePosition());
    }
    if (translator_.hasVelocity()) {
        state_repo_.updateVelocity(translator_.takeVelocity());
    }

    last_rx_time_ = now();
    state_repo_.setConnected(true);
    publishTelemetry();
}

void UasDriverNode::watchdogLoop() {
    const double dt = (now() - last_rx_time_).seconds();

    if (dt > timeout_sec_) {
        state_repo_.setConnected(false);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Telemetry timeout detected");
    }

    if (session_manager_) {
        const auto refresh_timeout = std::chrono::milliseconds(
            static_cast<int>(session_service_timeout_sec_ * 500.0)
        );
        (void)session_manager_->refreshProgramRunning(refresh_timeout);
    }

    publishHealth();
}

void UasDriverNode::publishTelemetry() {
    const auto snap = state_repo_.snapshot();

    std_msgs::msg::Bool armed;
    armed.data = snap.state.armed;
    armed_pub_->publish(armed);

    geometry_msgs::msg::PointStamped lp;
    lp.header.stamp = now();
    lp.point.x = snap.position.north;
    lp.point.y = snap.position.east;
    lp.point.z = snap.position.down;
    local_position_pub_->publish(lp);

    geometry_msgs::msg::Vector3Stamped lv;
    lv.header.stamp = now();
    lv.vector.x = snap.velocity.vn;
    lv.vector.y = snap.velocity.ve;
    lv.vector.z = snap.velocity.vd;
    local_velocity_pub_->publish(lv);

    sensor_msgs::msg::NavSatFix gp;
    gp.header.stamp = now();
    gp.latitude = snap.position.latitude;
    gp.longitude = snap.position.longitude;
    gp.altitude = snap.position.altitude;
    global_position_pub_->publish(gp);

    // Optional node-local CSV log for quick parity with udacidrone drone.py logging
    // For full-fidelity ROS telemetry capture, prefer rosbag2 recording
        // from launch
    if (log_telemetry_csv_) {
        // Append one row: stamp, n, e, d, vn, ve, vd, lat, lon, alt, armed, connected
        // (Implementation detail intentionally omitted in this template.)
    }

}

void UasDriverNode::publishHealth() {
    const auto health = health_monitor_.evaluate(now(), timeout_sec_);
    std_msgs::msg::String msg;
    msg.data = "status=" + health.status_text 
        + ", heartbeat_age_sec=" + std::to_string(health.heartbeat_age_sec)
        + ", decode_errors=" + std::to_string(health.decode_errors) 
        + ", command_failures=" + std::to_string(health.command_failures);
    driver_health_pub_->publish(msg);
}

bool UasDriverNode::executeGuardedCommand(
    UasCommandType type,
    const std::function<bool()> &fn,
    const std::string &command_name,
    std::string *failure_reason
) {
    const auto snap = state_repo_.snapshot();
    const bool program_running = session_manager_ ? session_manager_->isProgramRunning() : true;

    if (!gatekeeper_.allow(type, snap, program_running)) {
        const std::string deny = gatekeeper_.denyReason(type, snap, program_running);

        if (failure_reason) {
            *failure_reason = deny;
        }

        RCLCPP_WARN(get_logger(), "Rejected %s: %s", command_name.c_str(), deny.c_str());
        health_monitor_.onCommandResult(false);
        return false;
    }

    const bool ok = fn();
    health_monitor_.onCommandResult(ok);
    
    if (!ok && failure_reason) {
        *failure_reason = "transport or encode failure";
    }

    return ok;
}

void UasDriverNode::handleArm(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res
) {
    std::string reason;

    const bool ok = req->data 
        ? executeGuardedCommand(
            UasCommandType::ARM, 
            [this]() {
                return command_service_->arm();
            },
            "arm",
            &reason)
        : executeGuardedCommand(
            UasCommandType::DISARM, 
            [this]() {
                return command_service_->disarm();
            },
            "disarm",
            &reason);
    
    res->success = ok;
    res->message = ok ? "arm/disarm command sent" : ("arm/disarm failed: " + reason);
}

void UasDriverNode::handleTakeoff(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res
) {
    (void)req;
    std::string reason;

    const bool ok = executeGuardedCommand(
        UasCommandType::TAKEOFF,
        [this]() {
            return command_service_->takeoff(static_cast<float>(target_altitude_m_));
        },
        "takeoff",
        &reason
    );

    res->success = ok;
    res->message = ok ? "takeoff command sent" : ("takeoff failed: " + reason);
}

void UasDriverNode::handleLand(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res
) {
    (void)req;
    std::string reason;

    const bool ok = executeGuardedCommand(
        UasCommandType::LAND,
        [this]() {
            return command_service_->land();
        },
        "land",
        &reason
    );

    res->success = ok;
    res->message = ok ? "land command sent" : ("land failed: " + reason);
}

void UasDriverNode::cmdPositionCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg
) {
    const float n = static_cast<float>(msg->pose.position.x);
    const float e = static_cast<float>(msg->pose.position.y);
    const float d = static_cast<float>(msg->pose.position.z);
    const float yaw = 0.0F;

    std::string reason;
    (void)executeGuardedCommand(
        UasCommandType::CMD_POSITION,
        [this, n, e, d, yaw]() {
            return command_service_->cmdPosition(n, e, d, yaw);
        },
        "cmd_position",
        &reason
    );
}


} // namespace udacidrone_driver_cpp
