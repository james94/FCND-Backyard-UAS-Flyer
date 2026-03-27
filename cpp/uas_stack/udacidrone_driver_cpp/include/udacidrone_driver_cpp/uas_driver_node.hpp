#pragma once

#include <functional>
#include <memory>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "udacidrone_driver_cpp/command_gatekeeper.hpp"
#include "udacidrone_driver_cpp/driver_health_monitor.hpp"
#include "udacidrone_driver_cpp/driver_session_manager.hpp"
#include "udacidrone_driver_cpp/mavlink_translator.hpp"
#include "udacidrone_driver_cpp/simulator_connection.hpp"
#include "udacidrone_driver_cpp/uas_command_service.hpp"
#include "udacidrone_driver_cpp/uas_state_repository.hpp"

namespace udacidrone_driver_cpp {

class UasDriverNode : public rclcpp::Node {
public:
    UasDriverNode();

private:
    void readLoop();
    void watchdogLoop();
    void publishTelemetry();
    void publishHealth();

    bool executeGuardedCommand(
        UasCommandType type,
        const std::function<bool()> &fn,
        const std::string &command_name,
        std::string *failure_reason = nullptr
    );

    void handleArm(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res
    );

    void handleTakeoff(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
    );

    void handleLand(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res
    );

    void cmdPositionCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg
    );

    SimulatorConnection connection_;
    MavlinkTranslator translator_;
    UasStateRepository state_repo_;
    std::unique_ptr<UasCommandService> command_service_;

    std::unique_ptr<DriverSessionManager> session_manager_;
    CommandGatekeeper gatekeeper_;
    DriverHealthMonitor health_monitor_;

    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr local_velocity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_position_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr driver_health_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_position_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;

    bool connected_{false};
    bool use_mock_sim_{false};
    bool start_external_control_on_boot_{false};
    bool log_telemetry_csv_{false};
    std::string telemetry_log_csv_path_{};
    double session_service_timeout_sec_{1.5};
    rclcpp::Time last_rx_time_;
    double timeout_sec_{5.0};
    double target_altitude_m_{3.0};
};

} // namespace udacidrone_driver_cpp
