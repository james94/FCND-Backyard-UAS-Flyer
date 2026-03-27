#pragma once

// Class: Driver Session Manager
//
// Responsibilities:
//
// 1. Handle simulator session lifecycle and external-control handshake.
// 2. Expose run-state and connection-state signals to the main driver node.
// 3. Wrap service-level operations such as start/stop/reset program.

#include <atomic>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace udacidrone_driver_cpp {

class DriverSessionManager {
public:
    explicit DriverSessionManager(rclcpp::Node &node);

    bool waitForDependencies(std::chrono::milliseconds timeout);
    bool startExternalControlSession();
    bool stopExternalControlSession();
    bool refreshProgramRunning(std::chrono::milliseconds timeout);
    bool isProgramRunning() const;

private:
    bool callTrigger(
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
        std::chrono::milliseconds timeout,
        bool *ok_out = nullptr,
        std::string *message_out = nullptr
    );

    rclcpp::Node &node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_session_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_session_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr program_running_client_;
    std::atomic<bool> program_running_{false};
};

} // namespace udacidrone_driver_cpp
