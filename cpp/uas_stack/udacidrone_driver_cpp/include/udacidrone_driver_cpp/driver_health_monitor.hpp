#pragma once

// Class: Driver Health Monitor
//
// Responsibilities:
//
// 1. Track heartbeat age, decode errors, and command failure
// 2. Publish health status for observability and mission-level decisions
// 3. Emit transition events used by SMACC2 abort flows.

#include <atomic>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace udacidrone_driver_cpp {

struct DriverHealth {
    bool healthy{true};
    double heartbeat_age_sec{0.0};
    uint64_t decode_errors{0};
    uint64_t command_failures{0};
    std::string status_text;
};

class DriverHealthMonitor {
public:
    void onRxFrame(bool decoded_ok, rclcpp::Time stamp);
    void onCommandResult(bool ok);
    void markDependencyNotReady();
    DriverHealth evaluate(rclcpp::Time now, double timeout_sec) const;

private:
    mutable std::mutex mutex_;
    rclcpp::Time last_rx_time_{0, 0, RCL_ROS_TIME};
    std::atomic<uint64_t> decode_errors_{0};
    std::atomic<uint64_t> command_failures_{0};
    std::atomic<uint64_t> dependency_wait_events_{0};
};

} // namespace udacidrone_driver_cpp
