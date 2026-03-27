#include "udacidrone_driver_cpp/driver_health_monitor.hpp"

namespace udacidrone_driver_cpp {

void DriverHealthMonitor::onRxFrame(bool decoded_ok, rclcpp::Time stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rx_time_ = stamp;

    if (!decoded_ok) {
        ++decode_errors_;
    }
}

void DriverHealthMonitor::onCommandResult(bool ok) {
    if (!ok) {
        ++command_failures_;
    }
}

void DriverHealthMonitor::markDependencyNotReady() {
    ++dependency_wait_events_;
}

DriverHealth DriverHealthMonitor::evaluate(rclcpp::Time now, double timeout_sec) const {
    DriverHealth out;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        out.heartbeat_age_sec = (now - last_rx_time_).seconds();
    }

    out.decode_errors = decode_errors_;
    out.command_failures = command_failures_;
    out.healthy = (out.heartbeat_age_sec <= timeout_sec) && (dependency_wait_events_ == 0);
    out.status_text = out.healthy ? "ok" : "degraded";

    return out;
}

} // namespace udacidrone_driver_cpp
