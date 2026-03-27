#include "udacidrone_driver_cpp/driver_session_manager.hpp"

#include <future>
#include <memory>

namespace udacidrone_driver_cpp {

DriverSessionManager::DriverSessionManager(rclcpp::Node &node) : node_(node) {
    // Keep namespaced service paths so bringup can remap cleanly
    start_session_client_ = node_.create_client<std_srvs::srv::Trigger>("/uas/session/start_external_control");
    stop_session_client_ = node_.create_client<std_srvs::srv::Trigger>("/uas/session/stop_external_control");
    program_running_client_ = node_.create_client<std_srvs::srv::Trigger>("/uas/session/program_running");
}

bool DriverSessionManager::waitForDependencies(std::chrono::milliseconds timeout) {
    const auto t = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
    const bool start_ok = start_session_client_->wait_for_service(t);
    const bool stop_ok = stop_session_client_->wait_for_service(t);
    const bool running_ok = program_running_client_->wait_for_service(t);

    if (!start_ok || !stop_ok || !running_ok) {
        RCLCPP_WARN(node_.get_logger(), "Session dependencies not ready within timeout");
        return false;
    }

    RCLCPP_INFO(node_.get_logger(), "Session dependencies are ready");
    return true;
}

bool DriverSessionManager::startExternalControlSession() {
    bool ok = false;
    std::string message;

    if (!callTrigger(start_session_client_, std::chrono::milliseconds(1000), &ok, &message)) {
        RCLCPP_WARN(node_.get_logger(), "Failed to call start_external_control service");
        return false;
    }

    program_running_.store(ok);

    if (!message.empty()) {
        RCLCPP_INFO(node_.get_logger(), "start_external_control response: %s", message.c_str());
    }

    return ok;
}

bool DriverSessionManager::stopExternalControlSession() {
    bool ok = false;
    std::string message;

    if (!callTrigger(stop_session_client_, std::chrono::milliseconds(1000), &ok, &message)) {
        RCLCPP_WARN(node_.get_logger(), "Failed to call stop_external_control service");
        return false;
    }

    program_running_.store(!ok ? program_running_.load() : false);

    if (!message.empty()) {
        RCLCPP_INFO(node_.get_logger(), "stop_external_control response: %s", message.c_str());
    }

    return ok;
}

bool DriverSessionManager::refreshProgramRunning(std::chrono::milliseconds timeout) {
    bool ok = false;

    if (!callTrigger(program_running_client_, timeout, &ok, nullptr)) {
        return false;
    }

    program_running_.store(ok);
    return true;
}

bool DriverSessionManager::isProgramRunning() const {
    return program_running_.load();
}

bool DriverSessionManager::callTrigger(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
    std::chrono::milliseconds timeout,
    bool *ok_out,
    std::string *message_out
) {
    if (!client) {
        return false;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = client->async_send_request(req);

    if (fut.wait_for(timeout) != std::future_status::ready) {
        return false;
    }

    const auto res = fut.get();

    if (ok_out != nullptr) {
        *ok_out = res->success;
    }
    if (message_out != nullptr) {
        *message_out = res->message;
    }

    return true;
}


} // namespace udacidrone_driver_cpp
