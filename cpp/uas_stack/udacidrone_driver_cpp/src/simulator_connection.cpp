#include "udacidrone_driver_cpp/simulator_connection.hpp"

// #include <algorithm>
// #include <deque>
// #include <mutex>
// #include <string>

// #include <mavconn/interface.hpp>
// #include <mavros_msgs/mavlink_convert.hpp>


namespace udacidrone_driver_cpp {

namespace {
constexpr uint8_t kCompIdOnboardComputer = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER
}

bool SimulatorConnection::connect(const std::string &uri) {
    // Example URI expected: tcp:127.0.0.1:5760
    // Parse and open socket here; return true on success

    // Supported format mirrors UdaciDrone usage: tcp:<host>:<port>
    disconnect();

    std::string mavconn_url = uri;

    if (uri.rfind("tcp:", 0) == 0) {
        mavconn_url = "tcp://" + uri.substr(4);
    }

    try {
        link_ = mavconn::MAVConnInterface::open_url(
            mavconn_url,
            1,
            kCompIdOnboardComputer,
            [this](const mavlink::mavlink_message_t *message, const mavconn::Framing framing) {
                mavros_msgs::msg::Mavlink ros_msg;
                if (!mavros_msgs::mavlink::convert(*message, ros_msg, static_cast<uint8_t>(framing))) {
                    return;
                }
                std::lock_guard<std::mutex> lock(rx_mutex_);
                rx_queue_.push_back(std::move(ros_msg));
            },
            [this]() {
                connected_.store(false);
            }
        );
    }
    catch (...) {
        link_.reset();
        connected_.store(false);
        return false;
    }

    connected_.store(static_cast<bool>(link_) && link_->is_open());
    return connected_.load();
}

void SimulatorConnection::disconnect() {
    if (link_) {
        link_->close();
        link_.reset();
    }
    connected_.store(false);
    std::lock_guard<std::mutex> lock(rx_mutex_);
    rx_queue_.clear();
}

bool SimulatorConnection::isConnected() const {
    return connected_.load() && link_ && link_->is_open();
}

bool SimulatorConnection::readMessage(mavros_msgs::msg::Mavlink &out) {
    if (!isConnected()) {
        return false;
    }
    std::lock_guard<std::mutex> lock(rx_mutex_);
    if (rx_queue_.empty()) {
        return false;
    }

    out = std::move(rx_queue_.front());
    rx_queue_.pop_front();
    return true;
}

bool SimulatorConnection::sendMessage(const mavros_msgs::msg::Mavlink &msg) {
    if (!isConnected()) {
        return false;
    }

    mavlink::mavlink_message_t wire_msg;
    if (!mavros_msgs::mavlink::convert(msg, wire_msg)) {
        return false;
    }

    link_->send_message(&wire_msg);
    return true;
}

} // namespace udacidrone_driver_cpp
