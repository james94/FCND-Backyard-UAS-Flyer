#pragma once

// Simulator Connection:
//
// 1. Open/close simulator transport endpoint
// 2. Read MAVLink packets through libmavconn
// 3. Write outbound MAVLink packets through libmavconn
// 4. Surface connection health

#include <algorithm>
#include <atomic>
#include <deque>
#include <mutex>
#include <string>

#include <mavconn/interface.hpp>
#include <mavros_msgs/mavlink_convert.hpp>

namespace udacidrone_driver_cpp {

class SimulatorConnection {
public:
    bool connect(const std::string &uri);
    void disconnect();
    bool isConnected() const;
    bool readMessage(mavros_msgs::msg::Mavlink &out);
    bool sendMessage(const mavros_msgs::msg::Mavlink &msg);

private:
    mavconn::MAVConnInterface::Ptr link_;
    std::mutex rx_mutex_;
    std::deque<mavros_msgs::msg::Mavlink> rx_queue_;
    std::atomic<bool> connected_{false};
};

} // namespace udacidrone_driver_cpp
