#include "udacidrone_driver_cpp/uas_driver_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<udacidrone_driver_cpp::UasDriverNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
