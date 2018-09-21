#include <AprilTag2Node.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTag2Node>());
    rclcpp::shutdown();
    return 0;
}
