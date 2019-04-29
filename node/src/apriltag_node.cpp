#include <AprilTagNode.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagNode>());
    rclcpp::shutdown();
    return 0;
}
