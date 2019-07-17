#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <image_transport/camera_subscriber.h>

// apriltag
#include <apriltag.h>

#include <Eigen/Core>


class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

    ~AprilTagNode() override;

private:
    typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;
    const std::string tag_family;
    const double tag_edge_size;
    const int max_hamming;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    Mat3 K;

    const bool z_up;

    // function pointer for tag family creation / destruction
    static const std::map<std::string, apriltag_family_t *(*)(void)> tag_create;
    const static std::map<std::string, void (*)(apriltag_family_t*)> tag_destroy;

    const image_transport::CameraSubscriber sub_cam;
    const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    void getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const double size) const;
};
