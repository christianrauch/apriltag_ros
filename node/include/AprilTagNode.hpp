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

    ~AprilTagNode();

private:
    typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;

    apriltag_family_t* tf;
    apriltag_detector_t* td;
    std::string tag_family;
    double tag_edge_size;
    int max_hamming;
    std::map<int, std::string> tracked_tags;

    Mat3 K;

    bool z_up;

    // function pointer for tag family creation / destruction
    static std::map<std::string, apriltag_family_t *(*)(void)> tag_create;
    static std::map<std::string, void (*)(apriltag_family_t*)> tag_destroy;

    image_transport::CameraSubscriber sub_cam;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf;
    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    void getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const bool z_up = false);
};
