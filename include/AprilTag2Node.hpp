#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

// apriltag
#include <apriltag.h>
#include <common/pjpeg.h>

#include <Eigen/Core>


class AprilTag2Node : public rclcpp::Node {
public:
    AprilTag2Node();

    ~AprilTag2Node();

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

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf;
    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

    void onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img);

    void getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const bool z_up = false);
};
