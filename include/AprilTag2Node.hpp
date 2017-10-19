#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

// apriltag
#include <apriltag.h>
#include <tag36h11.h>
#include <common/pjpeg.h>

#include <iomanip>


class AprilTag2Node : public rclcpp::Node {
public:
    AprilTag2Node();

    ~AprilTag2Node();

private:
    apriltag_family_t* tf;
    apriltag_detector_t* td;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_pose;
    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

    void onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img);
};
