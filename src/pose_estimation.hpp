#pragma once

#include <apriltag/apriltag.h>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>
#include <opencv2/calib3d.hpp>

typedef std::function<geometry_msgs::msg::Transform(apriltag_detection_t* const, const std::array<double, 4>&, const double&)> pose_estimation_f;

extern const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods;

geometry_msgs::msg::Transform pnp_bundle(std::vector<apriltag_detection_t*> detections,
                                         const std::array<double, 4>& intr,
                                         const std::unordered_map<int, std::array<cv::Point3d, 4>>& id_to_corners);
