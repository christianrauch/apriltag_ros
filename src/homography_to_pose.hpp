#pragma once

#include <Eigen/Dense>
#include <apriltag/apriltag.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>


typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;
typedef std::function<geometry_msgs::msg::Transform(apriltag_detection_t* const, const Mat3&, double)> estim_pose_f;

extern const std::unordered_map<std::string, estim_pose_f> estim_pose_fun;
