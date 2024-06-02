#include <Eigen/Geometry>
#include <apriltag/common/matd.h>
#include <apriltag_pose.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/core/mat.hpp>
#include <tf2/convert.h>

template<>
void tf2::convert(const Eigen::Quaterniond& eigen_quat, geometry_msgs::msg::Quaternion& msg_quat)
{
    msg_quat.w = eigen_quat.w();
    msg_quat.x = eigen_quat.x();
    msg_quat.y = eigen_quat.y();
    msg_quat.z = eigen_quat.z();
}

template<>
void tf2::convert(const matd_t& mat, geometry_msgs::msg::Vector3& msg_vec)
{
    assert((mat.nrows == 3 && mat.ncols == 1) || (mat.nrows == 1 && mat.ncols == 3));

    msg_vec.x = mat.data[0];
    msg_vec.y = mat.data[1];
    msg_vec.z = mat.data[2];
}

template<>
void tf2::convert(const cv::Mat_<double>& vec, geometry_msgs::msg::Vector3& msg_vec)
{
    assert((vec.rows == 3 && vec.cols == 1) || (vec.rows == 1 && vec.cols == 3));

    msg_vec.x = vec.at<double>(0);
    msg_vec.y = vec.at<double>(1);
    msg_vec.z = vec.at<double>(2);
}

template<>
geometry_msgs::msg::Transform
tf2::toMsg(const apriltag_pose_t& pose)
{
    const Eigen::Quaterniond q(Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(pose.R->data));

    geometry_msgs::msg::Transform t;
    tf2::convert(*pose.t, t.translation);
    tf2::convert(q, t.rotation);
    return t;
}

template<>
geometry_msgs::msg::Transform
tf2::toMsg(const std::pair<cv::Mat_<double>, cv::Mat_<double>>& pose)
{
    assert((pose.first.rows == 3 && pose.first.cols == 1) || (pose.first.rows == 1 && pose.first.cols == 3));
    assert((pose.second.rows == 3 && pose.second.cols == 1) || (pose.second.rows == 1 && pose.second.cols == 3));

    // convert compact rotation vector to angle-axis to quaternion
    const Eigen::Map<const Eigen::Vector3d> rvec(reinterpret_cast<double*>(pose.second.data));
    const Eigen::Quaterniond q({rvec.norm(), rvec.normalized()});

    geometry_msgs::msg::Transform t;
    tf2::convert(pose.first, t.translation);
    tf2::convert(q, t.rotation);
    return t;
}
