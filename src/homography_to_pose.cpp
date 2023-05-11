#include "homography_to_pose.hpp"
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/quaternion.hpp>


geometry_msgs::msg::Transform tf_from_eigen(const Eigen::Vector3d& translation, const Mat3& rotation)
{
    const Eigen::Quaterniond q(rotation);

    geometry_msgs::msg::Transform t;

    t.translation.x = translation.x();
    t.translation.y = translation.y();
    t.translation.z = translation.z();
    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();

    return t;
}

geometry_msgs::msg::Transform tf_from_cv(const cv::Mat_<double>& tvec, const cv::Mat_<double>& rvec)
{
    const cv::Quat<double> q = cv::Quat<double>::createFromRvec(rvec);

    geometry_msgs::msg::Transform t;

    t.translation.x = tvec.at<double>(0);
    t.translation.y = tvec.at<double>(1);
    t.translation.z = tvec.at<double>(2);
    t.rotation.w = q.w;
    t.rotation.x = q.x;
    t.rotation.y = q.y;
    t.rotation.z = q.z;

    return t;
}

estim_pose_f from_homography = [](const apriltag_detection_t* const detection, const Mat3& P, double size) -> geometry_msgs::msg::Transform {
    // compute extrinsic camera parameter
    // https://dsp.stackexchange.com/a/2737/31703
    // H = K * T  =>  T = K^(-1) * H
    const Mat3 T = P.inverse() * Eigen::Map<const Mat3>(detection->H->data);
    Mat3 R;
    R.col(0) = T.col(0).normalized();
    R.col(1) = T.col(1).normalized();
    R.col(2) = R.col(0).cross(R.col(1));

    // rotate by half rotation about x-axis to have z-axis
    // point upwards orthogonal to the tag plane
    R.col(1) *= -1;
    R.col(2) *= -1;

    // the corner coordinates of the tag in the canonical frame are (+/-1, +/-1)
    // hence the scale is half of the edge size
    const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm()) / 2.0) * (size / 2.0);

    return tf_from_eigen(tt, R);
};

estim_pose_f apriltag_orthogonal_iteration = [](apriltag_detection_t* const detection, const Mat3& P, double tagsize) -> geometry_msgs::msg::Transform {
    // https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#pose-estimation
    apriltag_detection_info_t info;
    info.det = detection;
    info.tagsize = tagsize;
    info.fx = P(0, 0);
    info.fy = P(1, 1);
    info.cx = P(0, 1);
    info.cy = P(1, 0);

    apriltag_pose_t pose;
    estimate_tag_pose(&info, &pose);

    return tf_from_eigen(Eigen::Map<const Eigen::Vector3d>(pose.t->data), Eigen::Map<const Mat3>(pose.R->data));
};

estim_pose_f apriltag_homography = [](apriltag_detection_t* const detection, const Mat3& P, double tagsize) -> geometry_msgs::msg::Transform {
    apriltag_detection_info_t info;
    info.det = detection;
    info.tagsize = tagsize;
    info.fx = P(0, 0);
    info.fy = P(1, 1);
    info.cx = P(0, 1);
    info.cy = P(1, 0);

    apriltag_pose_t pose;
    estimate_pose_for_tag_homography(&info, &pose);

    return tf_from_eigen(Eigen::Map<const Eigen::Vector3d>(pose.t->data), Eigen::Map<const Mat3>(pose.R->data));
};

estim_pose_f solve_pnp = [](apriltag_detection_t* const detection, const Mat3& P, double tagsize) -> geometry_msgs::msg::Transform {
    std::vector<cv::Point3d> objectPoints;
    objectPoints.emplace_back(-tagsize, -tagsize, 0);
    objectPoints.emplace_back(+tagsize, -tagsize, 0);
    objectPoints.emplace_back(+tagsize, +tagsize, 0);
    objectPoints.emplace_back(-tagsize, +tagsize, 0);

    std::vector<cv::Point2d> imagePoints;
    double tag_x[4] = {-1, 1, 1, -1};
    double tag_y[4] = {1, 1, -1, -1};
    for(int i = 0; i < 4; i++) {
        // Homography projection taking tag local frame coordinates to image pixels
        double im_x, im_y;
        homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
        imagePoints.push_back(cv::Point2d(im_x, im_y));
    }

    cv::Mat rvec, tvec;
    cv::Matx33d cameraMatrix;
    cv::eigen2cv(P, cameraMatrix);
    // with "SOLVEPNP_IPPE_SQUARE"?
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);
    //    cv::Matx33d R;
    //    cv::Rodrigues(rvec, R);

    return tf_from_cv(tvec, rvec);
};

const std::unordered_map<std::string, estim_pose_f> estim_pose_fun{
    {"from_homography", from_homography},
    {"apriltag_orthogonal_iteration", apriltag_orthogonal_iteration},
    {"apriltag_homography", apriltag_homography},
    {"solve_pnp", solve_pnp},
};
