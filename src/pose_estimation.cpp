#include "pose_estimation.hpp"
#include <Eigen/Geometry>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
#include <opencv2/calib3d.hpp>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>

geometry_msgs::msg::Transform
homography(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize)
{
    apriltag_detection_info_t info = {detection, tagsize, intr[0], intr[1], intr[2], intr[3]};

    apriltag_pose_t pose;
    estimate_pose_for_tag_homography(&info, &pose);

    // rotate frame such that z points in the opposite direction towards the camera
    for(int i = 0; i < 3; i++) {
        // swap x and y axes
        std::swap(MATD_EL(pose.R, 0, i), MATD_EL(pose.R, 1, i));
        // invert z axis
        MATD_EL(pose.R, 2, i) *= -1;
    }

    return tf2::toMsg<apriltag_pose_t, geometry_msgs::msg::Transform>(const_cast<const apriltag_pose_t&>(pose));
}

geometry_msgs::msg::Transform
pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize)
{
    const std::vector<cv::Point3d> objectPoints{
        {-tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, +tagsize / 2, 0},
        {-tagsize / 2, +tagsize / 2, 0},
    };

    const std::vector<cv::Point2d> imagePoints{
        {detection->p[0][0], detection->p[0][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[3][0], detection->p[3][1]},
    };

    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];// fx
    cameraMatrix(1, 1) = intr[1];// fy
    cameraMatrix(0, 2) = intr[2];// cx
    cameraMatrix(1, 2) = intr[3];// cy

    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);

    return tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(std::make_pair(tvec, rvec));
}

geometry_msgs::msg::Transform
pnp_bundle(std::vector<apriltag_detection_t*> detections,
           const std::array<double, 4>& intr,
           std::unordered_map<int, double> tagsizes,
           std::unordered_map<int, std::vector<double>> transforms)
{
    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2d> imagePoints;

    for(auto& detection : detections) {
        int id = detection->id;
        std::vector<cv::Point3d> untransformed_object_points = {cv::Point3d(-(tagsizes[id] / 2), -(tagsizes[id] / 2), 0),
                                                                cv::Point3d(+(tagsizes[id] / 2), -(tagsizes[id] / 2), 0),
                                                                cv::Point3d(+(tagsizes[id] / 2), +(tagsizes[id] / 2), 0),
                                                                cv::Point3d(-(tagsizes[id] / 2), +(tagsizes[id] / 2), 0)};

        std::vector<double> transformation_vec = transforms[detection->id];
        // Add transformed object points to objectPoints
        for(cv::Point3d& untransformed_obj_pt : untransformed_object_points) {
            cv::Point3d transformed_obj_pt = apply_transform_to_point3d(
                untransformed_obj_pt, transformation_vec);
            objectPoints.push_back(transformed_obj_pt);
        }

        // Add image points
        for(int i = 0; i < 4; i++) {
            imagePoints.push_back({detection->p[i][0], detection->p[i][1]});
        }
    }

    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];// fx
    cameraMatrix(1, 1) = intr[1];// fy
    cameraMatrix(0, 2) = intr[2];// cx
    cameraMatrix(1, 2) = intr[3];// cy

    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);

    return tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(std::make_pair(tvec, rvec));
}

cv::Point3d apply_transform_to_point3d(cv::Point3d point, std::vector<double> tf_vec)
{
    Eigen::Vector3d point_vec(point.x, point.y, point.z);

    // Apply rotation first
    tf2::Quaternion quat;
    quat.setRPY(tf_vec[3], tf_vec[4], tf_vec[5]);
    Eigen::Vector3d rotated_point_vec;
    geometry_msgs::msg::TransformStamped rotation;
    rotation.transform.rotation.x = quat.x();
    rotation.transform.rotation.y = quat.y();
    rotation.transform.rotation.z = quat.z();
    rotation.transform.rotation.w = quat.w();
    tf2::doTransform(point_vec, rotated_point_vec, rotation);

    // Then apply a translation
    Eigen::Vector3d translated_pt_vec;
    geometry_msgs::msg::TransformStamped translation;
    translation.transform.translation.x = tf_vec[0];
    translation.transform.translation.y = tf_vec[1];
    translation.transform.translation.z = tf_vec[2];
    tf2::doTransform(rotated_point_vec, translated_pt_vec, translation);

    cv::Point3d transformed_point;
    transformed_point.x = translated_pt_vec[0];
    transformed_point.y = translated_pt_vec[1];
    transformed_point.z = translated_pt_vec[2];
    return transformed_point;
}

const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods{
    {"homography", homography},
    {"pnp", pnp},
};
