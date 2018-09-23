#include <AprilTag2Node.hpp>
#include <class_loader/register_macro.hpp>

// default tag families
#include <tag16h5.h>
#include <tag25h7.h>
#include <tag25h9.h>
#include <tag36h10.h>
#include <tag36h11.h>
#include <tag36artoolkit.h>

#include <Eigen/Dense>

AprilTag2Node::AprilTag2Node() : Node("apriltag2", "apriltag", true) {
    sub_img = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",
        std::bind(&AprilTag2Node::onImage, this, std::placeholders::_1),
        rmw_qos_profile_sensor_data);
    rmw_qos_profile_t tf_qos_profile = rmw_qos_profile_default;
    tf_qos_profile.depth = 100;
    pub_tf = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf_qos_profile);
    pub_detections = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections");

    // get single camera info message
    sub_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "image/camera_info",
        [this](sensor_msgs::msg::CameraInfo::UniquePtr info){
            RCLCPP_INFO(get_logger(), "got camera parameters");
            std::memcpy(K.data(), info->k.data(), 9*sizeof(double));
            // delete subscription
            sub_info.reset();
        }
    );

    get_parameter_or<std::string>("family", tag_family, "36h11");
    get_parameter_or<double>("size", tag_edge_size, 2.0);
    get_parameter_or<int>("max_hamming", max_hamming, 0);

    // get tag names and IDs
    static const std::string tag_list_prefix = "tag_lists";
    auto parameters_and_prefixes = list_parameters({tag_list_prefix}, 10);
    for (const std::string &name : parameters_and_prefixes.names) {
        const int id = get_parameter(name).get_value<int>();
        tracked_tags[id] = name.substr(tag_list_prefix.size()+1, name.size());
    }

    get_parameter_or<bool>("z_up", z_up, false);

    if(!tag_create.count(tag_family)) {
        throw std::runtime_error("unsupported tag family: "+tag_family);
    }
    tf = tag_create.at(tag_family)();
    td = apriltag_detector_create();
    get_parameter_or<float>("decimate", td->quad_decimate, 1.0);
    get_parameter_or<float>("blur", td->quad_sigma, 0.0);
    get_parameter_or<int>("threads", td->nthreads, 1);
    get_parameter_or<int>("debug", td->debug, false);
    get_parameter_or<int>("refine-edges", td->refine_edges, true);
    get_parameter_or<int>("refine-decode", td->refine_decode, false);
    get_parameter_or<int>("refine-pose", td->refine_pose, false);
    apriltag_detector_add_family(td, tf);
}

AprilTag2Node::~AprilTag2Node() {
    apriltag_detector_destroy(td);
    tag_destroy.at(tag_family)(tf);
}

void AprilTag2Node::onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img) {
    // decode image
    image_u8_t* im = nullptr;

    if(msg_img->format=="jpeg" || msg_img->format=="jpg") {
        // convert jpeg data
        int err = 0;
        pjpeg_t* pj = pjpeg_create_from_buffer(msg_img->data.data(), msg_img->data.size(), 0, &err);
        if(pj==nullptr) {
            RCLCPP_ERROR(get_logger(), "pjpeg error");
            return;
        }

        im = pjpeg_to_u8_baseline(pj);

        pjpeg_destroy(pj);
    }
    else {
        RCLCPP_ERROR(get_logger(), "not supported: %s",  msg_img->format.c_str());
        return;
    }

    if (im==nullptr) {
        RCLCPP_ERROR(get_logger(), "could not load image");
        return;
    }

    // decode image
    zarray_t* detections = apriltag_detector_detect(td, im);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    tf2_msgs::msg::TFMessage tfs;

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        // ignore untracked tags
        if(tracked_tags.size()>0 && !tracked_tags.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming>max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.goodness = det->goodness;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double)*8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double)*9);
        msg_detections.detections.push_back(msg_detection);

        // 3D orientation and position
        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg_img->header;
        // set child frame name by generic tag name or configured tag name
        tf.child_frame_id = tracked_tags.size() ? tracked_tags.at(det->id) : std::string(det->family->name)+":"+std::to_string(det->id) ;
        getPose(*(det->H), tf.transform, z_up);

        tfs.transforms.push_back(tf);
    }

    pub_detections->publish(msg_detections);
    pub_tf->publish(tfs);

    apriltag_detections_destroy(detections);

    image_u8_destroy(im);
}

void AprilTag2Node::getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const bool z_up) {

    const Eigen::Map<const Mat3>Hm(H.data);

    // compute extrinsic camera parameter
    // https://dsp.stackexchange.com/a/2737/31703
    // H = K * T  =>  T = K^(-1) * H
    const Mat3 T = K.inverse() * Hm / Hm(2,2);
    Mat3 R;
    R.col(0) = T.col(0).normalized();
    R.col(1) = T.col(1).normalized();
    R.col(2) = R.col(0).cross(R.col(1));

    if(z_up) {
        // rotate by half rotation about x-axis
        R.col(1) *= -1;
        R.col(2) *= -1;
    }

    // the corner coordinates of the tag in the canonical frame are (+/-1, +/-1)
    // hence the scale is half of the edge size
    const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm())/2.0) * (tag_edge_size/2.0);

    const Eigen::Quaterniond q(R);

    t.translation.x = tt.x();
    t.translation.y = tt.y();
    t.translation.z = tt.z();
    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
}

std::map<std::string, apriltag_family_t *(*)(void)> AprilTag2Node::tag_create =
{
    {"16h5", tag16h5_create},
    {"25h7", tag25h7_create},
    {"25h9", tag25h9_create},
    {"36h10", tag36h10_create},
    {"36h11", tag36h11_create},
    {"36artoolkit", tag36artoolkit_create},
};

std::map<std::string, void (*)(apriltag_family_t*)> AprilTag2Node::tag_destroy =
{
    {"16h5", tag16h5_destroy},
    {"25h7", tag25h7_destroy},
    {"25h9", tag25h9_destroy},
    {"36h10", tag36h10_destroy},
    {"36h11", tag36h11_destroy},
    {"36artoolkit", tag36artoolkit_destroy},
};

CLASS_LOADER_REGISTER_CLASS(AprilTag2Node, rclcpp::Node)
