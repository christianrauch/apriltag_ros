#include <AprilTagNode.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// default tag families
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>

#include <Eigen/Dense>

// create and delete functions for default tags
#define TAG_CREATE(name) { #name, tag##name##_create },
#define TAG_DESTROY(name) { #name, tag##name##_destroy },

const std::map<std::string, apriltag_family_t *(*)(void)> AprilTagNode::tag_create =
{
    TAG_CREATE(36h11)
    TAG_CREATE(25h9)
    TAG_CREATE(16h5)
    TAG_CREATE(Circle21h7)
    TAG_CREATE(Circle49h12)
    TAG_CREATE(Custom48h12)
    TAG_CREATE(Standard41h12)
    TAG_CREATE(Standard52h13)
};

const std::map<std::string, void (*)(apriltag_family_t*)> AprilTagNode::tag_destroy =
{
    TAG_DESTROY(36h11)
    TAG_DESTROY(25h9)
    TAG_DESTROY(16h5)
    TAG_DESTROY(Circle21h7)
    TAG_DESTROY(Circle49h12)
    TAG_DESTROY(Custom48h12)
    TAG_DESTROY(Standard41h12)
    TAG_DESTROY(Standard52h13)
};

AprilTagNode::AprilTagNode(rclcpp::NodeOptions options)
  : Node("apriltag", "apriltag", options.use_intra_process_comms(true)),
    // topics
    pub_tf(create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100))),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    sub_cam(image_transport::create_camera_subscription(this, "image", std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2), declare_parameter<std::string>("image_transport", "raw"), rmw_qos_profile_sensor_data)),
    // parameter
    tag_family(declare_parameter<std::string>("family", "36h11")),
    tag_edge_size(declare_parameter<double>("size", 2.0)),
    max_hamming(declare_parameter<int>("max_hamming", 0)),
    z_up(declare_parameter<bool>("z_up", false)),
    td(apriltag_detector_create())
{
    td->quad_decimate = declare_parameter<float>("decimate", 1.0);
    td->quad_sigma =    declare_parameter<float>("blur", 0.0);
    td->nthreads =      declare_parameter<int>("threads", 1);
    td->debug =         declare_parameter<int>("debug", false);
    td->refine_edges =  declare_parameter<int>("refine-edges", true);

    // get tag names, IDs and sizes
    const auto ids = declare_parameter<std::vector<int64_t>>("tag_ids", {});
    const auto frames = declare_parameter<std::vector<std::string>>("tag_frames", {});

    if(!frames.empty()) {
        if(ids.size()!=frames.size()) {
            throw std::runtime_error("Number of tag ids ("+std::to_string(ids.size())+") and frames ("+std::to_string(frames.size())+") mismatch!");
        }
        for(size_t i = 0; i<ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    const auto sizes = declare_parameter<std::vector<double>>("tag_sizes", {});
    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size()!=sizes.size()) {
            throw std::runtime_error("Number of tag ids ("+std::to_string(ids.size())+") and sizes ("+std::to_string(sizes.size())+") mismatch!");
        }
        for(size_t i = 0; i<ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if(tag_create.count(tag_family)) {
        tf = tag_create.at(tag_family)();
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: "+tag_family);
    }
}

AprilTagNode::~AprilTagNode() {
    apriltag_detector_destroy(td);
    tag_destroy.at(tag_family)(tf);
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) {
    // copy camera intrinsics
    std::memcpy(K.data(), msg_ci->k.data(), 9*sizeof(double));

    // convert to 8bit monochrome image
    const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

    image_u8_t im = {
        .width = img_uint8.cols,
        .height = img_uint8.rows,
        .stride = img_uint8.cols,
        .buf = img_uint8.data
    };

    // detect tags
    zarray_t* detections = apriltag_detector_detect(td, &im);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    tf2_msgs::msg::TFMessage tfs;

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming>max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
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
        tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name)+":"+std::to_string(det->id);
        getPose(*(det->H), tf.transform, tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size);

        tfs.transforms.push_back(tf);
    }

    pub_detections->publish(msg_detections);
    pub_tf->publish(tfs);

    apriltag_detections_destroy(detections);
}

void AprilTagNode::getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const double size) const {

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
    const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm())/2.0) * (size/2.0);

    const Eigen::Quaterniond q(R);

    t.translation.x = tt.x();
    t.translation.y = tt.y();
    t.translation.z = tt.z();
    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)
