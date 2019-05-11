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

std::map<std::string, apriltag_family_t *(*)(void)> AprilTagNode::tag_create =
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

std::map<std::string, void (*)(apriltag_family_t*)> AprilTagNode::tag_destroy =
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

AprilTagNode::AprilTagNode(rclcpp::NodeOptions options) : Node("apriltag", "apriltag", options.use_intra_process_comms(true)) {
    pub_tf = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
    pub_detections = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1));

    // declare parameters
    declare_parameter<std::string>("image_transport", "raw");
    declare_parameter<std::string>("family", "36h11");
    declare_parameter<double>("size", 2.0);
    declare_parameter<int>("max_hamming", 0);
    declare_parameter<bool>("z_up", false);
    declare_parameter<float>("decimate", 1.0);
    declare_parameter<float>("blur", 0.0);
    declare_parameter<int>("threads", 1);
    declare_parameter<int>("debug", false);
    declare_parameter<int>("refine-edges", true);

    std::string image_transport;
    get_parameter<std::string>("image_transport", image_transport);

    sub_cam = image_transport::create_camera_subscription(this, "image", std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2), image_transport, rmw_qos_profile_sensor_data);

    get_parameter<std::string>("family", tag_family);
    get_parameter<double>("size", tag_edge_size);
    get_parameter<int>("max_hamming", max_hamming);

    // get tag names and IDs
    static const std::string tag_list_prefix = "tag_lists";
    auto parameters_and_prefixes = list_parameters({tag_list_prefix}, 10);
    for (const std::string &name : parameters_and_prefixes.names) {
        const int id = get_parameter(name).get_value<int>();
        tracked_tags[id] = name.substr(tag_list_prefix.size()+1, name.size());
    }

    get_parameter<bool>("z_up", z_up);

    if(!tag_create.count(tag_family)) {
        throw std::runtime_error("unsupported tag family: "+tag_family);
    }
    tf = tag_create.at(tag_family)();
    td = apriltag_detector_create();
    get_parameter<float>("decimate", td->quad_decimate);
    get_parameter<float>("blur", td->quad_sigma);
    get_parameter<int>("threads", td->nthreads);
    get_parameter<int>("debug", td->debug);
    get_parameter<int>("refine-edges", td->refine_edges);
    apriltag_detector_add_family(td, tf);
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
        if(tracked_tags.size()>0 && !tracked_tags.count(det->id)) { continue; }

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
        tf.child_frame_id = tracked_tags.size() ? tracked_tags.at(det->id) : std::string(det->family->name)+":"+std::to_string(det->id) ;
        getPose(*(det->H), tf.transform, z_up);

        tfs.transforms.push_back(tf);
    }

    pub_detections->publish(msg_detections);
    pub_tf->publish(tfs);

    apriltag_detections_destroy(detections);
}

void AprilTagNode::getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const bool z_up) {

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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)
