#include <AprilTag2Node.hpp>
#include <class_loader/class_loader_register_macro.h>

AprilTag2Node::AprilTag2Node() : Node("apriltag2") {
    sub_img = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",
        std::bind(&AprilTag2Node::onImage, this, std::placeholders::_1));
    pub_pose = this->create_publisher<geometry_msgs::msg::TransformStamped>("tf");
    pub_detections = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections");

    // get single camera info message
    sub_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "image/camera_info",
        [this](sensor_msgs::msg::CameraInfo::UniquePtr info){
            std::cout << "got camera parameters" << std::endl;
            std::memcpy(K.data(), info->k.data(), 9*sizeof(double));
            // delete subscription
            sub_info.reset();
        }
    );

    get_parameter_or<std::string>("family", tag_family, "16h5");

    // default tag size (scale 1)
    tag_size = 2.0;

    tf = tag_create.at(tag_family)();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
}

AprilTag2Node::~AprilTag2Node() {
    apriltag_detector_destroy(td);
    tag_destroy.at(tag_family)(tf);
}

void AprilTag2Node::onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img) {
    // decode image
    image_u8_t* im = NULL;

    if(msg_img->format=="jpeg") {
        // convert jpeg data
        int err = 0;
        pjpeg_t* pj = pjpeg_create_from_buffer(msg_img->data.data(), msg_img->data.size(), 0, &err);
        if(pj==NULL) {
            std::cerr << "pjpeg error " << err << std::endl;
            return;
        }

        im = pjpeg_to_u8_baseline(pj);

        pjpeg_destroy(pj);
    }
    else {
        std::cerr << "not supported: " << msg_img->format << std::endl;
        return;
    }

    if (im == NULL) {
        std::cerr << "could not load image" << std::endl;
        return;
    }

    // decode image
    zarray_t* detections = apriltag_detector_detect(td, im);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

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
        tf.child_frame_id = std::string(det->family->name)+":"+std::to_string(det->id);
        getPose(*(det->H), tf.transform);

        pub_pose->publish(tf);
    }

    pub_detections->publish(msg_detections);

    apriltag_detections_destroy(detections);

    image_u8_destroy(im);
}

void AprilTag2Node::getPose(const matd_t& H, geometry_msgs::msg::Transform& t) {

    const Eigen::Map<const Mat3>Hm(H.data);

    // compute extrinsic camera parameter
    // https://dsp.stackexchange.com/a/2737/31703
    // H = K * T  =>  T = K^(-1) * H
    const Mat3 T = K.inverse() * Hm / Hm(2,2);
    Mat3 R;
    R.col(0) = T.col(0).normalized();
    R.col(1) = T.col(1).normalized();
    R.col(2) = R.col(0).cross(R.col(1));

    const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm())/2.0) * (tag_size/2.0);

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
