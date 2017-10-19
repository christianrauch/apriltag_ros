#include <AprilTag2Node.hpp>
#include <class_loader/class_loader_register_macro.h>

AprilTag2Node::AprilTag2Node() : Node("apriltag2") {
    sub_img = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",
        std::bind(&AprilTag2Node::onImage, this, std::placeholders::_1));
    pub_pose = this->create_publisher<geometry_msgs::msg::TransformStamped>("tag_pose");
    pub_detections = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections");

    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
}

AprilTag2Node::~AprilTag2Node() {
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
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

//            std::cout.precision(2);
//            std::cout << "id(" << det->family->d*det->family->d << "x" << det->family->h << "): " << det->id;
//            std::cout << ", goodness: " << det->goodness << std::fixed;
//            std::cout << ", margin: " << det->decision_margin << std::fixed;
//            std::cout << ", cx,cy: " << det->c[0] << " " << det->c[1] << std::fixed;
//            std::cout << std::endl;

        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.goodness = det->goodness;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre[0] = det->c[0];
        msg_detection.centre[1] = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double)*8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double)*9);

        msg_detections.detections.push_back(msg_detection);

        pub_detections->publish(msg_detections);
    }

    apriltag_detections_destroy(detections);

    image_u8_destroy(im);
}

CLASS_LOADER_REGISTER_CLASS(AprilTag2Node, rclcpp::Node)
