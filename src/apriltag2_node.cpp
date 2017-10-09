// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// apriltag
#include <apriltag.h>
#include <tag36h11.h>
#include <common/pjpeg.h>

class AprilTag2Node : public rclcpp::Node {
public:
    AprilTag2Node() : Node("apriltag2") {
        sub_img = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",
            std::bind(&AprilTag2Node::onImage, this, std::placeholders::_1));
        pub_pose = this->create_publisher<geometry_msgs::msg::TransformStamped>("tag_pose");

        tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);
    }

    ~AprilTag2Node() {
        apriltag_detector_destroy(td);
        tag36h11_destroy(tf);
    }

private:
    apriltag_family_t* tf;
    apriltag_detector_t* td;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_pose;

    void onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg_img) {
        // decode image
        image_u8_t* im = NULL;

        if(msg_img->format=="jpeg") {
            // convert jpeg data
            int err = 0;
            pjpeg_t* pj = pjpeg_create_from_buffer(msg_img->data.data(), msg_img->data.size(), 0, &err);
            if(pj==NULL) {
                printf("pjpeg error %d\n", err);
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
            printf("could not load image\n");
            return;
        }

        // decode image
        zarray_t* detections = apriltag_detector_detect(td, im);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n",
                       i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);

            // TODO: do the projection
            const matd_t* H = det->H;
        }

        apriltag_detections_destroy(detections);

        image_u8_destroy(im);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTag2Node>());
    rclcpp::shutdown();
    return 0;
}