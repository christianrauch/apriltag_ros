import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect all 36h11 tags
cfg_36h11 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.162,
    "max_hamming": 0,
    "z_up": True
}

def generate_launch_description():
    # 'usb_cam' node from https://github.com/ros-drivers/usb_cam.git
    cam_node = ComposableNode(
        namespace='camera',
        package='usb_cam', plugin='usb_cam::UsbCamNode',
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    tag_node = ComposableNode(
        name='apriltag_36h11',
        namespace='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/apriltag/image_rect", "/camera/image_raw"),
            ("/apriltag/camera_info", "/camera/camera_info"),
        ],
        parameters=[cfg_36h11],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[cam_node, tag_node],
        output='screen'
    )

    return launch.LaunchDescription([container])
