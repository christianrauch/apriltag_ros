# AprilTag ROS2 Node

This ROS2 node uses the AprilTag library to detect AprilTags in images and publish their pose, id and additional metadata.

For more information on AprilTag, the paper and the reference implementation: https://april.eecs.umich.edu/software/apriltag.html

## Topics

### Subscriptions:
The node subscribes via a `image_transport::CameraSubscriber` to `/apriltag/image`. The set of topic names depends on the type of image transport (parameter `image_transport`) selected (`raw` or `compressed`):
- `/apriltag/image` (`raw`, type: `sensor_msgs/Image`)
- `/apriltag/image/compressed` (`compressed`, type: `sensor_msgs/CompressedImage`)
- `/apriltag/camera_info` (type: `sensor_msgs/CameraInfo`)

### Publisher:
- `/tf` (type: `tf2_msgs/TFMessage`)
- `/apriltag/detections` (type: `apriltag_msgs/AprilTagDetectionArray`)

The camera intrinsics `K` in `CameraInfo` are used to compute the marker tag pose `T` from the homography `H`. The image and the camera intrinsics need to have the same timestamp.

The tag poses are published on the standard TF topic `/tf` with the header set to the image header and `child_frame_id` set to either `tag<family>:<id>` (e.g. "tag36h11:0") or the frame name selected via configuration file. Additional information about detected tags is published as `AprilTagDetectionArray` message, which contains the original homography  matrix, the `hamming` distance and the `decision_margin` of the detection.

## Configuration

The node is configured via a yaml configurations file. For the complete ROS yaml parameter file syntax, see: https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser.

The file has the format:
```YAML
apriltag:                           # namespace
    apriltag:                       # node name
        ros__parameters:
            # required
            image_transport: raw    # image format: "raw" or "compressed" (default: raw)
            family: <tag family>    # tag family name: 16h5, 25h9, 36h11 (default: 36h11)
            size: <tag edge size>   # tag edge size in meter (default: 2.0)
            z_up: true              # rotate about x-axis to have Z pointing upwards (default: false)

            # (optional) tuning of detection
            max_hamming: 0          # maximum allowed hamming distance (corrected bits)
            decimate: 1.0           # decimate resolution for quad detection
            blur: 0.0               # sigma of Gaussian blur for quad detection
            refine-edges: 1         # snap to strong gradients
            threads: 1              # number of threads
            debug: 0                # write additional debugging images to current working directory

            # (optional) list of tags
            tag_ids: [<id1>, <id2>, ...]            # tag IDs for which to publish transform
            tag_frames: [<frame1>, <frame2>, ...]   # optional frame names
            tag_sizes: [<size1>, <size1>, ...]      # optional tag-specific edge size
```

The parameters `family` and `size` are required. `family` (string) defines the tag family for the detector and must be one of `16h5`, `25h9`, `36h11`, `Circle21h7`, `Circle49h12`, `Custom48h12`, `Standard41h12`, `Standard52h13`. `size` (float) is the tag edge size in meters, assuming square markers.

Instead of publishing all tag poses, the list `tag_ids` can be used to only publish selected tag IDs. Each tag can have an associated child frame name in `tag_frames` and a tag specific size in `tag_sizes`. These lists must either have the same length as `tag_ids` or may be empty. In this case, a default frame name of the form `tag<family>:<id>` and the default tag edge size `size` will be used.

The remaining parameters are set to the their default values from the library. See `apriltag.h` for a more detailed description of their function.

See [tags_16h5_all.yaml](node/cfg/tags_16h5_all.yaml) for an example configuration that publishes all markers in the 16h5 family and [tags_16h5_filtered.yaml](node/cfg/tags_16h5_filtered.yaml) for filtering tags.

The composable node can be loaded into an already running component manager with a configuration file, by passing the configuration file path to `__params`:
```bash
ros2 component load /ComponentManager apriltag_ros AprilTagNode \
    -r /apriltag/image:=/camera/image \
    -r /apriltag/camera_info:=/camera/camera_info \
    -r __params:=`ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_16h5_all.yaml
```
A component manager can be started via:
```bash
ros2 run rclcpp_components component_container
```

Alternatively, a launch file can be used to start a component manager and load the composable node with configuration:
```bash
ros2 launch apriltag_ros tag_16h5_all.launch.py
```
