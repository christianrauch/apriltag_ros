# AprilTag 2 ROS2 Node

This ROS2 node uses the AprilTag 2 library to detect AprilTags in images and publish their pose, id and additional metadata.

For more information on AprilTag 2, the paper and the reference implementation: https://april.eecs.umich.edu/software/apriltag.html

## Topics

### Subscriptions:
- `/apriltag/image/compressed` (type: `sensor_msgs/CompressedImage`, format: `"jpeg"` or `"jpg"`)
- `/apriltag/image/camera_info` (type: `sensor_msgs/CameraInfo`)

### Publisher:
- `/tf` (type: `tf2_msgs/TFMessage`)
- `/apriltag/detections` (type: `apriltag_msgs/AprilTagDetectionArray`)

The camera intrinsics `K` in `CameraInfo` are used to compute the marker tag pose `T` from the homography `H`. The node sets `K` from the first `CameraInfo` message and unsubscribes after this.

The tag poses are published on the standard TF topic `/tf` with the header set to the image header and `child_frame_id` set to either `<tag_family>:<id>` (e.g. "36h11:0") or the frame name selected via configuration file. Additional information about detected tag is published as `AprilTagDetectionArray` message, which contains the original homography  matrix, the `goodness` and the `decision_margin` of the detection.

## Configuration

The node is configured via a yaml configurations file. For the complete ROS yaml parameter file syntax, see: https://github.com/ros2/rcl/tree/master/rcl_yaml_param_parser.

The file has the format:
```YAML
apriltag:                           # namespace
    apriltag2:                      # node name
        ros__parameters:
            # required
            family: <tag family>    # tag family name
            size: <tag edge size>   # tag edge size in meter
            z_up: true              # rotate about x-axis to have Z pointing upwards

            # (optional) tuning of detection
            max_hamming: 0          # maximum allowed hamming distance (corrected bits)
            decimate: 1.0           # decimate resolution for quad detection
            blur: 0.0               # sigma of Gaussian blur for quad detection
            refine-edges: 1         # snap to strong gradients
            threads: 1              # number of threads
            refine-decode: 0        # increase the number of detected tags
            refine-pose: 0          # increase the accuracy of the extracted pose
            debug: 0                # write additional debugging images to current working directory

            # (optional) list of tags
            tag_lists:
                <frame name>: <id>  # tag frame name and ID
                <frame name>: <id>
                [...]
```

The parameters `family` and `size` are required. `family` (string) defines the tag family for the detector and must be one of `16h5`, `25h7`, `25h9`, `36h10`, `36h11`, `36artoolkit`. `size` (float) is the tag edge size in meters, assuming square markers.

`tag_lists` is an optional list to map detected tag IDs to frame names. If it is provided, only the the listed tag IDs will be published, with `child_frame_id` set to the frame name. If not provided, the pose of all detected tags is published with a generic `child_frame_id`.

The remaining parameters are set to the their default values from the library. See `apriltag.h` for a more detailed description of their function.

See [tags_16h5_all.yaml](node/cfg/tags_16h5_all.yaml) for an example configuration that publishes all markers in the 16h5 family and [tags_16h5_filtered.yaml](node/cfg/tags_16h5_filtered.yaml) for filtering tags.

To run the node and load the configuration, pass the configuration file to `__params`:
```bash
ros2 run apriltag2_node apriltag2_node \
    /apriltag/image/compressed:=/camera/image/compressed \
    /apriltag/image/camera_info:=/camera/image/camera_info \
    __params:=`ros2 pkg prefix apriltag2_node`/share/apriltag2_node/cfg/tags_16h5_all.yaml
```