cmake_minimum_required(VERSION 3.5)

project(apriltag_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(Threads REQUIRED)

find_package(apriltag REQUIRED)

include_directories(include)

add_library(AprilTagNode SHARED src/AprilTagNode.cpp)
ament_target_dependencies(AprilTagNode rclcpp rclcpp_components sensor_msgs apriltag_msgs tf2_msgs image_transport cv_bridge)
target_link_libraries(AprilTagNode apriltag)
rclcpp_components_register_nodes(AprilTagNode "AprilTagNode")

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

install(TARGETS AprilTagNode
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY cfg/ DESTINATION share/${PROJECT_NAME}/cfg)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
