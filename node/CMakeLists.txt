cmake_minimum_required(VERSION 3.5)

project(apriltag2_node)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(Threads REQUIRED)

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
include_directories(${apriltag_INCLUDE_DIRS}/apriltag)
link_directories(${apriltag_LIBDIR})

include_directories(include)

add_library(AprilTag2Node SHARED src/AprilTag2Node.cpp)
ament_target_dependencies(AprilTag2Node rclcpp sensor_msgs apriltag_msgs class_loader tf2_msgs)
target_link_libraries(AprilTag2Node ${apriltag_LIBRARIES})
rclcpp_register_node_plugins(AprilTag2Node "AprilTag2Node")

add_executable(apriltag2_node src/apriltag2_node.cpp)
target_link_libraries(apriltag2_node AprilTag2Node)

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

install(TARGETS
    AprilTag2Node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS apriltag2_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY cfg/ DESTINATION share/${PROJECT_NAME}/cfg)

ament_package()
