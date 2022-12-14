cmake_minimum_required(VERSION 3.5)

project(apriltag_ros)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic)
  add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(Threads REQUIRED)

find_package(apriltag 3.2 REQUIRED)

add_library(utils src/tag_functions.cpp)
target_link_libraries(utils apriltag::apriltag)
set_property(TARGET utils PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(AprilTagNode SHARED src/AprilTagNode.cpp)
ament_target_dependencies(AprilTagNode rclcpp rclcpp_components sensor_msgs apriltag_msgs tf2_ros image_transport cv_bridge)
target_link_libraries(AprilTagNode apriltag::apriltag utils)
rclcpp_components_register_node(AprilTagNode PLUGIN "AprilTagNode" EXECUTABLE "apriltag_node")

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

install(TARGETS AprilTagNode
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY cfg/ DESTINATION share/${PROJECT_NAME}/cfg)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
