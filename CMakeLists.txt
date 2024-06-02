cmake_minimum_required(VERSION 3.5)

project(apriltag_ros)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic)
  add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()

option(ASAN "use AddressSanitizer to detect memory issues" OFF)

if(ASAN)
    set(ASAN_FLAGS "\
        -fsanitize=address \
        -fsanitize=bool \
        -fsanitize=bounds \
        -fsanitize=enum \
        -fsanitize=float-cast-overflow \
        -fsanitize=float-divide-by-zero \
        -fsanitize=nonnull-attribute \
        -fsanitize=returns-nonnull-attribute \
        -fsanitize=signed-integer-overflow \
        -fsanitize=undefined \
        -fsanitize=vla-bound \
        -fno-sanitize=alignment \
        -fsanitize=leak \
        -fsanitize=object-size \
    ")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)
find_package(apriltag 3.2 REQUIRED)

if(cv_bridge_VERSION VERSION_GREATER_EQUAL 3.3.0)
    add_compile_definitions(cv_bridge_HPP)
endif()


# database of tag functions
add_library(tags STATIC
    src/tag_functions.cpp
)
target_link_libraries(tags
    apriltag::apriltag
)
set_property(TARGET tags PROPERTY
    POSITION_INDEPENDENT_CODE ON
)


# conversion functions as template specialisation
add_library(conversion STATIC
    src/conversion.cpp
)
target_link_libraries(conversion
    apriltag::apriltag
    Eigen3::Eigen
    opencv_core
)
ament_target_dependencies(conversion
    geometry_msgs
    tf2
)
set_property(TARGET conversion PROPERTY
    POSITION_INDEPENDENT_CODE ON
)


# pose estimation methods
add_library(pose_estimation STATIC
    src/pose_estimation.cpp
)
target_link_libraries(pose_estimation
    apriltag::apriltag
    Eigen3::Eigen
    opencv_calib3d
    conversion
)
ament_target_dependencies(pose_estimation
    geometry_msgs
    tf2
)
set_property(TARGET pose_estimation PROPERTY
    POSITION_INDEPENDENT_CODE ON
)


# composable node
add_library(AprilTagNode SHARED
    src/AprilTagNode.cpp
)
ament_target_dependencies(AprilTagNode
    rclcpp
    rclcpp_components
    sensor_msgs
    apriltag_msgs
    tf2_ros
    image_transport
    cv_bridge
)
target_link_libraries(AprilTagNode
    apriltag::apriltag
    tags
    pose_estimation
)
rclcpp_components_register_node(AprilTagNode
    PLUGIN "AprilTagNode"
    EXECUTABLE "apriltag_node"
)

ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})

install(TARGETS AprilTagNode
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY cfg
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
