cmake_minimum_required(VERSION 3.5)

project(apriltag2_node)

set(CMAKE_CXX_STANDARD 14)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)

find_package(Threads REQUIRED)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
include_directories(${apriltag_INCLUDE_DIRS})
link_directories(${apriltag_LIBDIR})

add_executable(apriltag2_node src/apriltag2_node.cpp)
ament_target_dependencies(apriltag2_node
  geometry_msgs
  rclcpp
  sensor_msgs
  apriltag_msgs)
target_link_libraries(apriltag2_node ${apriltag_LIBRARIES} Threads::Threads)

install(TARGETS apriltag2_node DESTINATION lib/${PROJECT_NAME})

ament_package()
