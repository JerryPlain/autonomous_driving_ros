cmake_minimum_required(VERSION 3.0.2)
project(perception)

## Enable C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  octomap_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  nodelet
  depth_image_proc
)

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    octomap_ros
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    nodelet
    depth_image_proc
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 添加你新的 broadcaster 节点
add_executable(static_tf_broadcaster_node src/static_tf_broadcaster_node.cpp)
target_link_libraries(static_tf_broadcaster_node ${catkin_LIBRARIES})

add_executable(save_octomap_subscriber src/save_octomap_subscriber.cpp)
target_link_libraries(save_octomap_subscriber ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})


