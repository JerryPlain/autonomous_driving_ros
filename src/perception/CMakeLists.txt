cmake_minimum_required(VERSION 3.0.2)
project(perception)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  depth_image_proc
  image_transport
  nodelet
  octomap_ros
  octomap_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    depth_image_proc
    image_transport
    nodelet
    octomap_ros
    octomap_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(static_tf_broadcaster_node src/static_tf_broadcaster_node.cpp)
target_link_libraries(static_tf_broadcaster_node ${catkin_LIBRARIES})

add_executable(save_octomap_subscriber src/save_octomap_subscriber.cpp)
target_link_libraries(save_octomap_subscriber ${catkin_LIBRARIES})
