cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  image_transport
  cv_bridge
  message_filters
)

add_message_files(
  FILES
    TrafficLightBBox.msg
    TrafficLightBBoxArray.msg  
    TrafficLightColor.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
    image_transport
    cv_bridge
    message_filters
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(color_detector_node src/color_detector_node.cpp)
add_dependencies(color_detector_node ${PROJECT_NAME}_gencpp)
target_link_libraries(color_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(semantic_depth_median_node src/semantic_depth_median_node.cpp)
add_dependencies(semantic_depth_median_node ${PROJECT_NAME}_gencpp)
target_link_libraries(semantic_depth_median_node
  ${catkin_LIBRARIES}
)

add_executable(traffic_light_detection src/traffic_light_detection.cpp)
add_dependencies(traffic_light_detection ${PROJECT_NAME}_gencpp)
target_link_libraries(traffic_light_detection
  ${catkin_LIBRARIES}
)