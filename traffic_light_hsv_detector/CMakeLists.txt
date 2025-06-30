cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_hsv_detector)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  image_transport
  cv_bridge
)

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    image_transport
    cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## 5) SegmentationProcessor Node
add_executable(segmentation_processor_node
  src/segmentation_processor_node.cpp
)
add_dependencies(segmentation_processor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(segmentation_processor_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## 6) ColorDetector Node
add_executable(color_detector_node
  src/color_detector_node.cpp
)
add_dependencies(color_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(color_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


