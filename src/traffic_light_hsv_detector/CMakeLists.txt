cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_hsv_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  image_transport
  cv_bridge
  message_filters
  geometry_msgs
)

add_message_files(
  FILES
  TrafficLight.msg
  TrafficLightArray.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    image_transport
    cv_bridge
    message_filters
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
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

add_executable(data_fusion src/data_fusion.cpp)
add_dependencies(data_fusion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(data_fusion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(semantic_depth_median_node src/semantic_depth_median_node.cpp)
add_dependencies(semantic_depth_median_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_depth_median_node ${catkin_LIBRARIES})

add_executable(traffic_light_detection src/traffic_light_detection.cpp)
add_dependencies(traffic_light_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traffic_light_detection ${catkin_LIBRARIES})

