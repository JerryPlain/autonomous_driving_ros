cmake_minimum_required(VERSION 3.0.2)
project(decision_making)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  simulation
  image_transport
  cv_bridge
  traffic_light_detector   
  message_runtime          
)

catkin_package(
  CATKIN_DEPENDS
    simulation
    roscpp
    std_msgs
    image_transport
    traffic_light_detector  
    message_runtime         
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(decision_making_node src/decision_making_node.cpp)
target_link_libraries(decision_making_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

