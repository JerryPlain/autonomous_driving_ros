cmake_minimum_required(VERSION 3.0.2)
project(decision_making)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  traffic_light_detector
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    traffic_light_detector
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(decision_making_node src/decision_making_node.cpp)
target_link_libraries(decision_making_node ${catkin_LIBRARIES})
