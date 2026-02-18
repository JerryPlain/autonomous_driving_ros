cmake_minimum_required(VERSION 3.0.2)
project(dummy_controller)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  simulation
  std_msgs
  tf2
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    simulation
    std_msgs
    tf2
    tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dummy_controller_node src/dummy_controller_node.cpp)
target_link_libraries(dummy_controller_node ${catkin_LIBRARIES})
