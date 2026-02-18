cmake_minimum_required(VERSION 3.0.2)
project(path_recorder)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_publisher_node
  src/path.cpp
  src/path_recorder.cpp
)

target_link_libraries(path_publisher_node ${catkin_LIBRARIES})

add_executable(show_path_node src/showpath.cpp)
target_link_libraries(show_path_node ${catkin_LIBRARIES})

add_executable(recordpoints_node src/recordpoints.cpp)
target_link_libraries(recordpoints_node ${catkin_LIBRARIES})

add_executable(showpoints src/showpoints.cpp)
target_link_libraries(showpoints ${catkin_LIBRARIES})

add_executable(csv_goal_publisher src/csv_goal_publisher.cpp)
target_link_libraries(csv_goal_publisher ${catkin_LIBRARIES})
