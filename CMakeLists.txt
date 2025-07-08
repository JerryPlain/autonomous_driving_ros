cmake_minimum_required(VERSION 3.0.2)
project(waypoint_logger)

## 编译选项（可选）
add_compile_options(-std=c++11)

## 查找catkin依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

## 声明catkin包
catkin_package()

## 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 添加可执行文件并链接库
add_executable(waypoint_logger_node src/waypoint_logger_node.cpp)
target_link_libraries(waypoint_logger_node ${catkin_LIBRARIES})
add_executable(show_waypoints_node src/show_waypoints_node.cpp)
target_link_libraries(show_waypoints_node ${catkin_LIBRARIES})
add_executable(path_publisher_node src/path_publisher_node.cpp)
target_link_libraries(path_publisher_node ${catkin_LIBRARIES})

