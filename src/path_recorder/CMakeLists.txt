cmake_minimum_required(VERSION 3.0.2)
project(path_recorder)

## 编译类型
add_compile_options(-std=c++14)

## 查找 catkin 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

## 包含路径
catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 可执行文件
add_executable(path_publisher_node
  src/path.cpp
  src/path_recorder.cpp
)

add_executable(show_path_node
  src/showpath.cpp
)

add_executable(recordpoints_node
  src/recordpoints.cpp
)

## 链接库
target_link_libraries(path_publisher_node ${catkin_LIBRARIES})
target_link_libraries(show_path_node ${catkin_LIBRARIES})
target_link_libraries(recordpoints_node ${catkin_LIBRARIES})


add_executable(showpoints src/showpoints.cpp)
target_link_libraries(showpoints ${catkin_LIBRARIES})

add_executable(csv_goal_publisher src/csv_goal_publisher.cpp)
target_link_libraries(csv_goal_publisher ${catkin_LIBRARIES})

