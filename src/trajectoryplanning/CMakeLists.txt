cmake_minimum_required(VERSION 3.0.2)
project(trajectoryplanning)

## 编译选项
add_compile_options(-std=c++11)

## 找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  octomap_ros
  octomap_msgs
)

## catkin 配置
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs tf tf2 tf2_ros octomap_ros octomap_msgs
)

## include 路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 如果有 C++ 节点（示例）
##add_executable(save_octomap_subscriber src/save_octomap_subscriber.cpp)
##target_link_libraries(save_octomap_subscriber ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

## 安装 Python 脚本（比如自动发目标）
##catkin_install_python(PROGRAMS
##  scripts/goal_publisher.py
## DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##)

## 安装 launch 文件
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 安装 config 文件
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 安装 maps 文件
install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(multi_goals_sender src/multi_goals_sender.cpp)
target_link_libraries(multi_goals_sender ${catkin_LIBRARIES} yaml-cpp)


