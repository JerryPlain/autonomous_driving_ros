cmake_minimum_required(VERSION 3.0.2)
project(decision_making)
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  simulation
  image_transport
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  TrafficLightColor.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES decision_making_node
#  CATKIN_DEPENDS mav_msgs roscpp std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS simulation roscpp std_msgs image_transport
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(decision_making_node src/decision_making_node.cpp)
target_link_libraries(decision_making_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

