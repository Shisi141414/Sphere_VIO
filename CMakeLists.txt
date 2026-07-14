cmake_minimum_required(VERSION 3.0.2)
project(sphere_vio)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_filters roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_inspector_node
  src/ros/sensor_inspector_node.cpp
  src/ros/sensor_inspector_node_main.cpp
)
add_dependencies(sensor_inspector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_inspector_node ${catkin_LIBRARIES})

install(TARGETS sensor_inspector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
