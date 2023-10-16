cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

set(CMAKE_CXX_FLAGS "-O3 -fPIC -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  unitree_legged_msgs
)

catkin_package(
  CATKIN_DEPENDS unitree_legged_sdk
)

set(EXTRA_LIBS -pthread ${unitree_legged_sdk_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${unitree_legged_sdk_INCLUDE_DIRS}
)

add_executable(ros_udp src/exe/ros_udp.cpp)
target_link_libraries(ros_udp ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
