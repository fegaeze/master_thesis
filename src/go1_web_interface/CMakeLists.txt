cmake_minimum_required(VERSION 3.0.2)
project(go1_web_interface)

find_package(catkin REQUIRED COMPONENTS
  rosbridge_server
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES go1_web_interface
#  CATKIN_DEPENDS roslibjs
#  DEPENDS system_lib
)
