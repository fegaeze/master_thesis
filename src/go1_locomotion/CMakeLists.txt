cmake_minimum_required(VERSION 3.0.2)
project(go1_locomotion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    roscpp
    unitree_legged_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS unitree_legged_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(utils src/utils.cpp)
add_dependencies(utils unitree_legged_msgs_gencpp)

add_library(kinematics src/kinematics.cpp)

add_executable(go1_locomotion_main src/main.cpp)
target_link_libraries(go1_locomotion_main ${catkin_LIBRARIES} utils kinematics)