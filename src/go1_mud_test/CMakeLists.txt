cmake_minimum_required(VERSION 3.0.2)
project(go1_mud_test)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp
  controller_manager
  joint_state_controller
  gazebo_ros
  go1_web_interface
  robot_state_publisher
  roscpp
  tf
  unitree_legged_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS unitree_legged_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

add_library(controller src/controller.cpp)
add_dependencies(controller unitree_legged_msgs_gencpp)
target_link_libraries(controller ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_library(mudTestActions src/mudTestActions.cpp)
add_dependencies(mudTestActions unitree_legged_msgs_gencpp)
target_link_libraries(mudTestActions ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_library(mudTestConditions src/mudTestConditions.cpp)
add_dependencies(mudTestConditions unitree_legged_msgs_gencpp)
target_link_libraries(mudTestConditions ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_executable(go1_mud_test_main src/mudTestNode.cpp)
target_link_libraries(go1_mud_test_main ${catkin_LIBRARIES} ${EXTRA_LIBS} controller mudTestActions mudTestConditions)