cmake_minimum_required(VERSION 3.0.2)
project(go1_mud_test)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp
  controller_manager
  joint_state_controller
  gazebo_ros
  go1_web_interface
  message_generation
  robot_state_publisher
  roscpp
  rospy
  tf2_geometry_msgs
  tf2_ros
  unitree_legged_msgs
)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  ActionService.srv
  ControllerService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  unitree_legged_msgs
  message_runtime
)

catkin_install_python(PROGRAMS
  scripts/force_sensor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(actionServiceManager src/actionServiceManager.cpp)
add_dependencies(actionServiceManager ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(controllerServiceManager src/controllerServiceManager.cpp)
add_dependencies(controllerServiceManager ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(robotActionController src/robotActionController.cpp)
add_dependencies(robotActionController ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_library(robotActions src/robotActions.cpp)
add_dependencies(robotActions ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robotActions robotActionController)

add_library(rosInterfaceManager src/rosInterfaceManager.cpp)
add_dependencies(rosInterfaceManager ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(go1_mud_test_node src/mudTestNode.cpp)
add_dependencies(go1_mud_test_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(go1_mud_test_node 
  ${catkin_LIBRARIES} 
  actionServiceManager
  controllerServiceManager 
  robotActions 
  rosInterfaceManager
)
