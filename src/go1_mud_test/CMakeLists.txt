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
  tf
  unitree_legged_msgs
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

add_service_files(
  FILES
  ActionService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  unitree_legged_msgs 
  message_runtime
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(rosInterfaceManager src/rosInterfaceManager.cpp)
add_dependencies(rosInterfaceManager unitree_legged_msgs_gencpp)
target_link_libraries(rosInterfaceManager ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_library(mudTestActions src/mudTestActions.cpp)
add_dependencies(mudTestActions unitree_legged_msgs_gencpp)
target_link_libraries(mudTestActions ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_library(actionServer src/actionServer.cpp)
add_dependencies(actionServer unitree_legged_msgs_gencpp)
target_link_libraries(actionServer ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_executable(go1_mud_test_node src/mudTestNode.cpp)
target_link_libraries(go1_mud_test_node ${catkin_LIBRARIES} ${EXTRA_LIBS} rosInterfaceManager mudTestActions actionServer)