cmake_minimum_required(VERSION 3.0.2)
project(go1_mud_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    roscpp
    unitree_legged_msgs
	tf
  	tf2_ros
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
)

add_library(hardware src/hardware.cpp)
add_dependencies(hardware unitree_legged_msgs_gencpp)
target_link_libraries(hardware ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_library(mudTest src/mudTest.cpp)
target_link_libraries(mudTest ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_executable(go1_mud_test_main src/mudTestNode.cpp)
target_link_libraries(go1_mud_test_main ${catkin_LIBRARIES} hardware mudTest)