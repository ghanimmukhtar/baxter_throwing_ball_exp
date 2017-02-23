cmake_minimum_required(VERSION 2.8.3)
project(baxter_throwing_ball_exp)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface 
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES baxter_throwing_ball_exp
#  CATKIN_DEPENDS roscpp rospy std_msgs tf trajectory_msgs
#  DEPENDS system_lib
)

include_directories(include /usr/local/include ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

file ( GLOB_RECURSE all_sources src/lib/*.cpp )

add_executable(throw_ball_naive src/throw_ball_naive.cpp ${all_sources} src/parameters.hpp)
target_link_libraries(throw_ball_naive ${catkin_LIBRARIES})
