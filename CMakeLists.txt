cmake_minimum_required(VERSION 3.10.1...3.23.0)
project(go1_mpc_control)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)
# robot_type
#add_definitions(-DUSE_GO1)

#check environmental variables
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  unitree_legged_msgs
)

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rc_sim_oub
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/go1_mpc_control
  ${catkin_INCLUDE_DIRS}
)

# to find osqp.h
include_directories(/usr/local/include/osqp)
find_package(OsqpEigen REQUIRED)

# add unitree hardware library, these flags must be correctly set
include_directories(
  $ENV{UNITREE_LEGGED_SDK_PATH}/include
)

link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

# Declare cpp libraries
# common robot controller every type of robots need
add_library(go1_lib
  src/robot/go1_kinematics.cpp
  src/robot/go1_robot_control.cpp
  src/utils/convex_mpc.cpp
  src/utils/go1_basicEKF.cpp
  src/utils/utils.cpp
)
target_link_libraries(go1_lib ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

# lib of different types of robots
add_library(gazebo_go1_lib
  src/robot/gazebo_go1_ros.cpp
)

# all robots depends on the a1_lib
target_link_libraries(gazebo_go1_lib go1_lib)

# Declare a cpp executable for gazebo robot
add_executable(gazebo_go1_ctrl src/main.cpp)
target_link_libraries(gazebo_go1_ctrl
  gazebo_go1_lib
  ${catkin_LIBRARIES}
  OsqpEigen::OsqpEigen
)

# tests
# Declare a cpp executable

# add_executable(test_rotation src/test/test_rotation.cpp
#   src/utils/utils.cpp
# )
# 
# add_executable(test_bezier src/test/test_bezier.cpp
#   src/utils/utils.cpp
# )
# 
# add_executable(test_mpc src/test/test_mpc.cpp)
# target_link_libraries(test_mpc
#   go1_lib
#   ${catkin_LIBRARIES}
#   OsqpEigen::OsqpEigen
# )
# 
# add_executable(test_unitree_legged_msgs src/test/test_unitree_msgs.cpp)
# target_link_libraries(test_unitree_legged_msgs
#   ${catkin_LIBRARIES}
# )