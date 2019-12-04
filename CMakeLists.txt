cmake_minimum_required(VERSION 2.8.3)
project(plicp_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)
find_package(Eigen3)
set (csm_LIBRARY_DIRS /home/yifan/calibration/csm/lib)
set (csm_INCLUDE_DIRS /home/yifan/calibration/csm/include)
set (csm_INCLUDE_DIRS2 /home/yifan/calibration/csm/src)

message("csm_include" ${csm_INCLUDE_DIRS})

###########
## Build ##
###########
# set(Thirdparty_DIR $ENV{HOME}/cleanRobot/thirdparty)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${Thirdparty_DIR}/CMakeModules")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -Wno-deprecated-declarations")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS2}
  include
)

set(SOURCES  src/plicp_node.cpp src/plicp_wrapper.cpp)

add_executable(plicp_node ${SOURCES})

target_link_libraries(plicp_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${csm_LIBRARY_DIRS}/libcsm_eigen.a 
)