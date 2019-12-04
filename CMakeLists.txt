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
find_package(lcm REQUIRED)
set (csm_LIBRARY_DIRS /home/yifan/calibration/csm/lib)
set (csm_INCLUDE_DIRS /home/yifan/calibration/csm/include)
set (csm_INCLUDE_DIRS2 /home/yifan/calibration/csm/src)

###########
## Build ##
###########
set(Thirdparty_DIR /home/yifan/cleanRobot/thirdparty/include)
set(lcmmsgs_DIR ${Thirdparty_DIR}/lcm_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS2}
  ${Thirdparty_DIR}
  ${lcmmsgs_DIR}
  include
)
message("lcm" ${lcm})

set(SOURCES  src/plicp_node.cpp src/plicp_wrapper.cpp)

add_executable(plicp_node ${SOURCES})

target_link_libraries(plicp_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${LCM_LIBRARY}
  lcm
  ${csm_LIBRARY_DIRS}/libcsm_eigen.a 
)