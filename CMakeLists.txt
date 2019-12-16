cmake_minimum_required(VERSION 2.8.3)
project(plicp_node)

find_package(Eigen3)
find_package(lcm REQUIRED)

SET(CSM_SRC 
    src/csm/clustering.cpp
    src/csm/laser_data.cpp
    src/csm/logging.cpp
    src/csm/math_utils_gsl.cpp
    src/csm/math_utils.cpp
    src/csm/orientation.cpp
    src/csm/sm_config.cpp
    src/csm/sm_options.cpp
    src/csm/utils.cpp
)

SET(EGSL_SRC
    src/egsl/egsl.cpp
    src/egsl/egsl_conversions.cpp
    src/egsl/egsl_misc.cpp
    src/egsl/egsl_ops.cpp
)

SET(GPC_SRC
    src/gpc/gpc.cpp
    src/gpc/gpc_utils.cpp
)

SET(ICP_SRC
    src/icp/icp_corr_dumb.cpp
    src/icp/icp_corr_tricks.cpp
    src/icp/icp_covariance.cpp
    src/icp/icp_debug.cpp
    src/icp/icp_loop.cpp
    src/icp/icp_outliers.cpp
    src/icp/icp.cpp
)

ADD_LIBRARY(cms_plicp
  ${CSM_SRC}
  ${EGSL_SRC}
  ${GPC_SRC}
  ${ICP_SRC}
)

set(Thirdparty_DIR /home/yifan/cleanRobot/thirdparty/include)
set(lcmmsgs_DIR ${Thirdparty_DIR}/lcm_msgs)

include_directories(
  ${Eigen3_INCLUDE_DIRS}
  ${Thirdparty_DIR}
  ${lcmmsgs_DIR}
  include
)

set(SOURCES  src/plicp_node.cpp src/plicp_wrapper.cpp)

add_executable(plicp_node ${SOURCES})

target_link_libraries(plicp_node
  ${Eigen3_LIBRARIES}
  ${LCM_LIBRARY}
  lcm
  cms_plicp
)