cmake_minimum_required(VERSION 3.11.0)
project(airsim_ros_pkgs)

set(CMAKE_CXX_COMPILER /usr/bin/clang++-5.0)

set(AIRSIM_ROOT /projectsym/AirSim_wsl/AirSim)

LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules")
INCLUDE("${AIRSIM_ROOT}/cmake/cmake-modules/CommonSetup.cmake")
CommonSetup()

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) #name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# IncludeEigen()
include_directories(${AIRSIM_ROOT}/AirLib/deps/eigen3)

include_directories(
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

add_compile_options(-std=c++11)
# find_package(Eigen3 REQUIRED) 
# find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  image_transport
  cv_bridge
  tf
)

catkin_package( CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy 
  std_msgs
)

include_directories(
  include
 ${catkin_INCLUDE_DIRS}
 ) 

link_directories(${AIRSIM_ROOT}/build_debug/output/lib)

add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node AirLib ${RPC_LIB} MavLinkCom)
# target_link_libraries(airsim_node ${CMAKE_THREAD_LIBS_INIT})
