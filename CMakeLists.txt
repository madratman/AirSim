cmake_minimum_required(VERSION 3.11.0)
project(airsim_ros_pkgs)

# set(CMAKE_CXX_COMPILER /usr/bin/clang++-5.0)

set(AIRSIM_ROOT /projectsym/AirSim_wsl/AirSim)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules")
INCLUDE("${AIRSIM_ROOT}/cmake/cmake-modules/CommonSetup.cmake")
CommonSetup()

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) #name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# find_package(Eigen3 REQUIRED) 
# find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
)

catkin_package( CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
) 

include_directories(${AIRSIM_ROOT}/AirLib/deps/eigen3)

include_directories(
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

link_directories(${AIRSIM_ROOT}/build_debug/output/lib)

add_library(lib_airsim_ros_wrapper src/airsim_ros_wrapper.cpp)
add_dependencies(lib_airsim_ros_wrapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(lib_airsim_ros_wrapper ${catkin_LIBRARIES} AirLib ${RPC_LIB} MavLinkCom)

add_executable(airsim_node src/airsim_node.cpp)
add_dependencies(airsim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_node ${catkin_LIBRARIES} lib_airsim_ros_wrapper)