cmake_minimum_required(VERSION 3.11.0)
project(airsim_ros_pkgs)

# set(AIRSIM_ROOT /projectsym/AirSim_wsl/AirSim)
set(AIRSIM_ROOT /home/madratman/projects/AirSim)
# set(CMAKE_CXX_COMPILER /usr/bin/clang++-5.0)
# set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lc++ -lc++abi") #clang

## following 3 lines are not really needed (but wait they should be?)
# LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules")
# INCLUDE("${AIRSIM_ROOT}/cmake/cmake-modules/CommonSetup.cmake")
# CommonSetup()

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) #name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

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

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

link_directories(${AIRSIM_ROOT}/build_debug/output/lib)
link_directories(${AIRSIM_ROOT}/llvm-build/lib)

add_library(lib_airsim_ros_wrapper src/airsim_ros_wrapper.cpp)
# add_dependencies(lib_airsim_ros_wrapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(lib_airsim_ros_wrapper ${catkin_LIBRARIES} AirLib ${RPC_LIB} pthread MavLinkCom)

add_executable(airsim_node src/airsim_node.cpp)
# add_dependencies(airsim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_node lib_airsim_ros_wrapper ${catkin_LIBRARIES} AirLib ${RPC_LIB} pthread MavLinkCom)