cmake_minimum_required(VERSION 3.11.0)
project(airsim_ros_pkgs)

set(AIRSIM_ROOT /projectsym/AirSim_wsl/AirSim)
# set(AIRSIM_ROOT /home/madratman/projects/AirSim)
# set(CMAKE_CXX_COMPILER /usr/bin/clang++-5.0)
# set(CMAKE_CXX_COMPILER /usr/bin/g++)

# set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}") # this causes /home/madratman/projects/AirSim/external/rpclib/rpclib-2.2.1/include/rpc/rpc_error.h:6:10: fatal error: 'exception' file not found #include <exception>

## following 3 lines are not really needed (but wait they should be?)
# LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules")
# INCLUDE("${AIRSIM_ROOT}/cmake/cmake-modules/CommonSetup.cmake")
# CommonSetup()

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) #name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

set(CMAKE_CXX_STANDARD 11) 
# set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=0")
# find_package(Eigen3 REQUIRED) 
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

add_message_files(
  FILES
  GimbalAngleEulerCmd.msg
  GimbalAngleQuatCmd.msg
  GPSYaw.msg
  VelCmd.msg
  WaypointXYZVPsi.msg
  PathXYZVPsi.msg
)

add_service_files(
  FILES
  SetGPSPosition.srv
  SetLocalPosition.srv
  PathXYZVPsiSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${AIRSIM_ROOT}/build_debug/output/lib)
link_directories(${AIRSIM_ROOT}/llvm-build/lib)
link_directories(${AIRSIM_ROOT}/Unity/linux-build)

add_library(lib_airsim_ros_wrapper src/airsim_ros_wrapper.cpp)
add_library(lib_path_tracking_control src/path_tracking_control.cpp)
add_library(lib_pid_position_controller_simple src/pid_position_controller_simple.cpp)

# add_dependencies(lib_airsim_ros_wrapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(lib_path_tracking_control ${catkin_LIBRARIES})
target_link_libraries(lib_airsim_ros_wrapper ${catkin_LIBRARIES} AirsimWrapper ${OpenCV_LIBS} yaml-cpp lib_path_tracking_control)

add_executable(airsim_node src/airsim_node.cpp)
# add_dependencies(airsim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_node lib_airsim_ros_wrapper ${catkin_LIBRARIES} AirsimWrapper)

# add_dependencies(airsim_node ${catkin_EXPORTED_TARGETS})
add_executable(pid_position_controller_simple_node src/pid_position_controller_simple_node.cpp)
target_link_libraries(pid_position_controller_simple_node lib_pid_position_controller_simple lib_airsim_ros_wrapper ${catkin_LIBRARIES} AirsimWrapper)