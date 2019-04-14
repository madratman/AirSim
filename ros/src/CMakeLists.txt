cmake_minimum_required(VERSION 3.10.0)
project(airsim_ros_pkgs)

#set(AIRSIM_ROOT /projectsym/AirSim_wsl/AirSim)
set(AIRSIM_ROOT /home/madratman/projects/AirSim)
# set(CMAKE_CXX_COMPILER /usr/bin/clang++-5.0)
# set(CMAKE_CXX_COMPILER /usr/bin/g++) #gcc 5.4.0

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 11) 
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) #name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# find_package(Eigen3 REQUIRED) # this in airlib already
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

# add_library(lib_path_tracking_control src/path_tracking_control.cpp)
# target_link_libraries(lib_path_tracking_control ${catkin_LIBRARIES})

add_library(lib_pid_position_controller_simple src/pid_position_controller_simple.cpp)
add_dependencies(lib_pid_position_controller_simple ${catkin_EXPORTED_TARGETS})

add_library(lib_airsim_ros_wrapper src/airsim_ros_wrapper.cpp)
add_dependencies(lib_airsim_ros_wrapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(lib_airsim_ros_wrapper ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib)

add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node lib_airsim_ros_wrapper ${catkin_LIBRARIES} AirLib)

add_executable(pid_position_controller_simple_node src/pid_position_controller_simple_node.cpp)
target_link_libraries(pid_position_controller_simple_node lib_pid_position_controller_simple lib_airsim_ros_wrapper ${catkin_LIBRARIES} AirLib)

install(TARGETS
    #list of nodes
    airsim_node
    pid_position_controller_simple_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    #list of shared libraries
    lib_airsim_ros_wrapper
    lib_pid_position_controller_simple
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES 
  #list of necessary files (xml...)
  README_binaries.md
  README_binaries.html
  settings.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# todo automate launch files, rviz copy
install(DIRECTORY
  launch
  calib
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)
