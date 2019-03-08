# airsim_roscpp_pkgs

For internal use, don't use this in open source version as of now
##  Setup 
- Ubuntu 16.04 + ROS Kinetic. WSL works fine as well
- Deps:
`$ sudo apt-get install ros-kinetic-mavros-msgs`

##  Build
- Build AirSim 
```
cd $(AIRSIM_ROOT);
./setup.sh;
./build.sh;
cd $(AIRSIM_ROOT)/Unity;
./build.sh;
```
- Build ROS package

```
# set compiler to clang
export CXX=/usr/bin/clang++-5.0
mkdir -p airsim_ros_ws/src && cd $_
git clone https://github.com/madratman/airsim_roscpp_pkgs.git
cd ../
catkin_make # or catkin build
```

## Setup for Windows Subsystem for Linux for running RViz, rqt_image_view, terminator 
- Install [Xming X Server](https://sourceforge.net/projects/xming/). 
- Run "XLaunch" from Windows start menu. Select `Multiple Windows` in first popup, `Start no client` in second, **ONLY** `Clipboard` in third popup. Do not select `Native Opengl`.  
- Install terminator : `sudo apt-get install terminator` and [learn how to split windows](http://www.ubuntugeek.com/terminator-multiple-gnome-terminals-in-one-window.html)
- Open Ubuntu 16.04 session, and enter `$ DISPLAY=:0 terminator -u`. 

## Running
```
cd airsim_ros_ws
source devel/setup.bash
roslaunch airsim_ros_pkgs airsim_with_simple_PID_position_controller.launch
rviz -d rviz/default.rviz
```
## Compute disparity using stereo_image_proc
- `ROS_NAMESPACE=front rosrun stereo_image_proc stereo_image_proc`
- View disparity `rosrun image_view stereo_view stereo:=/front image:=image_rect_color`
- Read stereo_image_proc's [documentation](https://wiki.ros.org/stereo_image_proc)
- Improve disparity/depth: [Choose good stereo params](https://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters)

# ROS API
## AirSim ROS Wrapper Node
### Publishers:
- /global_gps [sensor_msgs/NavSatFix] -- TODO: description
- /home_geo_point [airsim_ros_pkgs/GPSYaw] -- TODO: description
- /imu_ground_truth [sensor_msgs/Imu] -- TODO: description
- /odom_local_ned [nav_msgs/Odometry] -- TODO: description
- /vehicle_state [mavros_msgs/State] -- TODO: description
- /front/left/camera_info [sensor_msgs/CameraInfo] -- TODO: description
- /front/left/depth_planar [sensor_msgs/Image] -- TODO: description
- /front/left/image_raw [sensor_msgs/Image] -- TODO: description
- /front/right/camera_info [sensor_msgs/CameraInfo] -- TODO: description
- /front/right/image_raw [sensor_msgs/Image] -- TODO: description
- /tf [tf2_msgs/TFMessage] -- TODO: description

### Subscribers:
- /gimbal_angle_euler_cmd [airsim_ros_pkgs/GimbalAngleEulerCmd] -- TODO: description
- /gimbal_angle_quat_cmd [airsim_ros_pkgs/GimbalAngleQuatCmd] -- TODO: description
- /vel_cmd_body_frame [airsim_ros_pkgs/VelCmd] -- TODO: description
- /vel_cmd_world_frame [airsim_ros_pkgs/VelCmd] -- TODO: description

### Services:
- /land [TODO: type] -- TODO: description
- /reset [TODO: type] -- TODO: description
- /takeoff [TODO: type] -- TODO: description

### Parameters:
- /front_left_calib_file [TODO: type] -- TODO: description
- /front_right_calib_file [TODO: type] -- TODO: description
- /update_airsim_control_every_n_sec [TODO: type] -- TODO: description
- /update_airsim_img_response_every_n_sec [TODO: type] -- TODO: description
- /max_horz_vel [TODO: type] -- TODO: description
- /max_vert_vel_ [TODO: type] -- TODO: description

## Simple PID Position Controller Node 

### Parameters:
- /enable_statistics [TODO: type] -- TODO: description
- /max_vel_horz_abs [TODO: type] -- TODO: description
- /max_vel_vert_abs [TODO: type] -- TODO: description
- /kp_x [TODO: type] -- TODO: description
- /update_control_every_n_sec [TODO: type] -- TODO: description

### Services:
- /airsim_node/gps_goal [TODO: type] -- TODO: description
- /airsim_node/local_position_goal [TODO: type] -- TODO: description

### Subscribers:
- /airsim_node/home_geo_point [airsim_ros_pkgs/GPSYaw] -- TODO: description
- /airsim_node/odom_local_ned [nav_msgs/Odometry] -- TODO: description

### Publishers:
- /vel_cmd_world_frame [airsim_ros_pkgs/VelCmd] -- TODO: description


# AirSim camera settings 
## Changing camera resolution

## Changing stereo baseline 

## Changing camera lens configuration 
- Not supported yet
