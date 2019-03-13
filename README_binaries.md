# airsim_roscpp_pkgs

##  Setup 
- Windows 10 machine + [Windows Subsytem for Linux (Ubuntu 16.04)](https://www.microsoft.com/en-us/p/ubuntu-1604-lts)
  * Get [Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/install-win10)
  * Get [Ubuntu 16.04](https://www.microsoft.com/en-us/p/ubuntu-1604-lts/9pjn388hp8c9?activetab=pivot:overviewtab) distro  
  * Install ROS Kinetic
  * Deps: `$ sudo apt-get install ros-kinetic-mavros-msgs`


- Setup for running RViz, rqt_image_view, terminator in Windows + WSL
  * Install [Xming X Server](https://sourceforge.net/projects/xming/). 
  * Find and run `XLaunch` from the Windows start menu.   
  Select `Multiple Windows` in first popup, `Start no client` in second popup, **only** `Clipboard` in third popup. Do **not** select `Native Opengl`.  
  * Open Ubuntu 16.04 session by typing `Ubuntu 16.04` in Windows start menu.  
  * Recommended: Install [terminator](http://www.ubuntugeek.com/terminator-multiple-gnome-terminals-in-one-window.html) : `$ sudo apt-get install terminator.` 
    - You can open terminator in a new window by entering `$ DISPLAY=:0 terminator -u`. 

## Running
- Running the unreal project:
  * Make a folder: `C:\Users\{USERNAME}\Documents\AirSim`
  * Copy provided `settings.json` in release's root folder to `Documents/AirSim/settings.json`. 
  * Go inside the `WindTurbineBinary` directory.   
   Run unreal project binary by double clicking on either `run.bat` for full resolution, `run_720p.bat` for 720x1280, or `run_480p.bat` for 480x640.
 * ROS wrapper   
    - Source the binary workspace   
    `$ source airsim_release_ws/install/setup.bash`
    - Run the launch file:   
    `$ roslaunch airsim_ros_pkgs airsim_with_simple_PID_position_controller.launch`
    - View the launch, msg, srv, rviz, camera calibration files:   
    `$ roscd airsim_ros_pkgs`   
    - Rviz:
    `$ roslaunch airsim_ros_pkgs rviz.launch`
    The default config file is in `airsim_ros_pkgs/rviz/default.rviz`.     
    For using a custom config, you can do a `$ roslaunch airsim_ros_pkgs rviz.launch PATH_TO_RVIZ_FILE`

    -  Compute disparity using stereo_image_proc
        * `ROS_NAMESPACE=front rosrun stereo_image_proc stereo_image_proc`
        * View disparity `rosrun image_view stereo_view stereo:=/front image:=image_rect_color`
        * Read stereo_image_proc's [documentation](https://wiki.ros.org/stereo_image_proc)
        * Improve disparity/depth: [Choose good stereo params](https://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters)

## Tips for the Unreal binary
- Pressing `TAB` shows|hides the blade configuration menu. 
- Press `F1` for AirSim's camera control options. 
  * Here, manual mode, which can be toggled by pressing `M` at any time should be useful to control the unreal viewport camera. Please see how to control to   

## ROS API
This ROS wrapper is composed of two ROS nodes - the first is a wrapper over AirSim's multirotor C++ client library, and the second is a simple PD position controller.    
The ROS parameters can be easily changed in the launch files:
  - `airsim_node.launch`
  - `dynamic_constraints.launch`
  - `position_controller_simple.launch`
  - `airsim_with_simple_PID_position_controller.launch`

**Note**: 
- Please ignore `vehicle_name` field in any rosmsg/srv. Right now, we don't need it as we're focusing on one drone only. But later, the `vehicle_name` field would help us scale to multiple drones. 
- In control messages (velocity set points, gimbal set points), the header of the msg can be ignored.  

Let's look at the ROS API for both nodes: 

### AirSim ROS Wrapper Node
#### Publishers:
- `/global_gps` [sensor_msgs/NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)
- `/home_geo_point` [airsim_ros_pkgs/GPSYaw](msg/GPSYaw.msg)
- `/imu_ground_truth` [sensor_msgs/Imu](https://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
- `/odom_local_ned` [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)   
Odometry in NED frame wrt take-off point 
- `/vehicle_state` [mavros_msgs/State](https://docs.ros.org/api/mavros_msgs/html/msg/State.html)    
- `/front/left/camera_info` [sensor_msgs/CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
- `/front/left/image_raw` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
- `/front/right/camera_info` [sensor_msgs/CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
- `/front/right/image_raw` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
- `/front/left/depth_planar` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)   
 Ground truth depth from left camera's focal plane from AirSim. 
- `/tf` [tf2_msgs/TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

#### Subscribers:
- `/gimbal_angle_euler_cmd` [airsim_ros_pkgs/GimbalAngleEulerCmd](msg/GimbalAngleEulerCmd.msg)   
  Requested gimbal orientation for front-center monocular camera as euler angles (degrees), in **world** frame. 
  Example:
  ``` rostopic pub /gimbal_angle_euler_cmd airsim_ros_pkgs/GimbalAngleEulerCmd "header:
      seq: 0                    
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
      camera_name: 'front_right'
      vehicle_name: ''
      roll: 10.0
      pitch: 0.0
      yaw: 0.0" 
  ```
- `/gimbal_angle_quat_cmd` [airsim_ros_pkgs/GimbalAngleQuatCmd](msg/GimbalAngleQuatCmd.msg)    
  Requested gimbal orientationangle for front-center monocular camera as quaternion, , in **world** frame.
- `/vel_cmd_body_frame` [airsim_ros_pkgs/VelCmd](msg/VelCmd.msg)    
  Ignore `vehicle_name` field, leave it to blank. We can use `vehicle_name` in future for multiple drones.
- `/vel_cmd_world_frame` [airsim_ros_pkgs/VelCmd](msg/VelCmd.msg)    
  Ignore `vehicle_name` field, leave it to blank. We can use `vehicle_name` in future for multiple drones.

#### Services:
- `/land` [std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html)
- `/reset` [std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html)
- `/takeoff` [std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html)

#### Parameters:
- `/front_left_calib_file` [string]   
Default: `airsim_ros_pkgs/calib/front_left_376x672.yaml`
- `/front_right_calib_file` [string]    
 Default: `airsim_ros_pkgs/calib/front_right_376x672.yaml`
- `/update_airsim_control_every_n_sec` [double]   
  Default: 0.01 seconds.    
  Timer callback frequency for updating drone odom and state from airsim, and sending in control commands.    
  The current RPClib interface to unreal engine maxes out at 50 Hz.   
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter. 
- `/update_airsim_img_response_every_n_sec` [double]   
  Default: 0.01 seconds.    
  Timer callback frequency for receiving images from all cameras in airsim.    
  The speed will depend on number of images requested and their resolution.   
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter. 

### Simple PID Position Controller Node 

#### Parameters:
- PD controller parameters:
  * `/kp_x` [double], `/kp_y` [double], `/kp_z` [double, `/kp_yaw` [double]   
    Proportional gain
  * `/kd_x` [double], `/kd_y` [double], `/kd_z` [double, `/kd_yaw` [double]   
    Derivative gain
  * `reached_thresh_xyz` [double]   
    Threshold euler distance from current position to setpoint position 
  * `reached_yaw_degrees` [double]   
    Threshold yaw distance, in degrees from current position to setpoint position 

- `/update_control_every_n_sec` [double]
  Default: 0.01 seconds

#### Services:
- `/airsim_node/gps_goal` [Request: [msgs/airsim_ros_pkgs/GPSYaw](msgs/airsim_ros_pkgs/GPSYaw)]   
  Target gps position + yaw. In absolute altitude
- `/airsim_node/local_position_goal` [Request: [msgs/airsim_ros_pkgs/XYZYaw](msgs/airsim_ros_pkgs/XYZYaw)   
  Target local position + yaw

#### Subscribers:
- `/airsim_node/home_geo_point` [airsim_ros_pkgs/GPSYaw](msg/GPSYaw.msg)   
  Listens to home geo coordinates published by `airsim_node`.  
- `/airsim_node/odom_local_ned` [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)   
  Listens to odometry published by `airsim_node`

#### Publishers:
- `/vel_cmd_world_frame` [airsim_ros_pkgs/VelCmd](airsim_ros_pkgs/VelCmd)   
  Sends velocity command to `airsim_node`

### Global params
- Dynamic constraints. These can be changed in `dynamic_constraints.launch`:  
    * `/max_vel_horz_abs` [double]   
    Maximum horizontal velocity of the drone (meters/second)

    * `/max_vel_vert_abs` [double]   
      Maximum vertical velocity of the drone (meters/second)


## AirSim camera settings 
### Changing camera parameters 
- Frame of reference
The camera positions are defined in a **left-handed coordinate frame** as shown in the image below. +X-axis is along "body front", +Y-axis is along "body right", +Z axis is along "body up" direction.  
![](docs/images/unreal_m210_origin.PNG)

- Stereo   
[This page](https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-) enlists the possible resolutions, and corresponding focal lengths and field of views.   
You can change the default stereo pair pose, image resolution, and _horizontal_ FoV under the "front-left" and "front-right" fields in `Documents\AirSim\Settings.json`. The default parameters are according the `WVGA` settings as [detailed here](https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-).   
More details on AirSim's settings is [available here](https://microsoft.github.io/AirSim/docs/settings/).   
Defaults are (X,Y,Z are in **meters**. ZED's baseline is 12 centimeters, hence we have `-0.06` and `0.06` in Y axis of front_left and front_right):
  * for front-left:
  	```
      "front-left": {
        "CaptureSettings": [
          {
            "ImageType": 0,
            "Width": 672,
            "Height": 376,
            "FOV_Degrees": 87
          }
        ],
        "X": 0.25, "Y": -0.06, "Z": 0.10,
        "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
      },
	```

  * for front-right:
  	```
      "front-right": {
        "CaptureSettings": [
          {
            "ImageType": 0,
            "Width": 672,
            "Height": 376,
            "FOV_Degrees": 87
          }
        ],
        "X": 0.25, "Y": 0.06, "Z": 0.10,
        "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
      }
	```
