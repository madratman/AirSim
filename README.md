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