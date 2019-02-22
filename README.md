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

## Run
```
cd airsim_ros_ws
source devel/setup.bash
rosrun airsim_ros_pkgs airsim_node
```
