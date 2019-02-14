#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

class AirsimROSWrapper
{
public:
    AirsimROSWrapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    virtual ~AirsimROSWrapper() {}; // who will really derive this tho

    void initialize();

private:
    msr::airlib::MultirotorRpcLibClient airsim_client_;

    /// ROS camera messages
    sensor_msgs::CameraInfo stereo_left_info_;
    sensor_msgs::CameraInfo stereo_right_info_;
    sensor_msgs::CameraInfo mono_center_info_;

    /// ROS camera publishers
    image_transport::ImageTransport it_;
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;

    /// ROS other publishers
    ros::Publisher odom_ned_pub_;
    ros::Publisher odom_enu_pub_;

    /// ROS Subscribers
    ros::Subscriber vel_cmd_body_frame_sub_;
    ros::Subscriber vel_cmd_world_frame_sub_;

    /// ROS Services
    ros::ServiceServer takeoff_srvr_;
    ros::ServiceServer land_srvr_;
    ros::ServiceServer reset_srvr_;
};