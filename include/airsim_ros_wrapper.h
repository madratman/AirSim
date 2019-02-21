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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// todo move airlib typedefs to separate header file?
typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

class AirsimROSWrapper
{
public:
    AirsimROSWrapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~AirsimROSWrapper() {}; // who will really derive this tho

    void initialize_airsim();
    void initialize_ros();

    /// ROS subscriber callbacks
    void vel_cmd_world_frame_cb(const geometry_msgs::Twist &msg);
    void vel_cmd_body_frame_cb(const geometry_msgs::Twist &msg);

    /// ROS service callbacks
    bool takeoff_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool land_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reset_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    void process_and_publish_img_response(const std::vector<ImageResponse>& img_response);
    void img_response_timer_callback(const ros::TimerEvent& event);
    void manual_decode_rgb(const ImageResponse &img_response, cv::Mat &mat);

private:
    msr::airlib::MultirotorRpcLibClient airsim_client_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // const std::vector<ImageResponse> img_response_;

    /// ROS tf
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    /// ROS params
    double vel_cmd_duration_;

    /// ROS Timers.
    ros::Timer airsim_img_response_timer_;

    /// ROS camera messages
    sensor_msgs::CameraInfo stereo_left_info_;
    sensor_msgs::CameraInfo stereo_right_info_;
    sensor_msgs::CameraInfo mono_center_info_;

    /// ROS camera publishers
    // image_transport::ImageTransport it_;
    // image_transport::Publisher left_image_pub_;
    // image_transport::Publisher right_image_pub_;
    ros::Publisher cam_0_pose_pub_; 

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