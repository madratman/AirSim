#include <airsim_ros_wrapper.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <ros/console.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>

AirsimROSWrapper::AirsimROSWrapper(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{}

void AirsimROSWrapper::initialize_airsim()
{
    try
    {
        airsim_client_.confirmConnection();
        airsim_client_.enableApiControl(true); // todo expose as rosservice?
        airsim_client_.armDisarm(true); // todo expose as rosservice?

    }
    catch (rpc::rpc_error&  e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }
}

void AirsimROSWrapper::initialize_ros()
{
    // ros params
    vel_cmd_duration_ = 0.05;
/*  it_(nh_);
    left_image_pub_ = it.advertise("/airsim/zed/right/image_raw", 1)
    right_image_pub_ = it.advertise("/airsim/zed/left/image_raw", 1)
*/

    takeoff_srvr_ = nh_private_.advertiseService("takeoff", &AirsimROSWrapper::takeoff_srv_callback, this);
    land_srvr_ = nh_private_.advertiseService("land", &AirsimROSWrapper::land_srv_callback, this);
    reset_srvr_ = nh_private_.advertiseService("reset",&AirsimROSWrapper::reset_srv_callback, this);

    odom_ned_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom_enu", 10);
    odom_enu_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom_ned", 10);
    
    vel_cmd_body_frame_sub_ = nh_private_.subscribe("vel_cmd_body_frame", 50, &AirsimROSWrapper::vel_cmd_body_frame_cb, this); // todo ros::TransportHints().tcpNoDelay();
    vel_cmd_world_frame_sub_ = nh_private_.subscribe("vel_cmd_world_frame", 50, &AirsimROSWrapper::vel_cmd_world_frame_cb, this);
}

// todo minor: error check. if state is not landed, return error. 
bool AirsimROSWrapper::takeoff_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.takeoffAsync()->waitOnLastTask();
    return true; //todo
}

// todo minor: error check. if state is not in air, return error. 
bool AirsimROSWrapper::land_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.landAsync()->waitOnLastTask();
    return true; //todo
}

bool AirsimROSWrapper::reset_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.reset();
    return true; //todo
}

void AirsimROSWrapper::vel_cmd_body_frame_cb(const geometry_msgs::Twist &msg)
{
    double roll, pitch, yaw;
    auto drone_state = airsim_client_.getMultirotorState();
    auto quaternion_ned = drone_state.kinematics_estimated.pose.orientation; // airsim uses wxyz
    tf2::Matrix3x3(tf2::Quaternion(quaternion_ned.x(), quaternion_ned.y(), 
        quaternion_ned.z(), quaternion_ned.w())).getRPY(roll, pitch, yaw); // ros uses xyzw

    double vx_body = (msg.linear.x * cos(yaw)) - (msg.linear.y * sin(yaw));
    double vy_body = (msg.linear.x * sin(yaw)) + (msg.linear.y * cos(yaw));

    // rospy.loginfo("sending vel cmd body frame vx: {} vy: {} vz: {} ang.z: {} duration: {}".format(vx_body, vy_body, msg.linear.z, msg.angular.z, self.vel_cmd_duration))

    airsim_client_.moveByVelocityAsync(vx_body, vy_body, msg.linear.z, vel_cmd_duration_, 
        msr::airlib::DrivetrainType::MaxDegreeOfFreedom, msr::airlib::YawMode(true, msg.angular.z));
}

void AirsimROSWrapper::vel_cmd_world_frame_cb(const geometry_msgs::Twist &msg)
{
    airsim_client_.moveByVelocityAsync(msg.linear.x, msg.linear.y, msg.linear.z, vel_cmd_duration_, 
        msr::airlib::DrivetrainType::MaxDegreeOfFreedom, msr::airlib::YawMode(true, msg.angular.z));
}