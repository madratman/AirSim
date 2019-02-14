#include <airsim_ros_wrapper.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

AirsimROSWrapper::AirsimROSWrapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
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
/*  it_(nh_);
    left_image_pub_ = it.advertise("/airsim/zed/right/image_raw", 1)
    right_image_pub_ = it.advertise("/airsim/zed/left/image_raw", 1)
*/

    takeoff_srvr_ = nh_private_.advertiseService("generate_mesh", &TsdfServer::generateMeshCallback, this);
    land_srvr_ = nh_private_
    reset_srvr_ = nh_private_

    odom_ned_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom_enu", 10)
    odom_enu_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom_ned", 10)
    
    vel_cmd_body_frame_sub_ = nh_private_.subscribe("vel_cmd_body_frame", 50, &AirsimROSWrapper::vel_cmd_body_frame_cb, this);
    vel_cmd_world_frame_sub_ = nh_private_.subscribe("vel_cmd_world_frame", 50, &AirsimROSWrapper::vel_cmd_world_frame_cb, this);
}

// todo minor: error check. if state is not landed, return error. 
void takeoff_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.takeoffAsync().join()
}

// todo minor: error check. if state is not in air, return error. 
void land_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.landAsync().join()
}

void AirsimROSWrapper::vel_cmd_world_frame_cb(const geometry_msgs::Twist &msg)
{
    auto drone_state = airsim_client_.getMultirotorState()
    auto quaternion_ned = drone_state.kinematics_estimated.orientation # airsim uses wxyz
    quaternion_ros = (quaternion_ned.x_val, quaternion_ned.y_val, quaternion_ned.z_val, quaternion_ned.w_val) # ros uses xyzw
    roll, pitch, yaw = transformations.euler_from_quaternion(quaternion_ros)
    double vx_body = (msg.linear.x * cos(yaw)) - (msg.linear.y * sin(yaw))
    double vy_body = (msg.linear.x * sin(yaw)) + (msg.linear.y * cos(yaw))

    // rospy.loginfo("sending vel cmd body frame vx: {} vy: {} vz: {} ang.z: {} duration: {}".format(vx_body, vy_body, msg.linear.z, msg.angular.z, self.vel_cmd_duration))

    airsim_client_.moveByVelocityAsync(vx_body, vy_body, msg.linear.z, self.vel_cmd_duration, 
        drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(is_rate=True, yaw_or_rate=msg.angular.z))

}

void AirsimROSWrapper::vel_cmd_world_frame_cb(const geometry_msgs::Twist &msg)
{
    airsim_client_.moveByVelocityAsync(msg.linear.x, msg.linear.y, msg.linear.z, self.vel_cmd_duration, 
        drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(is_rate=True, yaw_or_rate=msg.angular.z), vehicle_name = '')
}