#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "common/common_utils/FileSystem.hpp"
#include "ros/ros.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "yaml-cpp/yaml.h"
#include <airsim_ros_pkgs/GimbalAngleEulerCmd.h>
#include <airsim_ros_pkgs/GimbalAngleQuatCmd.h>
#include <airsim_ros_pkgs/SetGlobalPosition.h>
#include <airsim_ros_pkgs/WaypointXYZVPsi.h>
#include <airsim_ros_pkgs/SetLocalPosition.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <airsim_ros_pkgs/GPSYaw.h>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math_common.h>

// todo move airlib typedefs to separate header file?
typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

struct SimpleMatrix
{
    int rows;
    int cols;
    double* data;

    SimpleMatrix(int rows, int cols, double* data)
        : rows(rows), cols(cols), data(data)
    {}
};

struct VelCmd
{
    double x;
    double y;
    double z;
    msr::airlib::DrivetrainType drivetrain;
    msr::airlib::YawMode yaw_mode;
    std::string vehicle_name;

    // VelCmd() : 
    //     x(0), y(0), z(0), 
    //     vehicle_name("") {drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    //             yaw_mode = msr::airlib::YawMode();};

    // VelCmd(const double &x, const double &y, const double &z, 
    //         msr::airlib::DrivetrainType drivetrain, 
    //         const msr::airlib::YawMode &yaw_mode,
    //         const std::string &vehicle_name) : 
    //     x(x), y(y), z(z), 
    //     drivetrain(drivetrain), 
    //     yaw_mode(yaw_mode), 
    //     vehicle_name(vehicle_name) {};
};

struct GimbalCmd
{
    std::string vehicle_name;
    std::string camera_name;
    msr::airlib::Quaternionr target_quat;

    // GimbalCmd() : vehicle_name(vehicle_name), camera_name(camera_name), target_quat(msr::airlib::Quaternionr(1,0,0,0)) {}

    // GimbalCmd(const std::string &vehicle_name, 
    //         const std::string &camera_name, 
    //         const msr::airlib::Quaternionr &target_quat) : 
    //         vehicle_name(vehicle_name), camera_name(camera_name), target_quat(target_quat) {};
};

class AirsimROSWrapper
{
public:
    AirsimROSWrapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~AirsimROSWrapper() {}; // who will really derive this tho

    void initialize_airsim();
    void initialize_ros();
    void setup_vehicle_constraints(); // todo make ros params

    /// ROS timer callbacks
    void img_response_timer_cb(const ros::TimerEvent& event); // update images from airsim_client_ every nth sec
    void drone_state_timer_cb(const ros::TimerEvent& event); // update drone state from airsim_client_ every nth sec

    /// ROS subscriber callbacks
    void vel_cmd_world_frame_cb(const airsim_ros_pkgs::VelCmd &msg);
    void vel_cmd_body_frame_cb(const airsim_ros_pkgs::VelCmd &msg);
    void gimbal_angle_quat_cmd_cb(const airsim_ros_pkgs::GimbalAngleQuatCmd &gimbal_angle_quat_cmd_msg);
    void gimbal_angle_euler_cmd_cb(const airsim_ros_pkgs::GimbalAngleEulerCmd &gimbal_angle_euler_cmd_msg);

    void set_zero_vel_cmd();

    /// ROS service callbacks
    bool takeoff_srv_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool land_srv_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reset_srv_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    /// ROS tf broadcasters
    void publish_camera_tf(const ImageResponse &img_response, const std_msgs::Header &header, const std::string &child_frame_id);
    void publish_odom_tf(const nav_msgs::Odometry &odom_ned_msg);

    /// camera helper methods
    // TODO migrate to image_tranport camera publisher https://answers.ros.org/question/278602/how-to-use-camera_info_manager-to-publish-camera_info/
    void process_and_publish_img_response(const std::vector<ImageResponse>& img_response);
    cv::Mat manual_decode_rgb(const ImageResponse &img_response);
    cv::Mat manual_decode_depth(const ImageResponse &img_response);
    void read_params_from_yaml_and_fill_cam_info_msg(const std::string& file_name, sensor_msgs::CameraInfo& cam_info);
    void convert_yaml_to_simple_mat(const YAML::Node& node, SimpleMatrix& m); // todo ugly

    /// utils. parse into an Airlib<->ROS conversion class
    tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat);
    msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::Quaternion& geometry_msgs_quat);
    msr::airlib::Quaternionr get_airlib_quat(const tf2::Quaternion& tf2_quat);
    nav_msgs::Odometry get_odom_msg_from_airsim_state(const msr::airlib::MultirotorState &drone_state);
    airsim_ros_pkgs::GPSYaw get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint &geo_point);
    sensor_msgs::NavSatFix get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint &geo_point);
    mavros_msgs::State get_vehicle_state_msg(msr::airlib::MultirotorState &drone_state);
    sensor_msgs::Imu get_ground_truth_imu_msg_from_airsim_state(const msr::airlib::MultirotorState &drone_state);

private:
    msr::airlib::MultirotorRpcLibClient airsim_client_;
    msr::airlib::MultirotorState curr_drone_state_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    msr::airlib::GeoPoint home_geo_point_;// gps coord of unreal origin 
    airsim_ros_pkgs::GPSYaw home_geo_point_msg_; // todo duplicate

    bool in_air_; // todo not really used 
    double max_horz_vel_;
    double max_vert_vel_;

    /// vehiclecontrol commands (received from last callback)
    // todo make a struct for control cmd, perhaps line with airlib's API 
    bool has_vel_cmd_;
    VelCmd vel_cmd_;

    nav_msgs::Odometry curr_odom_ned_;

    // gimbal control
    bool has_gimbal_cmd_;
    GimbalCmd gimbal_cmd_; 

    /// ROS tf
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    /// ROS params
    double vel_cmd_duration_;

    /// ROS Timers.
    ros::Timer airsim_img_response_timer_;
    ros::Timer airsim_control_update_timer_;

    /// ROS camera messages
    sensor_msgs::CameraInfo front_left_cam_info_msg_;
    sensor_msgs::CameraInfo front_right_cam_info_msg_;
    // sensor_msgs::CameraInfo front_center_mono_cam_info_msg_;

    /// ROS camera publishers
    image_transport::ImageTransport it_;
    image_transport::Publisher front_left_img_raw_pub_;
    image_transport::Publisher front_right_img_raw_pub_;
    image_transport::Publisher front_left_depth_planar_pub_;
    ros::Publisher front_left_cam_info_pub_;
    ros::Publisher front_right_cam_info_pub_;

    /// ROS other publishers
    ros::Publisher clock_pub_;
    ros::Publisher odom_local_ned_pub_;
    ros::Publisher global_gps_pub_;
    ros::Publisher attitude_euler_pub_;
    ros::Publisher attitude_quat_pub_;
    ros::Publisher vehicle_state_pub_;
    ros::Publisher imu_ground_truth_pub_;
    ros::Publisher origin_geo_point_pub_; // geo coord of unreal origin
    ros::Publisher home_geo_point_pub_; // home geo coord of drones

    /// ROS Subscribers
    // ros::CallbackQueue img_callback_queue_
    // ros::SubscribeOptions sub_ops_;
    ros::Subscriber vel_cmd_body_frame_sub_;
    ros::Subscriber vel_cmd_world_frame_sub_;
    ros::Subscriber gimbal_angle_quat_cmd_sub_;
    ros::Subscriber gimbal_angle_euler_cmd_sub_;

    /// ROS Services
    ros::ServiceServer takeoff_srvr_;
    ros::ServiceServer land_srvr_;
    ros::ServiceServer reset_srvr_;

    static constexpr char CAM_YML_NAME[]    = "camera_name";
    static constexpr char WIDTH_YML_NAME[]  = "image_width";
    static constexpr char HEIGHT_YML_NAME[] = "image_height";
    static constexpr char K_YML_NAME[]      = "camera_matrix";
    static constexpr char D_YML_NAME[]      = "distortion_coefficients";
    static constexpr char R_YML_NAME[]      = "rectification_matrix";
    static constexpr char P_YML_NAME[]      = "projection_matrix";
    static constexpr char DMODEL_YML_NAME[] = "distortion_model";

    std::string front_left_calib_file_;
    std::string front_right_calib_file_;

    bool is_armed_;
    std::string mode_;
};