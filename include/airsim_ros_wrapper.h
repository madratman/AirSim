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
#include "yaml-cpp/yaml.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <ros/console.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>
#include <airsim_ros_pkgs/GimbalAngleQuatCmd.h>
#include <airsim_ros_pkgs/GimbalAngleEulerCmd.h>

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

class AirsimROSWrapper
{
public:
    AirsimROSWrapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~AirsimROSWrapper() {}; // who will really derive this tho

    void initialize_airsim();
    void initialize_ros();

    /// ROS timer callbacks
    void img_response_timer_callback(const ros::TimerEvent& event); // update images from airsim_client_ every nth sec
    void drone_state_timer_callback(const ros::TimerEvent& event); // update drone state from airsim_client_ every nth sec

    /// ROS subscriber callbacks
    void vel_cmd_world_frame_cb(const geometry_msgs::Twist &msg);
    void vel_cmd_body_frame_cb(const geometry_msgs::Twist &msg);
    void gimbal_angle_quat_cmd_cb(const airsim_ros_pkgs::GimbalAngleQuatCmd &gimbal_angle_quat_cmd_msg);
    void gimbal_angle_euler_cmd_cb(const airsim_ros_pkgs::GimbalAngleEulerCmd &gimbal_angle_euler_cmd_msg);

    /// ROS service callbacks
    bool takeoff_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool land_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reset_srv_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    /// ROS tf broadcasters
    void publish_camera_tf(const ImageResponse &img_response, const std_msgs::Header &header, const std::string &child_frame_id);

    /// camera helper methods
    void process_and_publish_img_response(const std::vector<ImageResponse>& img_response);
    void manual_decode_rgb(const ImageResponse &img_response, cv::Mat &mat);
    void read_params_from_yaml_and_fill_cam_info_msg(const std::string& file_name, sensor_msgs::CameraInfo& cam_info);
    void convert_yaml_to_simple_mat(const YAML::Node& node, SimpleMatrix& m); // todo ugly

    /// utils. parse into an Airlib<->ROS conversion class
    tf2::Quaternion get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat);
    msr::airlib::Quaternionr get_airlib_quat(const geometry_msgs::Quaternion& geometry_msgs_quat);
    msr::airlib::Quaternionr get_airlib_quat(const tf2::Quaternion& tf2_quat);
    nav_msgs::Odometry get_odom_msg_from_airsim_state(const msr::airlib::MultirotorState &drone_state);
    sensor_msgs::NavSatFix get_gps_msg_from_airsim_state(const msr::airlib::MultirotorState &drone_state);
    mavros_msgs::State get_vehicle_state_msg(msr::airlib::MultirotorState &drone_state);


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
    ros::Timer airsim_control_update_timer_;

    /// ROS camera messages
    sensor_msgs::CameraInfo airsim_cam_info_front_left_;
    sensor_msgs::CameraInfo airsim_cam_info_front_right_;
    sensor_msgs::CameraInfo airsim_cam_info_front_mono_;

    /// ROS camera publishers
    // image_transport::ImageTransport it_;
    // image_transport::Publisher left_image_pub_;
    // image_transport::Publisher right_image_pub_;
    ros::Publisher cam_0_pose_pub_; 

    /// ROS other publishers
    ros::Publisher odom_local_ned_pub_;
    ros::Publisher global_gps_pub_;
    ros::Publisher attitude_euler_pub_;
    ros::Publisher attitude_quat_pub_;
    ros::Publisher vehicle_state_pub_;

    /// ROS Subscribers
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