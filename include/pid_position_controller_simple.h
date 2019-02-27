#ifndef _PID_POSITION_CONTROLLER_SIMPLE_H_
#define _PID_POSITION_CONTROLLER_SIMPLE_H_

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <airsim_ros_pkgs/SetLocalPosition.h>

// todo nicer api
class PIDParams
{
public:
    double kp_x;
    double kp_y;
    double kp_z;
    double kp_yaw;
    double kd_x;
    double kd_y;
    double kd_z;
    double kd_yaw;

    double reached_thresh_xyz;

    PIDParams():
        kp_x(0.5),
        kp_y(0.5),
        kp_z(0.5),
        kp_yaw(0.5),
        kd_x(0.1),
        kd_y(0.1),
        kd_z(0.1),
        kd_yaw(0.1),
        reached_thresh_xyz(0.5)
        {}

    bool load_from_rosparams(const ros::NodeHandle& nh);
};

// todo should be a common representation
struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};

// todo should be a common representation
class DynamicConstraints
{
public:
    double max_vel_horz_abs; // meters/sec
    double max_vel_vert_abs;

    DynamicConstraints():
        max_vel_horz_abs(1.0),
        max_vel_vert_abs(0.5)
        {}

    bool load_from_rosparams(const ros::NodeHandle& nh);
};

class PIDPositionControllerNode
{
public:
    PIDPositionControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    // ROS service callbacks
    bool local_position_goal_srv_cb(airsim_ros_pkgs::SetLocalPosition::Request& request, airsim_ros_pkgs::SetLocalPosition::Response& response);
    // todo airsim_node should publish geo coordinates for conversion outside
    // bool gps_goal_srv_cb(airsim_ros_pkgs::SetGlobalPosition::Request& request, airsim_ros_pkgs::SetGlobalPosition::Response& response);

    // ROS subscriber callbacks
    void airsim_odom_cb(const nav_msgs::Odometry odom_msg);

    void update_control_cmd_timer_cb(const ros::TimerEvent& event);

    void reset_errors();

    void initialize_ros();
    void compute_control_cmd();
    void enforce_dynamic_constraints();
    void publish_control_cmd();
    void check_reached_goal();


private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    DynamicConstraints constraints_;
    PIDParams params_;
    XYZYaw target_position_;
    XYZYaw curr_position_;
    XYZYaw prev_error_;
    XYZYaw curr_error_;
    nav_msgs::Odometry curr_odom_;
    airsim_ros_pkgs::VelCmd vel_cmd_;
    bool reached_goal_;
    bool has_goal_;
    bool has_odom_;
    // todo check for odom msg being older than n sec

    ros::Publisher airsim_vel_cmd_world_frame_pub_;
    ros::Subscriber airsim_odom_sub_;

    ros::Timer update_control_cmd_timer_;
};

#endif /* _PID_POSITION_CONTROLLER_SIMPLE_ */