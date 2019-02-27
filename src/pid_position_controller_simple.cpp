#include "pid_position_controller_simple.h"

bool PIDParams::load_from_rosparams(const ros::NodeHandle &nh)
{
    bool found = true;

    found = found && nh.getParam("kp_x", kp_x);
    found = found && nh.getParam("kp_y", kp_y);
    found = found && nh.getParam("kp_z", kp_z);
    found = found && nh.getParam("kp_yaw", kp_yaw);

    found = found && nh.getParam("kd_x", kd_x);
    found = found && nh.getParam("kd_y", kd_y);
    found = found && nh.getParam("kd_z", kd_z);
    found = found && nh.getParam("kd_yaw", kd_yaw);

    found = found && nh.getParam("reached_thresh_xyz", reached_thresh_xyz);

    return found;
}

bool DynamicConstraints::load_from_rosparams(const ros::NodeHandle &nh)
{
    bool found = true;

    found = found && nh.getParam("max_vel_horz_abs", max_vel_horz_abs);
    found = found && nh.getParam("max_vel_vert_abs", max_vel_vert_abs);

    return found;
}


PIDPositionControllerNode::PIDPositionControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), 
    has_odom_(false), has_goal_(false), reached_goal_(false)
{
    params_.load_from_rosparams(nh_private_);
    constraints_.load_from_rosparams(nh_);
    initialize_ros();
    reset_errors();
}

void PIDPositionControllerNode::reset_errors()
{
    prev_error_.x = 0.0;
    prev_error_.y = 0.0;
    prev_error_.z = 0.0;
    prev_error_.yaw = 0.0;
}

void PIDPositionControllerNode::initialize_ros()
{
    vel_cmd_ = airsim_ros_pkgs::VelCmd();
    // ROS params
    double update_control_every_n_sec;
    nh_private_.getParam("update_control_every_n_sec", update_control_every_n_sec);

    // ROS publishers
    airsim_vel_cmd_world_frame_pub_ = nh_private_.advertise<airsim_ros_pkgs::VelCmd>("/vel_cmd_body_frame", 1);
 
    // ROS subscribers
    airsim_odom_sub_ = nh_.subscribe("/airsim_node/odom_local_ned", 50, &PIDPositionControllerNode::airsim_odom_cb, this);

    // ROS timers
    update_control_cmd_timer_ = nh_private_.createTimer(ros::Duration(update_control_every_n_sec), &PIDPositionControllerNode::update_control_cmd_timer_cb, this);
}

void PIDPositionControllerNode::airsim_odom_cb(const nav_msgs::Odometry odom_msg)
{
    has_odom_ = true;
    curr_odom_ = odom_msg;
    curr_position_.x = odom_msg.pose.pose.position.x;
    curr_position_.y = odom_msg.pose.pose.position.y;
    curr_position_.z = odom_msg.pose.pose.position.z;
    // curr_position_.yaw = odom_msg.pose.pose.position.yaw; // todo
}

// todo maintain internal representation as eigen vec?
// todo check if low velocity if within thresh?
void PIDPositionControllerNode::check_reached_goal()
{
    double euler_xyz = sqrt((target_position_.x - curr_position_.x) * (target_position_.x - curr_position_.x) 
                        + (target_position_.y - curr_position_.y) * (target_position_.y - curr_position_.y)
                        + (target_position_.z - curr_position_.z) * (target_position_.z - curr_position_.z));
    // double dyaw = (target_position_.x - target_position_.x)
    if (euler_xyz < params_.reached_thresh_xyz)
        reached_goal_ = true; 
}

bool PIDPositionControllerNode::local_position_goal_srv_cb(airsim_ros_pkgs::SetLocalPosition::Request& request, airsim_ros_pkgs::SetLocalPosition::Response& response)
{
    if ( has_goal_ && !reached_goal_)
    {
        // todo maintain array of position goals
        ROS_ERROR_STREAM("denying position goal request. I am still following the previous goal");
        return false;
    }

    if (!has_goal_ || (has_goal_ && reached_goal_))
    {
        target_position_.x = request.x;
        target_position_.y = request.y;
        target_position_.z = request.z;
        target_position_.yaw = request.yaw;
    }       

    // todo error checks 
    // todo fill response
    has_goal_ = true;
    reset_errors(); // todo
    return true;
}

void PIDPositionControllerNode::update_control_cmd_timer_cb(const ros::TimerEvent& event)
{
    check_reached_goal();
    if (has_goal_ && (!reached_goal_) && (has_odom_))
    {
        compute_control_cmd();
        enforce_dynamic_constraints();
        publish_control_cmd();
    }
    else
    {
        // todo
    }
}

void PIDPositionControllerNode::compute_control_cmd()
{
    curr_error_.x = target_position_.x - curr_position_.x;
    curr_error_.y = target_position_.y - curr_position_.y;
    curr_error_.z = target_position_.z - curr_position_.z;
    // curr_error_.yaw = target_position_.yaw - curr_position_.yaw;
   
    double p_term_x = params_.kp_x * curr_error_.x;
    double p_term_y = params_.kp_y * curr_error_.y;
    double p_term_z = params_.kp_z * curr_error_.z;
    // p_term_yaw = params_.kp_yaw * curr_error_.yaw

    double d_term_x = params_.kd_x * prev_error_.x;
    double d_term_y = params_.kd_y * prev_error_.y;
    double d_term_z = params_.kd_z * prev_error_.z;
    // d_term_yaw = params_.kp_yaw * prev_error_.yaw

    prev_error_ = curr_error_;

    vel_cmd_.twist.linear.x = p_term_x + d_term_x;
    vel_cmd_.twist.linear.y = p_term_y + d_term_y;
    vel_cmd_.twist.linear.z = p_term_z + d_term_z;
    // vel_cmd_.twist.angular.z = 0.0; // todo
}

void PIDPositionControllerNode::enforce_dynamic_constraints()
{
    double vel_norm_horz = sqrt((vel_cmd_.twist.linear.x * vel_cmd_.twist.linear.x) 
                            + (vel_cmd_.twist.linear.y * vel_cmd_.twist.linear.y));

    if (vel_norm_horz > constraints_.max_vel_horz_abs)
    {
        vel_cmd_.twist.linear.x = (vel_cmd_.twist.linear.x / vel_norm_horz) * constraints_.max_vel_horz_abs; 
        vel_cmd_.twist.linear.y = (vel_cmd_.twist.linear.y / vel_norm_horz) * constraints_.max_vel_horz_abs; 
    }

    if (std::fabs(vel_cmd_.twist.linear.z) > constraints_.max_vel_vert_abs)
    {
        // just do a sgn?
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * constraints_.max_vel_vert_abs; 
    }
    // todo yaw limits
}

void PIDPositionControllerNode::publish_control_cmd()
{
    airsim_vel_cmd_world_frame_pub_.publish(vel_cmd_);
}