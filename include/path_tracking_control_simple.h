#ifndef _PATH_TRACKING_CONTROL_H_
#define _PATH_TRACKING_CONTROL_H_

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>

struct OdometryEuler
{
    double time;
    Eigen::Vector3d position;
    Eigen::Vector3d rpy; //[roll, pitch, yaw]
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;

    nav_msgs::Odometry get_ros_msg()
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time(time);
        // todo automate casting for points
        odom.pose.pose.position.x = position[0];
        odom.pose.pose.position.y = position[1];
        odom.pose.pose.position.z = position[2];

        tf2::Quaternion quat_tf;
        quat_tf.setRPY(rpy[0], rpy[2], rpy[3]);
        odom.pose.pose.orientation = tf2::toMsg(quat_tf);

        odom.twist.twist.linear.x = linear_velocity[0];
        odom.twist.twist.linear.y = linear_velocity[1];
        odom.twist.twist.linear.z = linear_velocity[2];
        odom.twist.twist.angular.x = angular_velocity[0];
        odom.twist.twist.angular.y = angular_velocity[1];
        odom.twist.twist.angular.z = angular_velocity[2];

        return odom;
    }

    void from_ros_msg(const nav_msgs::Odometry &odom)
    {
        time = odom.header.stamp.toSec();
        position[0] = odom.pose.pose.position.x;
        position[1] = odom.pose.pose.position.y;
        position[2] = odom.pose.pose.position.z;

        tf2::Quaternion quat_tf;
        tf2::fromMsg(odom.pose.pose.orientation, quat_tf);
        tf2::Matrix3x3(quat_tf).getRPY(rpy[0], rpy[1], rpy[2]); // ros uses xyzw

        // todo automate casting
        linear_velocity[0] = odom.twist.twist.linear.x;
        linear_velocity[1] = odom.twist.twist.linear.y;
        linear_velocity[2] = odom.twist.twist.linear.z;
        angular_velocity[0] = odom.twist.twist.angular.x;
        angular_velocity[1] = odom.twist.twist.angular.y;
        angular_velocity[2] = odom.twist.twist.angular.z;
    }

};

struct VelControlCmd
{
    Eigen::Vector3d velocity;
    double heading;

    bool isfinite() const 
    {
        return std::isfinite(velocity.x()) && std::isfinite(velocity.y()) && std::isfinite(velocity.z()) &&
            std::isfinite(heading);
    }
};

struct XYZVPsi
{
    Eigen::Vector3d position;
    double vel;
    double heading;
};

typedef std::vector<XYZVPsi> PathXYZVPsi;

class PathTrackingControlParameters
{
public:
    double cross_track_P;
    double cross_track_I;
    double cross_track_D;
    double cross_track_PZ;
    double cross_track_DZ;
    double loop_rate;
    double max_speed;
    double look_ahead_time;
    double look_ahead_angle;
    double cross_track_IMax;
    double tracking_threshold;
    double max_deceleration;
    double reaction_time;
    double land_capture_radius;

    PathTrackingControlParameters():
        cross_track_P(0.0),
        cross_track_I(0.0),
        cross_track_D(5.0),
        cross_track_PZ(0.0),
        cross_track_DZ(0.0),
        loop_rate(5.0),
        max_speed(5.0),
        look_ahead_time(0.5),
        look_ahead_angle(1.0),
        cross_track_IMax(5.0),
        tracking_threshold(10.),
        max_deceleration(3.0),
        reaction_time(0.5) 
        {}

    bool load_params(const ros::NodeHandle &nh);
};


class PathTrackingControl
{
public:
    struct PathTrackingControlState
    {
        double along_track_integrator;
        double cross_track_integrator;
        double prev_along_track_error;
        double prev_cross_track_error;
        double prev_z_track_error;
        double closest_idx;

        PathTrackingControlState():
            along_track_integrator(0.0),
            cross_track_integrator(0.0),
            prev_along_track_error(0.0),
            prev_cross_track_error(0.0),
            prev_z_track_error(0.0),
            closest_idx(0.0){}

        void Reset(void)
        {
            along_track_integrator = 0.0;
            cross_track_integrator = 0.0;
            prev_along_track_error = 0.0;
            prev_cross_track_error = 0.0;
            prev_z_track_error = 0.0;
            closest_idx = 0.0;
        }
    };

    PathTrackingControl(const PathTrackingControlParameters &params) : params_(params) {}

    std::pair<VelControlCmd, bool> get_vel_cmd(double dt, 
        const nav_msgs::Odometry &curr_odom, 
        const PathXYZVPsi &path, 
        PathTrackingControlState &control_state, 
        nav_msgs::Odometry &look_ahead_pose); // next lookahead pose from the path
        
private:
    PathTrackingControlParameters params_;

    // todo take out interpolate etc out of controller class!  
    XYZVPsi interpolate(const XYZVPsi &start_waypoint, const XYZVPsi& end_waypoint, const double &interp_ratio);
    XYZVPsi sample_path(const PathXYZVPsi &path, const double &float_idx);
    XYZVPsi project_odom_on_path(const OdometryEuler &odom, const PathXYZVPsi &path, double &closest_idx_float);

    double get_stopping_distance(const double &curr_speed);
    double get_stopping_speed(const double &stopping_distance);
    std::pair<double, bool> get_dist_to_end_and_is_sharp_corner(const PathXYZVPsi &path, const double &idx_float, const double &max_lookahead_dist);
    std::pair<Eigen::Vector3d, double> vector_to_line_segment(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c);
    std::pair<XYZVPsi, Eigen::Vector3d> get_pursuit_state_pair(const PathXYZVPsi &path, double idx_float, double look_ahead_dist, bool &at_end);

    bool isfinite(const Eigen::Vector3d &v);
    bool isfinite(const OdometryEuler &o);
    bool isfinite(const PathXYZVPsi &p);
    bool isfinite(const VelControlCmd &p);
};

#endif /* _PATH_TRACKING_CONTROL_H_ */
