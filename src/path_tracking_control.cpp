#include "path_tracking_control.h"

// todo
double Limit(double limit, double value)
{
    if ( value>limit ) {
        return limit;
    } else {
        if ( value<-limit )
            return -limit;
        else
            return value;
    }
}

bool PathTrackingControlParameters::load_params(const ros::NodeHandle &nh)
{
    bool found = true;

    found = found && nh.getParam("cross_track_P", cross_track_P);
    found = found && nh.getParam("cross_track_I", cross_track_I);
    found = found && nh.getParam("cross_track_D", cross_track_D);
    found = found && nh.getParam("cross_track_IMax", cross_track_IMax);

    found = found && nh.getParam("cross_track_PZ", cross_track_PZ);
    found = found && nh.getParam("cross_track_DZ", cross_track_DZ);

    found = found && nh.getParam("loop_rate", loop_rate);
    found = found && nh.getParam("max_speed", max_speed);
    found = found && nh.getParam("look_ahead_time", look_ahead_time);
    found = found && nh.getParam("look_ahead_angle", look_ahead_angle);
    found = found && nh.getParam("tracking_threshold", tracking_threshold);
    found = found && nh.getParam("max_deceleration", max_deceleration);
    found = found && nh.getParam("reaction_time", reaction_time);
    return found;
}

// todo return std::pair 

std::pair<Eigen::Vector3d, double> PathTrackingControl::vector_to_line_segment(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c)
{
    Eigen::Vector3d AB(b - a);
    Eigen::Vector3d AC(c - a);
    double ABl2 = AB.dot(AB) + std::numeric_limits<double>::epsilon();
    double r = AC.dot(AB) / ABl2;
    Eigen::Vector3d result;
    if(r < 0)
    {
        result = a - c; // Closer to A
    }
    else if(r > 1)
    {
        result = b - c; // Closer to B
    } 
    else
    {
        Eigen::Vector3d mix((1.0-r) * a + r * b);
        result = mix - c;
    }

    return std::make_pair(result, r);
}


std::pair<VelControlCmd, bool> PathTrackingControl::get_vel_cmd(double dt, const nav_msgs::Odometry &curr_odom, const PathXYZVPsi &path, PathTrackingControlState &control_state, nav_msgs::Odometry &look_ahead_pose)
{
    OdometryEuler curr_state;
    curr_state.from_ros_msg(curr_odom);
    //ROS_ERROR_STREAM_THROTTLE(1.0, "*** current state - pos: " << curr_state.position[0] << " / " << curr_state.position[1] << " / " << curr_state.position[2]); 
    //ROS_ERROR_STREAM_THROTTLE(1.0, "*** current state - ori: " << curr_state.rpy[0] << " / " << curr_state.rpy[1] << " / " << curr_state.rpy[2]); 
    //ROS_ERROR_STREAM_THROTTLE(1.0, "*** current state - lin: " << curr_state.linear_velocity[0] << " / " << curr_state.linear_velocity[1] << " / " << curr_state.linear_velocity[2]); 
    //ROS_ERROR_STREAM_THROTTLE(1.0, "*** current state - ang: " << curr_state.angular_velocity[0] << " / " << curr_state.angular_velocity[1] << " / " << curr_state.angular_velocity[2]); 

    // Create empty control with zero velocity and current heading. 
    VelControlCmd command;
    command.velocity = Eigen::Vector3d::Zero();
    command.heading = curr_state.rpy[2];
    bool valid_cmd;

    //Check precondition of inputs:
    if(!std::isfinite(dt) || !isfinite(curr_state) || !isfinite(path))
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[PathTrackControl] Received invalid inputs stopping.");
        if (std::isfinite(dt)) ROS_ERROR_STREAM("A");
        if (isfinite(curr_state)) ROS_ERROR_STREAM("B");
        if (isfinite(path)) ROS_ERROR_STREAM("C");
        look_ahead_pose = curr_odom;
        valid_cmd = false;
        return std::make_pair(command, valid_cmd);
    }

    // if empty path, do nothing 
    if(path.size() < 1)
    {
        ROS_INFO_STREAM_THROTTLE(5.0, "[PathTrackControl] Path does not contain any waypoints, no command issued");
        look_ahead_pose = curr_odom;
        valid_cmd = true;
        return std::make_pair(command, valid_cmd);
    }

    bool is_near_end, is_sharp_corner;
    XYZVPsi closest_state = project_odom_on_path(curr_state, path, control_state.closest_idx);
    //ROS_ERROR_STREAM_THROTTLE(1.0, "***angular_velocity closest state: " << closest_state.position[0] << " / " << closest_state.position[1] << " / " << closest_state.position[2]); 

    //This is the state we will control to. This looks ahead based on the current speed to account for control reaction delays.
    double speed = curr_state.linear_velocity.norm();

    //lookahead distance - minimal value is 1.0 meter (@TODO: make it configurable?)
    double look_ahead_dist = std::max(1.0, params_.look_ahead_time * speed);

    std::pair<XYZVPsi, Eigen::Vector3d> pursuit_state_pair = get_pursuit_state_pair(path, control_state.closest_idx, look_ahead_dist, is_near_end);
    XYZVPsi pursuit_state = pursuit_state_pair.first;
    //ROS_ERROR_STREAM_THROTTLE(1.0, "*** pursuit state first: " << pursuit_state.position[0] << " / " << pursuit_state.position[1] << " / " << pursuit_state.position[2]);
    //ROS_ERROR_STREAM_THROTTLE(1.0, "*** pursuit state second: " << pursuit_state_pair.second[0] << " / " << pursuit_state_pair.second[1] << " / " << pursuit_state_pair.second[2]);

    //Next we also look up if we need to slow down based on our maximum acceleration.
    double stoppingDistance =  get_stopping_distance(speed);

    std::pair<double, bool> dist_is_sharp_pair = get_dist_to_end_and_is_sharp_corner(path, control_state.closest_idx, 5.0 + stoppingDistance);//Added an offset to prevent problems at low speed

    double dist_to_end = dist_is_sharp_pair.first;
    is_sharp_corner = dist_is_sharp_pair.second;
    //max speed - configurable parameter
    double target_speed = std::max(params_.max_speed, pursuit_state.vel);

    if (is_near_end)
    {
        double max_desired_speed = std::min(target_speed, get_stopping_speed(dist_to_end));

        if(max_desired_speed < 1.0 || is_sharp_corner)
        {
            double az = closest_state.position[2];
            closest_state = pursuit_state;
            closest_state.position[2] = az;
        }
        pursuit_state.vel = max_desired_speed;
    }

    Eigen::Vector3d desired_velocity = pursuit_state.vel * pursuit_state_pair.second;
    //ROS_ERROR_STREAM_THROTTLE(1.0, "*** desired_velocity: " << desired_velocity[0] << " / " << desired_velocity[1] << " / " << desired_velocity[2]); 
    Eigen::Vector3d curr_to_closest_vec = closest_state.position - curr_state.position;
    //ROS_ERROR_STREAM_THROTTLE(1.0, "*** curr_to_closest_vec: " << curr_to_closest_vec[0] << " / " << curr_to_closest_vec[1] << " / " << curr_to_closest_vec[2]); 

    if (curr_to_closest_vec.norm() < 0.1) // todo unhardcode
    {
        curr_to_closest_vec = pursuit_state.position - curr_state.position;
        //ROS_ERROR_STREAM_THROTTLE(1.0, "*** curr_to_closest_vec: " << curr_to_closest_vec[0] << " / " << curr_to_closest_vec[1] << " / " << curr_to_closest_vec[2]); 
    }

    double z_error = curr_to_closest_vec[2];
    if (z_error < 0.05)
    {
        z_error = pursuit_state.position[2] - curr_state.position[2];
    }

    //Separate x,y from z control
    if(curr_to_closest_vec.norm() > params_.tracking_threshold)
    {
        ROS_WARN_STREAM_THROTTLE(5.0, "[PathTrackControl] Greater than" << params_.tracking_threshold << "meters from path, no command issued");
        look_ahead_pose = curr_odom;
        valid_cmd = true;
        return std::make_pair(command, valid_cmd);
    }

    curr_to_closest_vec[2] = 0;
    if(desired_velocity.norm() > params_.max_speed)
    {
        ROS_WARN_STREAM_THROTTLE(5.0, "[PathTrackControl] Commanded path exceeds maximum allowed speed"<<desired_velocity.norm()<<" > "<<params_.max_speed);
        desired_velocity.normalize();
        desired_velocity *= params_.max_speed;
    }


    Eigen::Vector3d path_tangent = desired_velocity;
    if (path_tangent.norm())
        path_tangent.normalize();
    Eigen::Vector3d curr_to_path = curr_to_closest_vec - path_tangent * (path_tangent.dot(curr_to_closest_vec));
    Eigen::Vector3d path_normal = curr_to_path;
    if (path_normal.norm() > 0)
        path_normal.normalize();
    double cross_track_error = curr_to_closest_vec.dot(path_normal);
    double cross_track_error_d = cross_track_error - control_state.prev_cross_track_error;


    control_state.cross_track_integrator += cross_track_error*dt;
    control_state.cross_track_integrator  = Limit(params_.cross_track_IMax, control_state.cross_track_integrator);

    double u_cross_track = params_.cross_track_P * cross_track_error +
                         params_.cross_track_I * control_state.cross_track_integrator +
                         params_.cross_track_D * cross_track_error_d/dt;

    control_state.prev_cross_track_error = cross_track_error;

    Eigen::Vector3d cmdVel = desired_velocity +  u_cross_track * path_normal;

    //Do separate terms for the z control:
    double z_trackerror =   z_error - control_state.prev_z_track_error;
    control_state.prev_z_track_error = z_error;

    double ztrack = params_.cross_track_PZ * z_error + params_.cross_track_DZ * z_trackerror/dt;
    cmdVel[2] = desired_velocity[2] + ztrack;
    if(cmdVel.norm() > target_speed)
        cmdVel *= (target_speed/cmdVel.norm());
 
    //Set command velocity and heading   
    command.velocity = cmdVel;
    command.heading = pursuit_state.heading;

    //Check output invariants:
    bool command_isfinite = isfinite(command);
    if(!command_isfinite)
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[PathTrackControl] Calculated invalid command. Going to velocity hold.");
        command.velocity = Eigen::Vector3d::Zero();
        command.heading = curr_state.rpy[2];
        look_ahead_pose = curr_odom;
        valid_cmd = true;
        return std::make_pair(command, valid_cmd);
    }

    look_ahead_pose.pose.pose.position.x = pursuit_state.position[0];
    look_ahead_pose.pose.pose.position.y = pursuit_state.position[1];
    look_ahead_pose.pose.pose.position.z = pursuit_state.position[2];
    valid_cmd = true;
    return std::make_pair(command, valid_cmd);
}

XYZVPsi PathTrackingControl::interpolate(const XYZVPsi &start_waypoint, const XYZVPsi& end_waypoint, const double &interp_ratio)
{
    XYZVPsi interp_xyzpsi;
    interp_xyzpsi.position = (start_waypoint.position * (1.0 - interp_ratio)) + (end_waypoint.position * interp_ratio);
    interp_xyzpsi.vel = (start_waypoint.vel * (1.0 - interp_ratio)) + (end_waypoint.vel * interp_ratio);
    interp_xyzpsi.heading = (start_waypoint.heading * (1.0 - interp_ratio)) + (end_waypoint.heading * interp_ratio);
    return interp_xyzpsi;
}

XYZVPsi PathTrackingControl::sample_path(const PathXYZVPsi &path, const double &float_idx)
{
    XYZVPsi point;
    int prev_int = std::floor(float_idx);
    int next_int = std::ceil(float_idx);

    // last waypoint on path
    if (next_int > (int) path.size() - 1)
        return point;

    XYZVPsi prev_waypoint = path[prev_int];
    XYZVPsi next_waypt = path[next_int];
    double interp_ratio = float_idx - (double)prev_int;

    point = interpolate(prev_waypoint, next_waypt, interp_ratio);
    return point;
}

// updates closest_idx_float! ugly API
XYZVPsi PathTrackingControl::project_odom_on_path(const OdometryEuler &odom, const PathXYZVPsi &path, double &closest_idx_float)
{
    XYZVPsi new_state;
    double MAX_DOUBLE = std::numeric_limits<double>::max(); // todo class member

    int path_size = path.size();
    if (path_size == 0)
    {
       throw std::runtime_error("path size is 0");
    }

    double min_idx_float = 0.0;
    int start_idx = 0;
    min_idx_float = closest_idx_float;
    start_idx = std::floor(min_idx_float);
    start_idx = std::min(start_idx, path_size - 1);
    new_state = path.back(); // initialize to last waypoint

    if(path_size == 1) 
    {
        min_idx_float = 0;
        new_state = path[0];
    } 
    else if(path_size > 1) 
    {
        for(int curr_idx = start_idx; curr_idx < path_size-1; curr_idx++) 
        {
            XYZVPsi curr_waypt =  path[curr_idx];
            XYZVPsi next_waypt =  path[curr_idx + 1];
            std::pair<Eigen::Vector3d, double> vec_rv_pair = vector_to_line_segment(curr_waypt.position, next_waypt.position, odom.position);
            Eigen::Vector3d vec =  vec_rv_pair.first;
            double rv = vec_rv_pair.second;
            double len = vec.norm();

            // Take the closest point
            if (len < MAX_DOUBLE)
            {
                MAX_DOUBLE = len;

                if(rv < 0.0) rv = 0.0;
                if(rv > 1.0) rv = 1.0;

                min_idx_float = curr_idx + rv;
                new_state = interpolate(curr_waypt, next_waypt, rv);
            }
            else if (MAX_DOUBLE != std::numeric_limits<double>::max())
            {
                //If it starts to get points that are not closer, then stops searching
                //Use the first closest point - don't scan the whole trajectory
                if(len >= MAX_DOUBLE)
                  break;
            }
        }
    }

    //ROS_INFO_STREAM("[PathTrackControl] min_idx_float=" << min_idx_float << " / closest_idx_float=" << closest_idx_float << " / total size=" << ts);
    closest_idx_float = min_idx_float;
    return new_state;
}

double PathTrackingControl::get_stopping_distance(const double &curr_speed)
{
    return (curr_speed * params_.reaction_time) + ((curr_speed * curr_speed) / (2.0 * params_.max_deceleration)); // (v*params_.reaction_time) + (v^2 / 2a)
}

// todo how is this calculated?
double PathTrackingControl::get_stopping_speed(const double &stopping_distance)
{
    double sf1 = 2 * params_.max_deceleration * stopping_distance;
    double sf2 = params_.max_deceleration * params_.max_deceleration * params_.reaction_time;
    return std::max(0.0, -params_.max_deceleration * params_.reaction_time + std::sqrt(sf1 + sf2));
}

// todo better name and signature
std::pair<double, bool> PathTrackingControl::get_dist_to_end_and_is_sharp_corner(const PathXYZVPsi &path, const double &idx_float, const double &max_lookahead_dist)
{
    bool is_sharp_corner = false;

    // get int index of next waypoint
    int next_idx = std::ceil(idx_float);
    if(next_idx == idx_float)
        next_idx++;
    next_idx = std::min(next_idx, (int)path.size()-1);

    XYZVPsi next_waypt = path[next_idx];
    XYZVPsi curr_waypt = sample_path(path, idx_float);

    double dist_remaining = max_lookahead_dist;
    bool is_at_last_waypt = false;

    Eigen::Vector3d prev_vector = next_waypt.position - curr_waypt.position;
    if (prev_vector.norm() > 0)
      prev_vector.normalize();

    while(!is_at_last_waypt && dist_remaining > 0)
    {
        Eigen::Vector3d curr_vector = next_waypt.position - curr_waypt.position;
        double dist_on_seg = curr_vector.norm();
        if (curr_vector.norm() > 0)
            curr_vector.normalize();
        double curr_delta_yaw = std::fabs(std::acos(std::min(1.0, curr_vector.dot(prev_vector))));
        prev_vector = curr_vector;
        if(curr_delta_yaw > params_.look_ahead_angle)
        {
            bool is_sharp_corner = true;
            is_at_last_waypt = true;
        }
        if(dist_on_seg > dist_remaining)
        {
            dist_remaining = 0;
        }
        else
        {
            dist_remaining -= dist_on_seg;
            if(next_idx == (int)path.size()-1)
            {
                is_at_last_waypt = true;
            }
            else
            {
                curr_waypt = next_waypt;
                next_idx++;
                next_waypt = path[next_idx];
            }
        }
    }

    return std::make_pair(max_lookahead_dist - dist_remaining, is_sharp_corner);
}

// todo understand. also return std::tuple<XYZVPsi, Eigen::Vector3d, bool>
std::pair<XYZVPsi, Eigen::Vector3d> PathTrackingControl::get_pursuit_state_pair(const PathXYZVPsi &path, double idx_float, double look_ahead_dist, bool &at_end)
{

    int next_idx = std::ceil(idx_float);
    if ((next_idx > idx_float) && (std::fabs(next_idx - idx_float) < 0.05)) // todo hardcoded. and why only here
        next_idx++;

    int path_size = path.size();

    // if at end of path
    if(next_idx > (path_size - 1))
    {
        at_end = true;
        Eigen::Vector3d pursuit_vec = path[std::max(0,path_size-1)].position - path[std::max(0,path_size-2)].position;
        if (pursuit_vec.norm() > 0)
            pursuit_vec.normalize();
        return std::make_pair(path.back(), pursuit_vec);
    }

    //ROS_INFO_STREAM_THROTTLE(1.0, ">>> look_ahead_dist=" << look_ahead_dist);

    //If next_idx is too close to current idx_float, then increase index value
    XYZVPsi next_waypt = path[next_idx];
    XYZVPsi curr_waypt = sample_path(path, idx_float);
    Eigen::Vector3d delta = next_waypt.position - curr_waypt.position;
    while ((delta.norm() < 0.1) && (next_idx < (path_size-1))) // todo hard coding
    {
        next_idx++;
        next_waypt = path[next_idx];
        delta = next_waypt.position - curr_waypt.position;
    }

    //ROS_INFO_STREAM_THROTTLE(1.0, ">>> idx_float=" << idx_float << " / next_idx=" << next_idx << " / delta=" << delta.norm() << " / total size=" << path_size);
  
    XYZVPsi pursuit_state = curr_waypt;
    Eigen::Vector3d pursuit_vec = Eigen::Vector3d::Zero();

    double dist_remaining = look_ahead_dist;
    bool bAtEnd = false;

    Eigen::Vector3d prev_vector = next_waypt.position - curr_waypt.position;
    if (prev_vector.norm() > 0)
        prev_vector.normalize();

    while(!bAtEnd && dist_remaining > 0)
    {
        Eigen::Vector3d curr_vector = next_waypt.position - curr_waypt.position;
        double dist_on_seg = curr_vector.norm();
        if (curr_vector.norm() > 0)
            curr_vector.normalize();

        double curr_psi_offset = std::fabs(std::acos(std::min(1.0,curr_vector.dot(prev_vector))));
        prev_vector = curr_vector;

        if(curr_psi_offset >= params_.look_ahead_angle)
        {
            pursuit_state = path[std::max(0, next_idx-1)];
            pursuit_vec = path[std::max(0, next_idx-1)].position - path[std::max(0, next_idx-2)].position;
            //ROS_INFO_STREAM_THROTTLE(1.0, ">>> angle: curr_psi_offset=" << curr_psi_offset << " / params_.look_ahead_angle=" << params_.look_ahead_angle);    
            break;
        }

        if(dist_on_seg > dist_remaining)
        {
            double prog = dist_remaining / dist_on_seg;
            pursuit_state = interpolate(curr_waypt, next_waypt, prog);
            pursuit_vec = next_waypt.position - curr_waypt.position;
            dist_remaining = 0;
        } 
        else
        {
            if(next_idx == (int)path.size()-1)
            {
                bAtEnd = true;
                pursuit_state = path.back();
                pursuit_vec = path[std::max(0,(int)path.size()-1)].position - path[std::max(0,(int)path.size()-2)].position;
                //ROS_INFO_STREAM_THROTTLE(1.0, ">>> at the end");    
            } 
            else 
            {
                curr_waypt = next_waypt;
                next_idx++;
                dist_remaining -= dist_on_seg;
                next_waypt = path[next_idx];
            }
        }
        //ROS_INFO_STREAM_THROTTLE(1.0, ">>> dist_remanining=" << dist_remaining);
    }

    at_end = bAtEnd;
    if (pursuit_vec.norm() > 0)
        pursuit_vec.normalize();

    return std::make_pair(pursuit_state, pursuit_vec);
}

bool PathTrackingControl::isfinite(const Eigen::Vector3d &v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
};

bool PathTrackingControl::isfinite(const OdometryEuler &o)
{
    return isfinite(o.position) && isfinite(o.rpy) && isfinite(o.linear_velocity) && isfinite(o.angular_velocity);
}

// todo c++11
bool PathTrackingControl::isfinite(const PathXYZVPsi &p)
{
    for (std::vector<XYZVPsi>::const_iterator it = p.begin(); it != p.end(); ++it)
        if (!(isfinite(it->position) && std::isfinite(it->vel)))
          return false;
    return true;
}

bool PathTrackingControl::isfinite(const VelControlCmd &p)
{
  return isfinite(p.velocity) && std::isfinite(p.heading);
}
