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

// todo return std::pair 
std::pair<Eigen::Vector3d, double> PathTrackingControlParameters::vector_to_line_segment(const Eigen::Vector3d &curr_waypt, const Eigen::Vector3d &next_waypt, const Eigen::Vector3d &odom);
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

    return std::make_pair<Eigen::Vector3d, double>(result, r);
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
    found = found && nh.getParam("deccel_max", deccel_max);
    found = found && nh.getParam("reaction_time", reaction_time);
    return found;
}


XYZVPsi PathTrackingControl::interpolate(const XYZVPsi &start_waypoint, const XYZVPsi& end_waypoint, const double &interp_ratio)
{
    XYZVPsi interp_xyzpsi;
    interp_xyzpsi.position = (start_waypoint.position * (1.0 - interp_ratio)) + (end_waypoint.position * interp_ratio);
    interp_xyzpsi.vel = (start_waypoint.vel * (1.0 - interp_ratio)) + (end_waypoint.vel * interp_ratio);
    interp_xyzpsi.heading = (start_waypoint.heading * (1.0 - interp_ratio)) + (end_waypoint.heading * interp_ratio);
    return interp_xyzpsi;
}

XYZVPsi sample_path(const PathXYZVPsi &path, const double &float_idx) const
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

XYZVPsi PathTrackingControl::project_odom_on_path(const OdometryEuler &odom, const PathXYZVPsi &path, double &closest_idx_float)
{
    XYZVPsi new_state;
    double MAX_DOUBLE = std::numeric_limits<double>::max(); // todo class member

    unsigned path_size = path.size();
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
            Eigen::Vector3d vec;
            XYZVPsi curr_waypt =  path[curr_idx];
            XYZVPsi next_waypt =  path[curr_idx + 1];
            std::pair<Eigen::Vector3d, double> vec_rv_pair = vector_to_line_segment(curr_waypt.position, next_waypt.position, odom.position);
            Eigen::Vector3d vec =  vec_rv_pair.first;
            doubl rv = vec_rv_pair.second;
            double len = vec.norm();

            // Take the closest point
            if (len < MAX_DOUBLE)
            {
                MAX_DOUBLE = len;

                if(rv < 0.0) rv = 0.0;
                if(rv > 1.0) rv = 1.0;

                min_idx_float = curr_idx + rv;
                new_state = Interpolate(curr_waypt, next_waypt, rv);
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

double PathTrackingControl::get_stopping_distance(const double &max_deceleration, const double &reaction_time, const double &curr_speed)
{
    return (curr_speed * reaction_time) + ((curr_speed * curr_speed) / (2.0 * max_deceleration)); // (v*reaction_time) + (v^2 / 2a)
}

// todo how is this calculated?
double PathTrackingControl::get_stopping_speed(const double &max_deceleration, const double &reaction_time, const double &distance)
{
    double sf1 = 2 * max_deceleration * distance;
    double sf2 = max_deceleration * max_deceleration * reaction_time;
    return std::max(0.0, -max_deceleration * reaction_time + std::sqrt(sf1 + sf2));
}


// todo better name and signature
std::pair<double, bool> PathTrackingControl::get_dist_to_end_and_is_sharp_corner(const PathXYZVPsi &path, const double &idx_float, const double &max_lookahead_dist);
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

    return std::make_pair<double, bool>(max_lookahead_dist - dist_remaining, is_sharp_corner);
}

// todo understand. also return std::tuple<XYZVPsi, Eigen::Vector3d, bool>
std::pair<XYZVPsi, Eigen::Vector3d> PathTrackingControl::get_pursuit_state_pair(const PathXYZVPsi &path, double idx_float, double look_head_dist, double max_angle_rad, bool &at_end);
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

    //ROS_INFO_STREAM_THROTTLE(1.0, ">>> look_head_dist=" << look_head_dist);

    //If next_idx is too close to current idx_float, then increase index value
    XYZVPsi next_state = path[next_idx];
    XYZVPsi curr_state = sample_path(path, idx_float);
    Eigen::Vector3d delta = next_state.position - curr_state.position;
    while ((delta.norm() < 0.1) && (next_idx < (path_size-1))) // todo hard coding
    {
        next_idx++;
        next_state = path[next_idx];
        delta = next_state.position - curr_state.position;
    }

    //ROS_INFO_STREAM_THROTTLE(1.0, ">>> idx_float=" << idx_float << " / next_idx=" << next_idx << " / delta=" << delta.norm() << " / total size=" << path_size);
  
    XYZVPsi pursuit_state = curr_state;
    Eigen::Vector3d pursuit_vec = Eigen::Vector3d::Zero();

    double dist_remaining = look_head_dist;
    bool bAtEnd = false;

    Eigen::Vector3d prev_vector = next_state.position - curr_state.position;
    if (prev_vector.norm() > 0)
        prev_vector.normalize();

    while(!bAtEnd && dist_remaining > 0)
    {
        Eigen::Vector3d curr_vector = next_state.position - curr_state.position;
        double dist_on_seg = curr_vector.norm();
        if (curr_vector.norm() > 0)
            curr_vector.normalize();

        double curr_psi_offset = std::fabs(std::acos(std::min(1.0,curr_vector.dot(prev_vector))));
        prev_vector = curr_vector;

        if(curr_psi_offset >= max_angle_rad)
        {
            pursuit_state = path[std::max(0, next_idx-1)];
            pursuit_vec = path[std::max(0, next_idx-1)].position - path[std::max(0, next_idx-2)].position;
            //ROS_INFO_STREAM_THROTTLE(1.0, ">>> angle: curr_psi_offset=" << curr_psi_offset << " / max_angle_rad=" << max_angle_rad);    
            break;
        }

        if(dist_on_seg > dist_remaining)
        {
            double prog = dist_remaining / dist_on_seg;
            pursuit_state = Interpolate(curr_state, next_state, prog);
            pursuit_vec = next_state.position - curr_state.position;
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
                curr_state = next_state;
                next_idx++;
                dist_remaining -= dist_on_seg;
                next_state = path[next_idx];
            }
        }
        //ROS_INFO_STREAM_THROTTLE(1.0, ">>> dist_remanining=" << dist_remaining);
    }

    at_end = bAtEnd;
    if (pursuit_vec.norm() > 0)
        pursuit_vec.normalize();

    return std::make_pair(pursuit_state, pursuit_vec);
}

bool PathTrackingControl::is_finite(const Eigen::Vector3d &v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
};

bool PathTrackingControl::is_finite(const OdometryEuler &o)
{
    return is_finite(o.position) && is_finite(o.orientation) && is_finite(o.linear_velocity) && is_finite(o.angular_velocity);
}

// todo c++11
bool PathTrackingControl::is_finite(const PathXYZVPsi &p)
{
    for (std::vector<XYZVPsi>::const_iterator it = p.path.begin(); it != p.path.end(); ++it)
        if (!(is_finite(it->position) && std::isfinite(it->vel)))
          return false;
    return true;
}

bool PathTrackingControl::is_finite(const VelAccPsi &p)
{
  return is_finite(p.velocity) && std::isfinite(p.heading) && std::isfinite(p.heading_rate);
}
