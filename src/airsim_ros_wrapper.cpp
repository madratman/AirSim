#include <airsim_ros_wrapper.h>

constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

AirsimROSWrapper::AirsimROSWrapper(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), it_(nh_)
{
    initialize_airsim();
    initialize_ros();

    // intitialize placeholder control commands
    // vel_cmd_ = VelCmd();
    // gimbal_cmd_ = GimbalCmd();
}

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
    front_left_img_raw_pub_ = it_.advertise("front/left/image_raw", 1);
    front_right_img_raw_pub_ = it_.advertise("front/right/image_raw", 1);
    front_left_depth_planar_pub_ = it_.advertise("front/left/depth_planar", 1);

    // nh_private_.getParam("front_left_calib_file_", front_left_calib_file_);
    // nh_private_.getParam("front_right_calib_file_", front_right_calib_file_);

    // front_left_calib_file_ = "/mnt/c/Users/ramadaan/projects/AirSim_wsl/airsim_roscpp_ws/src/airsim_ros_pkgs/calib/front_left.yaml";
    // front_right_calib_file_ = "/mnt/c/Users/ramadaan/projects/AirSim_wsl/airsim_roscpp_ws/src/airsim_ros_pkgs/calib/front_right.yaml";
    front_left_calib_file_ = "/home/madratman/projects/airsim_roscpp_ws/src/airsim_roscpp_pkgs/calib/front_left.yaml";
    front_right_calib_file_ = "/home/madratman/projects/airsim_roscpp_ws/src/airsim_roscpp_pkgs/calib/front_right.yaml";

    // fill camera info msg from YAML calib file. todo error check path
    read_params_from_yaml_and_fill_cam_info_msg(front_left_calib_file_, front_left_cam_info_msg_);
    front_left_cam_info_msg_.header.frame_id = "airsim/front/left";

    read_params_from_yaml_and_fill_cam_info_msg(front_right_calib_file_, front_right_cam_info_msg_);
    front_right_cam_info_msg_.header.frame_id = "airsim/front/right";

    takeoff_srvr_ = nh_private_.advertiseService("takeoff", &AirsimROSWrapper::takeoff_srv_cb, this);
    land_srvr_ = nh_private_.advertiseService("land", &AirsimROSWrapper::land_srv_cb, this);
    reset_srvr_ = nh_private_.advertiseService("reset",&AirsimROSWrapper::reset_srv_cb, this);

    // clock_pub_ = nh_private_.advertise<rosgraph_msgs::Clock>("clock", 10); // mimic gazebo's /use_sim_time feature
    vehicle_state_pub_ = nh_private_.advertise<mavros_msgs::State>("vehicle_state", 10);
    odom_local_ned_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom_local_ned", 10);
    global_gps_pub_ = nh_private_.advertise<sensor_msgs::NavSatFix>("global_gps", 10);

    front_left_cam_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo> ("front/left/camera_info", 10);
    front_right_cam_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo> ("front/right/camera_info", 10);

    vel_cmd_body_frame_sub_ = nh_private_.subscribe("vel_cmd_body_frame", 50, &AirsimROSWrapper::vel_cmd_body_frame_cb, this); // todo ros::TransportHints().tcpNoDelay();
    vel_cmd_world_frame_sub_ = nh_private_.subscribe("vel_cmd_world_frame", 50, &AirsimROSWrapper::vel_cmd_world_frame_cb, this);
    gimbal_angle_quat_cmd_sub_ = nh_private_.subscribe("gimbal_angle_quat_cmd", 50, &AirsimROSWrapper::gimbal_angle_quat_cmd_cb, this);
    gimbal_angle_euler_cmd_sub_ = nh_private_.subscribe("gimbal_angle_euler_cmd", 50, &AirsimROSWrapper::gimbal_angle_euler_cmd_cb, this);

    double update_airsim_img_response_every_n_sec = 0.0001;
    double update_airsim_control_every_n_sec = 0.01;
    // nh_private_.param("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec, update_airsim_img_response_every_n_sec);
    airsim_img_response_timer_ = nh_private_.createTimer(ros::Duration(update_airsim_img_response_every_n_sec), &AirsimROSWrapper::img_response_timer_cb, this);
    airsim_control_update_timer_ = nh_private_.createTimer(ros::Duration(update_airsim_control_every_n_sec), &AirsimROSWrapper::drone_state_timer_cb, this);
}

void AirsimROSWrapper::init_path_controller()
{
    //Set default period as 50Hz
    path_tracking_dt_ = 1.0/50.0;

    // todo path tracking should be a differnt node itself
    if(!controller_params_.load_params(nh_private_))
    {
        ROS_ERROR_STREAM("[AirsimROSWrapper] Failed to load path control parameters");
        return;
    }

    // Initialize path tracking control
    controller_ = new PathTrackingControl(controller_params_);

    // if (!_speed_control_params.load_params(nh_private_))
    // {
    //     ROS_ERROR_STREAM("[AirsimROSWrapper] Failed to load speed control parameters");
    //     return;
    // }

    has_path_ = false;
    is_tracking_path_ = false;
    is_path_completed_ = false;

    path_xyzvpsi_ = PathXYZVPsi(); 
    path_controller_speed_cmd_ = VelControlCmd();

    // _traj_start_time = 0.0;
    waypoint_float_idx_ = -1.0;
}

// todo minor: error check. if state is not landed, return error. 
bool AirsimROSWrapper::takeoff_srv_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.takeoffAsync()->waitOnLastTask();
    return true; //todo
}

// todo minor: error check. if state is not in air, return error. 
bool AirsimROSWrapper::land_srv_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.landAsync()->waitOnLastTask();
    return true; //todo
}

bool AirsimROSWrapper::reset_srv_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    airsim_client_.reset();
    return true; //todo
}

tf2::Quaternion AirsimROSWrapper::get_tf2_quat(const msr::airlib::Quaternionr& airlib_quat)
{
    return tf2::Quaternion(airlib_quat.x(), airlib_quat.y(), airlib_quat.z(), airlib_quat.w());
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const geometry_msgs::Quaternion& geometry_msgs_quat)
{
    return msr::airlib::Quaternionr(geometry_msgs_quat.w, geometry_msgs_quat.x, geometry_msgs_quat.y, geometry_msgs_quat.z); 
}

msr::airlib::Quaternionr AirsimROSWrapper::get_airlib_quat(const tf2::Quaternion& tf2_quat)
{
    return msr::airlib::Quaternionr(tf2_quat.w(), tf2_quat.x(), tf2_quat.y(), tf2_quat.z()); 
}

void AirsimROSWrapper::vel_cmd_body_frame_cb(const airsim_ros_pkgs::VelCmd &msg)
{
    double roll, pitch, yaw;
    auto drone_state = airsim_client_.getMultirotorState(); // todo use the state from drone state timer callback
    tf2::Matrix3x3(get_tf2_quat(drone_state.kinematics_estimated.pose.orientation)).getRPY(roll, pitch, yaw); // ros uses xyzw

    // todo do actual body frame?
    vel_cmd_.x = (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw)); //body frame assuming zero pitch roll
    vel_cmd_.y = (msg.twist.linear.x * sin(yaw)) + (msg.twist.linear.y * cos(yaw)); //body frame
    vel_cmd_.z = msg.twist.linear.z;
    vel_cmd_.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    // vel_cmd_.yaw_mode = msr::airlib::YawMode(true, msg.twist.angular.z);
    vel_cmd_.yaw_mode.is_rate = true;
    vel_cmd_.yaw_mode.yaw_or_rate = msg.twist.angular.z;
    vel_cmd_.vehicle_name = msg.vehicle_name;
    has_vel_cmd_ = true;
}

void AirsimROSWrapper::vel_cmd_world_frame_cb(const airsim_ros_pkgs::VelCmd &msg)
{
    vel_cmd_.x = msg.twist.linear.x;
    vel_cmd_.y = msg.twist.linear.y;
    vel_cmd_.z = msg.twist.linear.z;
    vel_cmd_.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    // vel_cmd_.yaw_mode = msr::airlib::YawMode(true, msg.twist.angular.z);
    vel_cmd_.yaw_mode.is_rate = true;
    vel_cmd_.yaw_mode.yaw_or_rate = msg.twist.angular.z;
    vel_cmd_.vehicle_name = msg.vehicle_name;
    has_vel_cmd_ = true;
}

// todo either maintain list of paths or make this a ros service. better ros action
void AirsimROSWrapper::path_cb(const airsim_ros_pkgs::PathXYZVPsi &path_msg)
{
    path_xyzvpsi_ = PathXYZVPsi();

    for (auto waypoint_xvzpsi : path_msg.waypoints)
    {
        XYZVPsi xyzvpsi;
        xyzvpsi.position[0] = waypoint_xvzpsi.position.x;
        xyzvpsi.position[1] = waypoint_xvzpsi.position.y;
        xyzvpsi.position[2] = waypoint_xvzpsi.position.z;
        xyzvpsi.heading     = waypoint_xvzpsi.heading;
        xyzvpsi.vel         = waypoint_xvzpsi.vel;
        path_xyzvpsi_.push_back(xyzvpsi);        
    }

    has_path_ = true;
    is_path_completed_ = false;
}

void AirsimROSWrapper::gimbal_angle_quat_cmd_cb(const airsim_ros_pkgs::GimbalAngleQuatCmd &gimbal_angle_quat_cmd_msg)
{
    // airsim uses wxyz
    // todo error check
    gimbal_cmd_.target_quat = get_airlib_quat(gimbal_angle_quat_cmd_msg.orientation);
    gimbal_cmd_.camera_name = gimbal_angle_quat_cmd_msg.camera_name;
    gimbal_cmd_.vehicle_name = gimbal_angle_quat_cmd_msg.vehicle_name;
    has_gimbal_cmd_ = true; 
}

void AirsimROSWrapper::gimbal_angle_euler_cmd_cb(const airsim_ros_pkgs::GimbalAngleEulerCmd &gimbal_angle_euler_cmd_msg)
{
    // airsim uses wxyz
    tf2::Quaternion tf2_quat(gimbal_angle_euler_cmd_msg.yaw, gimbal_angle_euler_cmd_msg.pitch, gimbal_angle_euler_cmd_msg.roll);
    gimbal_cmd_.target_quat = get_airlib_quat(tf2_quat);
    gimbal_cmd_.camera_name = gimbal_angle_euler_cmd_msg.camera_name;
    gimbal_cmd_.vehicle_name = gimbal_angle_euler_cmd_msg.vehicle_name;
    has_gimbal_cmd_ = true; 
}

// todo to pass param and fill, or return value. 
nav_msgs::Odometry AirsimROSWrapper::get_odom_msg_from_airsim_state(const msr::airlib::MultirotorState &drone_state)
{
    nav_msgs::Odometry odom_ned_msg;
    // odom_ned_msg.header.frame = ;
    odom_ned_msg.child_frame_id = "/airsim/odom_local_ned"; // todo make param

    odom_ned_msg.pose.pose.position.x = drone_state.getPosition().x();
    odom_ned_msg.pose.pose.position.y = drone_state.getPosition().y();
    odom_ned_msg.pose.pose.position.z = drone_state.getPosition().z();
    odom_ned_msg.pose.pose.orientation.x = drone_state.getOrientation().x();
    odom_ned_msg.pose.pose.orientation.y = drone_state.getOrientation().y();
    odom_ned_msg.pose.pose.orientation.z = drone_state.getOrientation().z();
    odom_ned_msg.pose.pose.orientation.w = drone_state.getOrientation().w();

    odom_ned_msg.twist.twist.linear.x = drone_state.kinematics_estimated.twist.linear.x();
    odom_ned_msg.twist.twist.linear.y = drone_state.kinematics_estimated.twist.linear.y();
    odom_ned_msg.twist.twist.linear.z = drone_state.kinematics_estimated.twist.linear.z();
    odom_ned_msg.twist.twist.angular.x = drone_state.kinematics_estimated.twist.angular.x();
    odom_ned_msg.twist.twist.angular.y = drone_state.kinematics_estimated.twist.angular.y();
    odom_ned_msg.twist.twist.angular.z = drone_state.kinematics_estimated.twist.angular.z();

    return odom_ned_msg;
}

void AirsimROSWrapper::publish_odom_tf(const nav_msgs::Odometry &odom_ned_msg)
{
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header = odom_ned_msg.header;
    odom_tf.child_frame_id = "drone";
    odom_tf.transform.translation.x = odom_ned_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_ned_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_ned_msg.pose.pose.position.z;
    odom_tf.transform.rotation.x = odom_ned_msg.pose.pose.orientation.x;
    odom_tf.transform.rotation.y = odom_ned_msg.pose.pose.orientation.y;
    odom_tf.transform.rotation.z = odom_ned_msg.pose.pose.orientation.z;
    odom_tf.transform.rotation.w = odom_ned_msg.pose.pose.orientation.w;
    tf_broadcaster_.sendTransform(odom_tf);
}

sensor_msgs::NavSatFix AirsimROSWrapper::get_gps_msg_from_airsim_state(const msr::airlib::MultirotorState &drone_state)
{
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.latitude = drone_state.gps_location.latitude;
    gps_msg.longitude = drone_state.gps_location.longitude; 
    gps_msg.altitude = drone_state.gps_location.altitude;
    return gps_msg;
}

mavros_msgs::State AirsimROSWrapper::get_vehicle_state_msg(msr::airlib::MultirotorState &drone_state)
{
    mavros_msgs::State vehicle_state_msg;
    // vehicle_state_msg.connected = true; // not reqd
    vehicle_state_msg.armed = true; // todo is_armed_
    // vehicle_state_msg.guided; // not reqd 
    // vehicle_state_msg.mode; // todo
    // vehicle_state_msg.system_status; // not reqd
    return vehicle_state_msg;
}

// path_controller_speed_cmd_
VelControlCmd AirsimROSWrapper::get_safe_vel_cmd(const VelControlCmd &desired_vel_cmd)
{

}

void AirsimROSWrapper::update_path_tracker()
{
    
    nav_msgs::Odometry lookahead_pose;
    std::pair<VelControlCmd, bool> speed_cmd_is_valid_pair = controller_->get_vel_cmd(path_tracking_dt_, curr_odom_ned_, path_xyzvpsi_, controller_state_, lookahead_pose);
    path_controller_speed_cmd_ = speed_cmd_is_valid_pair.first;

    if (!speed_cmd_is_valid_pair.second)
    {
        ROS_INFO_STREAM("Speed command not valid!");
        return;
    }

    unsigned int curr_idx = (int)std::ceil(controller_state_.closest_idx);  
    int pathSize = path_xyzvpsi_.size();
    ROS_INFO_STREAM_THROTTLE(2.0, "[AirsimROSWrapper] Path Index=" << curr_idx << " [" << path_xyzvpsi_.size());

    //Check if reached end of trajectory command
    curr_idx = (unsigned int)std::round(controller_state_.closest_idx);
    if ((curr_idx+1) >= path_xyzvpsi_.size())
    {
        is_path_completed_ = true;
        // todo set has_path_ to false?
        ROS_INFO_STREAM_THROTTLE(1.0, "[AirsimROSWrapper] Path completed");
    }

    safe_speed_cmd_ = get_safe_vel_cmd(path_controller_speed_cmd_); // todo extract this into a safety module?

    // set velocity command to be sent to airsim in next call of drone_state_timer_cb
    vel_cmd_.x = safe_speed_cmd_.velocity[0];
    vel_cmd_.y = safe_speed_cmd_.velocity[1];
    vel_cmd_.z = safe_speed_cmd_.velocity[2];

    vel_cmd_.drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    // vel_cmd_.yaw_mode = msr::airlib::YawMode(true, msg.twist.angular.z);
    vel_cmd_.yaw_mode.is_rate = false;
    vel_cmd_.yaw_mode.yaw_or_rate = safe_speed_cmd_.heading;
    // vel_cmd_.vehicle_name = msg.vehicle_name; //todo
    has_vel_cmd_ = true;
}

void AirsimROSWrapper::drone_state_timer_cb(const ros::TimerEvent& event)
{
    // get drone state from airsim
    msr::airlib::MultirotorState drone_state = airsim_client_.getMultirotorState();
    ros::Time curr_ros_time = ros::Time::now();

    // convert airsim drone state to ROS msgs
    curr_odom_ned_ = get_odom_msg_from_airsim_state(drone_state);
    curr_odom_ned_.header.frame_id = "world";
    curr_odom_ned_.header.stamp = curr_ros_time;

    sensor_msgs::NavSatFix gps_msg = get_gps_msg_from_airsim_state(drone_state);
    gps_msg.header.stamp = curr_ros_time;

    mavros_msgs::State vehicle_state_msg = get_vehicle_state_msg(drone_state);

    // publish to ROS!  
    odom_local_ned_pub_.publish(curr_odom_ned_);
    publish_odom_tf(curr_odom_ned_);
    global_gps_pub_.publish(gps_msg);
    vehicle_state_pub_.publish(vehicle_state_msg);

    if(has_path_ && (!is_path_completed_))
    {
        update_path_tracker(); //sets velocity commands vel_cmd_
        // todo what if we have path tracker's vel_cmd_ and another vel_cmd_ set by velocity callback. 
        // define precedence. or define mode of operation? 
    }

    // send control commands from the last callback to airsim
    // airsim_client_.simSetCameraOrientation(gimbal_angle_cmd_msg.camera_name, orientation, gimbal_angle_cmd_msg.vehicle_name);
    if (has_vel_cmd_)
        airsim_client_.moveByVelocityAsync(vel_cmd_.x, vel_cmd_.y, vel_cmd_.z, vel_cmd_duration_, 
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom, vel_cmd_.yaw_mode);

    if (has_gimbal_cmd_)
        airsim_client_.simSetCameraOrientation(gimbal_cmd_.camera_name, gimbal_cmd_.target_quat, gimbal_cmd_.vehicle_name);

    // "clear" control cmds
    has_vel_cmd_ = false;
    has_gimbal_cmd_ = false;
}

void AirsimROSWrapper::img_response_timer_cb(const ros::TimerEvent& event)
{    
    std::vector<ImageRequest> img_request = { 
        ImageRequest("front_left", ImageType::Scene,false, false), 
        ImageRequest("front_right", ImageType::Scene, false, false), 
        ImageRequest("front_left", ImageType::DepthPlanner, true)
    };

    try
    {
        // std::cout << "AirsimROSWrapper::img_response_timer_cb\n";
        const std::vector<ImageResponse>& img_response = airsim_client_.simGetImages(img_request);
  
        if (img_response.size() == img_request.size()) 
        {
            
            // std::cout << "publishing now \n";
            process_and_publish_img_response(img_response);
        }
    }

    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl << msg << std::endl;
    }

}

cv::Mat AirsimROSWrapper::manual_decode_rgb(const ImageResponse &img_response)
{
    cv::Mat mat(img_response.height, img_response.width, CV_8UC3, cv::Scalar(0, 0, 0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
    {
        for (int col = 0; col < img_width; col++)
        {
            mat.at<cv::Vec3b>(row, col) = cv::Vec3b(
                img_response.image_data_uint8[row*img_width*4 + 4*col + 2],
                img_response.image_data_uint8[row*img_width*4 + 4*col + 1],
                img_response.image_data_uint8[row*img_width*4 + 4*col + 0]);
        }
    }
    return mat;
}

cv::Mat AirsimROSWrapper::manual_decode_depth(const ImageResponse &img_response)
{
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
        for (int col = 0; col < img_width; col++)
            mat.at<float>(row, col) = img_response.image_data_float[row*img_width + col];
    return mat;
}

void AirsimROSWrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response)
{
    // todo why is cv::imdecode not working
    // #if CV_MAJOR_VERSION==3
            // cv::Mat camera_0_img = cv::imdecode(img_response.at(0).image_data_uint8, cv::IMREAD_UNCHANGED);
            // auto rgb_right = cv::imdecode(img_response.at(1).image_data_uint8, cv::IMREAD_COLOR);
            // auto depth = cv::imdecode(img_response.at(2).image_data_uint8, cv::IMREAD_GRAYSCALE);
    // #else
    //     cv::Mat camera_0_img = cv::imdecode(img_response.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
    // #endif

    // decode images and convert to ROS image msgs
    cv::Mat bgr_front_left = manual_decode_rgb(img_response.at(0));
    sensor_msgs::ImagePtr bgr_front_left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_front_left).toImageMsg();

    cv::Mat bgr_front_right = manual_decode_rgb(img_response.at(1));
    sensor_msgs::ImagePtr bgr_front_right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_front_right).toImageMsg();

    cv::Mat front_left_depth_planar = manual_decode_depth(img_response.at(2));
    sensor_msgs::ImagePtr front_left_depth_planar_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", front_left_depth_planar).toImageMsg();

    // put ros time now in headers. 
    // TODO use airsim time if ros param /use_sim_time is set to true
    ros::Time curr_ros_time = ros::Time::now();
    bgr_front_left_msg->header.stamp = curr_ros_time;
    bgr_front_right_msg->header.stamp = curr_ros_time;
    front_left_depth_planar_msg->header.stamp = curr_ros_time;
    front_left_cam_info_msg_.header.stamp = curr_ros_time; // update timestamp of saved cam info msgs
    front_right_cam_info_msg_.header.stamp = curr_ros_time;

    // publish camera transforms
    std_msgs::Header tf_header;
    tf_header.stamp = curr_ros_time;
    tf_header.frame_id = "world";
    publish_camera_tf(img_response.at(0), tf_header, "airsim/cam_front_left");
    publish_camera_tf(img_response.at(1), tf_header, "airsim/cam_front_right");

    // publish everything
    front_right_img_raw_pub_.publish(bgr_front_right_msg);
    front_left_img_raw_pub_.publish(bgr_front_left_msg);
    front_left_depth_planar_pub_.publish(front_left_depth_planar_msg);
    front_left_cam_info_pub_.publish(front_left_cam_info_msg_);
    front_right_cam_info_pub_.publish(front_right_cam_info_msg_);
}

void AirsimROSWrapper::publish_camera_tf(const ImageResponse &img_response, const std_msgs::Header &header, const std::string &child_frame_id)
{
    geometry_msgs::TransformStamped cam_tf;
    cam_tf.header = header;
    cam_tf.child_frame_id = child_frame_id;
    cam_tf.transform.translation.x = img_response.camera_position.x();
    cam_tf.transform.translation.y = img_response.camera_position.y();
    cam_tf.transform.translation.z = img_response.camera_position.z();
    cam_tf.transform.rotation.x = img_response.camera_orientation.x();
    cam_tf.transform.rotation.y = img_response.camera_orientation.y();
    cam_tf.transform.rotation.z = img_response.camera_orientation.z();
    cam_tf.transform.rotation.w = img_response.camera_orientation.w();
    tf_broadcaster_.sendTransform(cam_tf);
}

void AirsimROSWrapper::convert_yaml_to_simple_mat(const YAML::Node& node, SimpleMatrix& m)
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
    {
        m.data[i] = data[i].as<double>();
    }
}

void AirsimROSWrapper::read_params_from_yaml_and_fill_cam_info_msg(const std::string& file_name, sensor_msgs::CameraInfo& cam_info)
{
    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);

    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.K[0]);
    convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.R[0]);
    convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.P[0]);
    convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);

    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.D.resize(D_rows*D_cols);
    for (int i = 0; i < D_rows*D_cols; ++i)
    {
        cam_info.D[i] = D_data[i].as<float>();
    }
}


bool AirsimROSWrapper::set_local_position_srv_cb(airsim_ros_pkgs::SetLocalPosition::Request& request, airsim_ros_pkgs::SetLocalPosition::Response& response)
{

}

bool AirsimROSWrapper::set_global_position_srv_cb(airsim_ros_pkgs::SetGlobalPosition::Request& request, airsim_ros_pkgs::SetGlobalPosition::Response& response) 
{
    
}