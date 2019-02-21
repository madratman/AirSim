#include <airsim_ros_wrapper.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <ros/console.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

constexpr char AirsimROSWrapper::CAM_YML_NAME[];
constexpr char AirsimROSWrapper::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapper::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapper::K_YML_NAME[];
constexpr char AirsimROSWrapper::D_YML_NAME[];
constexpr char AirsimROSWrapper::R_YML_NAME[];
constexpr char AirsimROSWrapper::P_YML_NAME[];
constexpr char AirsimROSWrapper::DMODEL_YML_NAME[];

AirsimROSWrapper::AirsimROSWrapper(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    initialize_airsim();
    initialize_ros();
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
/*  it_(nh_);
    left_image_pub_ = it.advertise("/airsim/zed/right/image_raw", 1)
    right_image_pub_ = it.advertise("/airsim/zed/left/image_raw", 1)
*/

    nh_private_.getParam("front_left_calib_file_", front_left_calib_file_);
    nh_private_.getParam("front_right_calib_file_", front_right_calib_file_);

    // fill camera info msg from YAML calib file. todo error check path
    read_params_from_yaml_and_fill_cam_info_msg(front_left_calib_file_, airsim_cam_info_front_left_);
    airsim_cam_info_front_left_.header.frame_id = "airsim/cam_front_left";

    read_params_from_yaml_and_fill_cam_info_msg(front_right_calib_file_, airsim_cam_info_front_right_);
    airsim_cam_info_front_right_.header.frame_id = "airsim/cam_front_right";

    takeoff_srvr_ = nh_private_.advertiseService("takeoff", &AirsimROSWrapper::takeoff_srv_callback, this);
    land_srvr_ = nh_private_.advertiseService("land", &AirsimROSWrapper::land_srv_callback, this);
    reset_srvr_ = nh_private_.advertiseService("reset",&AirsimROSWrapper::reset_srv_callback, this);

    odom_ned_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom_enu", 10);
    odom_enu_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom_ned", 10);
    cam_0_pose_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped> ("/cam_0/pose", 10);
    vel_cmd_body_frame_sub_ = nh_private_.subscribe("vel_cmd_body_frame", 50, &AirsimROSWrapper::vel_cmd_body_frame_cb, this); // todo ros::TransportHints().tcpNoDelay();
    vel_cmd_world_frame_sub_ = nh_private_.subscribe("vel_cmd_world_frame", 50, &AirsimROSWrapper::vel_cmd_world_frame_cb, this);

    double update_airsim_img_response_every_n_sec = 0.05;
    // nh_private_.param("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec, update_airsim_img_response_every_n_sec);
    airsim_img_response_timer_ = nh_private_.createTimer(ros::Duration(update_airsim_img_response_every_n_sec), &AirsimROSWrapper::img_response_timer_callback, this);

 

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

void AirsimROSWrapper::img_response_timer_callback(const ros::TimerEvent& event)
{    
    std::vector<ImageRequest> img_request = { 
        ImageRequest("front_left", ImageType::Scene,false, false), 
        ImageRequest("front_right", ImageType::Scene, false, false), 
        ImageRequest("front_left", ImageType::DepthPlanner, true)
    };

    try
    {
        std::cout << "call\n";
        const std::vector<ImageResponse>& img_response = airsim_client_.simGetImages(img_request);
  
        if (img_response.size() == img_request.size()) 
        {
            process_and_publish_img_response(img_response);
        }
    }

    catch (rpc::rpc_error& e)
    {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, didn't get image response." << std::endl << msg << std::endl;
    }

}

void AirsimROSWrapper::manual_decode_rgb(const ImageResponse &img_response, cv::Mat &mat)
{
    mat = cv::Mat(img_response.height, img_response.width, CV_8UC3, cv::Scalar(0, 0, 0));
    int camera_0_img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++)
    {
        for (int col = 0; col < camera_0_img_width; col++)
        {
            mat.at<cv::Vec3b>(row, col) = cv::Vec3b(
                img_response.image_data_uint8[row*camera_0_img_width*4 + 4*col + 2],
                img_response.image_data_uint8[row*camera_0_img_width*4 + 4*col + 1],
                img_response.image_data_uint8[row*camera_0_img_width*4 + 4*col + 0]);
        }
    }

    // cv::imshow("camera_0_img", mat);
    // cv::waitKey(1);
}

void AirsimROSWrapper::process_and_publish_img_response(const std::vector<ImageResponse>& img_response)
{
    // todo why is cv::imdecode not working
    // #if CV_MAJOR_VERSION==3
            // cv::Mat camera_0_img = cv::imdecode(img_response.at(0).image_data_uint8, cv::IMREAD_UNCHANGED);
            // auto rgb_right = cv::imdecode(response[1].image_data_uint8, cv::IMREAD_COLOR);
            // auto depth = cv::imdecode(response[2].image_data_uint8, cv::IMREAD_GRAYSCALE);
    // #else
    //     cv::Mat camera_0_img = cv::imdecode(img_response.at(0).image_data_uint8, CV_LOAD_IMAGE_COLOR);
    // #endif

    cv::Mat bgr_front_left = cv::Mat();
    manual_decode_rgb(img_response.at(0), bgr_front_left);

    cv::Mat bgr_front_right = cv::Mat();
    manual_decode_rgb(img_response.at(1), bgr_front_right);

    ros::Time curr_ros_time = ros::Time::now();

    std_msgs::Header tf_header;
    tf_header.stamp = curr_ros_time;
    tf_header.frame_id = "world";
    publish_camera_tf(img_response.at(0), tf_header, "airsim/cam_front_left");
    publish_camera_tf(img_response.at(1), tf_header, "airsim/cam_front_right");

    airsim_cam_info_front_left_.header.stamp = curr_ros_time; // update timestamp of cam info msg
    airsim_cam_info_front_right_.header.stamp = curr_ros_time; // update timestamp of cam info msg
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