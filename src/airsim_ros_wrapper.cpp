#include <airsim_ros_wrapper.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <ros/console.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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

    geometry_msgs::TransformStamped front_left_tf_stamped;

    front_left_tf_stamped.header.stamp = ros::Time::now();
    front_left_tf_stamped.header.frame_id = "world";
    front_left_tf_stamped.child_frame_id = "cam_front_left";
    front_left_tf_stamped.transform.translation.x = img_response.at(0).camera_position.x();
    front_left_tf_stamped.transform.translation.y = img_response.at(0).camera_position.y();
    front_left_tf_stamped.transform.translation.z = img_response.at(0).camera_position.z();
    front_left_tf_stamped.transform.rotation.x = img_response.at(0).camera_orientation.x();
    front_left_tf_stamped.transform.rotation.y = img_response.at(0).camera_orientation.y();
    front_left_tf_stamped.transform.rotation.z = img_response.at(0).camera_orientation.z();
    front_left_tf_stamped.transform.rotation.w = img_response.at(0).camera_orientation.w();
    tf_broadcaster_.sendTransform(front_left_tf_stamped);

    front_right_tf_stamped.header.stamp = ros::Time::now();
    front_right_tf_stamped.header.frame_id = "world";
    front_right_tf_stamped.child_frame_id = "cam_front_right";
    front_right_tf_stamped.transform.translation.x = img_response.at(1).camera_position.x();
    front_right_tf_stamped.transform.translation.y = img_response.at(1).camera_position.y();
    front_right_tf_stamped.transform.translation.z = img_response.at(1).camera_position.z();
    front_right_tf_stamped.transform.rotation.x = img_response.at(1).camera_orientation.x();
    front_right_tf_stamped.transform.rotation.y = img_response.at(1).camera_orientation.y();
    front_right_tf_stamped.transform.rotation.z = img_response.at(1).camera_orientation.z();
    front_right_tf_stamped.transform.rotation.w = img_response.at(1).camera_orientation.w();
    tf_broadcaster_.sendTransform(front_right_tf_stamped);
}
