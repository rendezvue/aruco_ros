#ifndef FOLLOW_MARKER
#define FOLLOW_MARKER
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>

enum follow_camera{NONE_CAM, LEFT_CAM, RIGHT_CAM,FRONT_CAM, BACK_CAM, LEFT_LIFT};

class FollowMarker{
private:

    ros::NodeHandle nh;
    
    ros::Publisher pub_omniwheel_velocity_QR_Marker;
    ros::Publisher pub_QR_localization_Complete;
    ros::Subscriber sub_QR_localization_Request;
    ros::ServiceServer srv_QR_localization;
    std::string m_camera_child_link_name;
    follow_camera m_cam_direction = NONE_CAM;
    std::string m_QR_localization_cmd;

    bool m_service_stop=false;
    void Ros_Sub_FollowInterface(const std_msgs::String::ConstPtr& msg);
    bool Ros_Srv_FollowInterface(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void Ros_Pub_State();
    bool Run_FollowMarker();
    bool Make_Filtered_Destination(const std::string camera_tf, const std::string destination_tf, tf::Vector3 &origin_sum, tf::Quaternion &quat_sum, tf::Transform &filter_tf,  int &filter_tf_sum_cnt);
    //bool Check_Goal_Reached(tf::Vector3 origin_sum, tf::Quaternion quad_sum);
    bool Make_Cmd_Vel(tf::Vector3 origin_sum, tf::Quaternion quad_sum, tf::Transform filter_tf, float &lin_x, float &lin_y ,float &lift_z, float &angular_z);
    void Move_Robot( float lin_x, float lin_y , float angular_z);
    void Move_Lift(float lift_z);

    ros::Subscriber sub_lift_height;
    void Ardu_Sub_LiftHeight(const std_msgs::Int32::ConstPtr& msg);
    int m_Lift_height;

    ros::Publisher lift_flag;
    ros::Publisher lift_cmd;
    std_msgs::Bool lift_bool;
    std_msgs::String lift_val;
    int lift_up_cnt = 0;
    int lift_down_cnt = 0;

    ros::Publisher pub_only_left_lift_qr_com;
    ros::Subscriber sub_only_left_lift_qr_req;
    void Sub_Only_left_lift(const std_msgs::String::ConstPtr& msg);
    std::string sub_o_left_lift;
    std_msgs::Bool pub_o_left_lift;

    double m_limit_dist = 0.15;
    double m_limit_front = 0.005;
    double m_limit_side = 0.005;
    double m_limit_ang = 1.0;
    double m_limit_lift = 0.01;

public:
    FollowMarker();
    ~FollowMarker();
    bool Update_Marker_TF(tf::TransformBroadcaster &br, tf::Transform, int marker_id, double limit_dist, double limit_x, double limit_y, double limit_ang, double limit_lift);
    bool Make_Destination_TF(tf::TransformBroadcaster &br, std::string marker_frame_id, double limit_dist);    
    void Thread_FollowMarker();
};

#endif