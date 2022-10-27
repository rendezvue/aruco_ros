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
#include <std_srvs/Trigger.h>

enum follow_camera{LEFT_CAM, RIGHT_CAM,FRONT_CAM, BACK_CAM};

class FollowMarker{
private:
    ros::Publisher pub_omniwheel_velocity_QR_Marker;
    ros::Publisher pub_QR_localization_Complete;
    ros::Subscriber sub_QR_localization_Request;
    ros::ServiceServer srv_QR_localization;
    std::string m_camera_child_link_name;
    follow_camera m_cam_direction = LEFT_CAM;
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
public:
    FollowMarker();
    ~FollowMarker();
    bool Update_Marker_TF(tf::TransformBroadcaster &br, tf::Transform, int marker_id);
    bool Make_Destination_TF(tf::TransformBroadcaster &br, std::string marker_frame_id);    
    void Thread_FollowMarker();
};

#endif