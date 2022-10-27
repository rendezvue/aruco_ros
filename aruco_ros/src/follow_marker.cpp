#include "aruco_ros/follow_marker.h"
#include <boost/thread.hpp>

#define DESTINATION_TF_NAME "destination_tf"
#define DESTINATION_TF_FILTERED_NAME "destination_tf_filter"

FollowMarker::FollowMarker()
{
    ros::NodeHandle nh;
    srv_QR_localization = nh.advertiseService("service_QR_localization", &FollowMarker::Ros_Srv_FollowInterface, this);
    pub_omniwheel_velocity_QR_Marker = nh.advertise<std_msgs::Float32MultiArray>("/omniwheel/velocity", 10);
    pub_QR_localization_Complete = nh.advertise<std_msgs::Bool>("QR_localization_complete",10);
}
FollowMarker::~FollowMarker()
{

}

bool FollowMarker::Ros_Srv_FollowInterface(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  boost::thread( boost::bind( &FollowMarker::Thread_FollowMarker, this ) );
  fprintf(stderr,"service_QR_localization service call !!!\n");
  return true;
}

void FollowMarker::Ros_Pub_State()
{
    
}


void FollowMarker::Thread_FollowMarker()
{
    while(1)
    {
        bool result =  Run_FollowMarker();
        std_msgs::Bool service_results;
        service_results.data = result;
        pub_QR_localization_Complete.publish(service_results);
    }
}

bool FollowMarker::Run_FollowMarker()
{
    tf::Vector3 origin_filter;
    tf::Quaternion quat_filter;
    int filter_tf_sum_cnt = 0;
    while(1)
    {   
        bool ret;     
        ret = Make_Filtered_Destination("left_camera_child",DESTINATION_TF_NAME, origin_filter , quat_filter, filter_tf_sum_cnt);
        if( ret == false ) return false;
        ret = Check_Goal_Reached(origin_filter , quat_filter);
        if( ret == true ) return true; // true : goal reached
        float lin_x, lin_y , lift_z, angular_z;
        Make_Cmd_Vel(origin_filter , quat_filter, lin_x, lin_y , lift_z, angular_z);
        Move_Robot(lin_x, lin_y ,angular_z);
        Move_Lift(lift_z);
    }
}

bool FollowMarker::Make_Destination_TF(tf::TransformBroadcaster &br, std::string marker_frame_id)
{
    tf::Transform destination_tf;
    destination_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Vector3 position_xyz(0.0, 0.0, 0.25); // z축 기준 20cm
    destination_tf.setOrigin(position_xyz);

    tf::Quaternion dest_quat;
    dest_quat.setRPY(1.541592,0,1.541592);
    destination_tf.setRotation(dest_quat);
    
    tf::StampedTransform stampedTransform_destination(destination_tf, ros::Time::now(), marker_frame_id.c_str(), DESTINATION_TF_NAME);
    br.sendTransform(stampedTransform_destination);

    //make_destination_cmd_vel("left_camera_child", "destination_tf");
    
    return true;
}

bool FollowMarker::Make_Filtered_Destination(const std::string camera_tf, const std::string destination_tf, tf::Vector3 &origin_sum, tf::Quaternion &quat_sum, int &filter_tf_sum_cnt)
{
    static tf::TransformBroadcaster filter_tf_br;
    tf::TransformListener tfListener;
    tf::StampedTransform stampedtf_destination;
    if(camera_tf != destination_tf)
    {
        try
        {
            ROS_INFO("INFO INFO HJPARK %s, %s",camera_tf.c_str(), destination_tf.c_str() );
            if( tfListener.waitForTransform(camera_tf, destination_tf, ros::Time(0), ros::Duration(1)) )
            {
                tfListener.lookupTransform(camera_tf, destination_tf, ros::Time(0), stampedtf_destination);
            }
            else
            {
                ROS_ERROR("HJPARK: waitForTransform failed");
                return false;
            }
        }
        catch ( std::exception &e)
        {
            ROS_ERROR("make_destination_cmd_vel, transform error!");
            return false;
        }
    }

    tf::Transform filter_tf;
    {
        
        tf::Vector3 origin = stampedtf_destination.getOrigin();
        tf::Quaternion quat = stampedtf_destination.getRotation();     
        if( filter_tf_sum_cnt == 0 )
        { 
            origin_sum = origin;
            quat_sum = quat;
        }
        else
        {
            float rate_A = 0.9;
            float rate_B = 0.1;
            origin_sum = (origin_sum * rate_A) + (origin * rate_B);
            quat_sum = (quat_sum * rate_A) + (quat * rate_B);
            // origin_sum = origin_sum + origin;
            // quat_sum = quat_sum + quat ;
        }
        filter_tf_sum_cnt++;

        filter_tf.setOrigin(origin_sum);
        filter_tf.setRotation(quat_sum);

        tf::StampedTransform filter_stamp(filter_tf, ros::Time::now(),camera_tf.c_str(), DESTINATION_TF_FILTERED_NAME );

        filter_tf_br.sendTransform(filter_stamp);
    }
    return true;
}


bool FollowMarker::Check_Goal_Reached(tf::Vector3 origin_sum, tf::Quaternion quad_sum)
{
     tf::Vector3 origin = origin_sum;
    tf::Quaternion quat = quad_sum;
    tf::Matrix3x3 m(quat);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    float calc_dRad = -yaw;

    if( abs(origin.x()) < 0.01 && 
        abs(origin.y()) < 0.01 &&
        abs((calc_dRad)*180/3.141592) < 1 )
    {
        std_msgs::Float32MultiArray omniwheel_velocity_QR_Marker;  
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        pub_omniwheel_velocity_QR_Marker.publish(omniwheel_velocity_QR_Marker);
        return true;
    }
    return false;
}


bool FollowMarker::Make_Cmd_Vel(tf::Vector3 origin_sum, tf::Quaternion quad_sum, float &lin_x, float &lin_y ,float &lift_z, float &angular_z)
{
    // tf::Vector3 origin = stampedtf_destination.getOrigin();
    // tf::Quaternion quat = stampedtf_destination.getRotation();
    tf::Vector3 origin = origin_sum;
    tf::Quaternion quat = quad_sum;


    tf::Matrix3x3 m(quat);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    fprintf(stderr, "r p y (%f / %f/ %f)\n",roll, pitch, yaw);

    float max_vel_x = 0.02;
    float max_vel_y = 0.02;
    max_vel_y = abs(origin.y() / 0.3 * 0.1);
    if( max_vel_y > 0.1 ) max_vel_y = 0.1;
    if( max_vel_y < 0.02) max_vel_y = 0.02;
    lin_x = origin.x() / (abs(origin.x()) + abs(origin.y())) * max_vel_x;
    lin_y = -(origin.y() / (abs(origin.x()) + abs(origin.y())) * max_vel_y);
    
    float calc_dRad = -yaw;

    float max_yaw = 0.02;
    if( calc_dRad > max_yaw )
    {
        calc_dRad = max_yaw;
    }
    else if( calc_dRad < -max_yaw)
    {
        calc_dRad = -max_yaw;
    }
    
    angular_z = calc_dRad;


    fprintf(stderr,"[curtime:%ld] TEST!! %f, %f, %f /  yaw ( rad: %f / deg : %f ) , lin_x = %f, lin_y = %f\n",
                ros::Time::now().toNSec(),
                //stampedtf_destination.stamp_.toNSec(),
                origin.x() ,origin.y(), origin.z(), calc_dRad , ((calc_dRad)*180/3.141592),
                lin_x, lin_y);


    return true;
}

void FollowMarker::Move_Robot( float lin_x, float lin_y , float angular_z)
{
                

    //ros::Duration calc_time = ros::Time::now() - stampedtf_destination.stamp_;

    long limit_time = 100000000;
    //if( calc_time.toNSec() <= 100000000 )
    {
        //fprintf(stderr, "duration time : %ld\n", calc_time.toNSec());

        // ros pub( cmd_vel )
        // /omni ( x) 
        // /시간계산전용토픽ㄱ주소

        double wheels_k = 0.521500; // = wheels_k_;
        double wheels_radius_ = 0.089;

        double w0_vel = 1.0 / wheels_radius_ * (lin_x - lin_y - wheels_k * angular_z);
        double w1_vel = 1.0 / wheels_radius_ * (lin_x + lin_y - wheels_k * angular_z);
        double w2_vel = 1.0 / wheels_radius_ * (lin_x - lin_y + wheels_k * angular_z);
        double w3_vel = 1.0 / wheels_radius_ * (lin_x + lin_y + wheels_k * angular_z);

        std_msgs::Float32MultiArray omniwheel_velocity_QR_Marker;  
        omniwheel_velocity_QR_Marker.data.push_back(w0_vel);
        omniwheel_velocity_QR_Marker.data.push_back(w1_vel);
        omniwheel_velocity_QR_Marker.data.push_back(w2_vel);
        omniwheel_velocity_QR_Marker.data.push_back(w3_vel);
        pub_omniwheel_velocity_QR_Marker.publish(omniwheel_velocity_QR_Marker);
        

    }
    /*else 
    {
        // lift_bool.data = false;
        // lift_val.data = "s";
        qr_corr.data = false;
        std_msgs::Float32MultiArray omniwheel_velocity_QR_Marker;  
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        pub_omniwheel_velocity_QR_Marker.publish(omniwheel_velocity_QR_Marker);
        origin_sum = tf::Vector3();
        quat_sum = tf::Quaternion();
        filter_tf_sum_cnt = 0;
        return false;
    }*/
}

void FollowMarker::Move_Lift(float lift_z)
{
    if ( lift_z < -0.01 )
    {
        // lift_bool.data = true;
        // lift_val.data = "w";
    }
    else if ( lift_z > 0.01 )
    {
        // lift_bool.data = true;
        // lift_val.data = "x";
    }
    else if ( lift_z > -0.01 && lift_z < 0.01)
    {
        // lift_bool.data = true;
        // lift_val.data = "s";
    }
}