#include "aruco_ros/follow_marker.h"
#include <boost/thread.hpp>

#define DESTINATION_TF_NAME "destination_tf"
#define DESTINATION_TF_FILTERED_NAME "destination_tf_filter"

FollowMarker::FollowMarker()
{
    
    srv_QR_localization = nh.advertiseService("service_QR_localization", &FollowMarker::Ros_Srv_FollowInterface, this);
    sub_QR_localization_Request = nh.subscribe("QR_localization_Request", 1, &FollowMarker::Ros_Sub_FollowInterface, this);
    pub_omniwheel_velocity_QR_Marker = nh.advertise<std_msgs::Float32MultiArray>("/omniwheel/velocity", 10);
    pub_QR_localization_Complete = nh.advertise<std_msgs::Bool>("QR_localization_complete",10);
    sub_lift_height = nh.subscribe("/arduino/lift/ToF_dist_mm", 1, &FollowMarker::Ardu_Sub_LiftHeight, this);
    
    lift_flag = nh.advertise<std_msgs::Bool>("/arduino/lift/flag", 1);
    lift_cmd = nh.advertise<std_msgs::String>("/arduino/lift/cmd", 1);

    sub_only_left_lift_qr_req = nh.subscribe("Lift_Left_QR_Req", 1, &FollowMarker::Sub_Only_left_lift, this);
    pub_only_left_lift_qr_com = nh.advertise<std_msgs::Bool>("Left_Lift_complete", 10);
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

void FollowMarker::Ardu_Sub_LiftHeight(const std_msgs::Int32::ConstPtr& msg)
{
    m_Lift_height = msg->data;
    // fprintf(stderr,"[Ardu_Sub_LiftHeight] received msg : [ %d ] mm\n", m_Lift_height);

}

void FollowMarker::Sub_Only_left_lift(const std_msgs::String::ConstPtr& msg)
{
    fprintf(stderr,"[Sub_Only_left_lift] received msg : [ %s ]\n", msg->data.c_str());
    sub_o_left_lift = msg->data;
}

void FollowMarker::Ros_Sub_FollowInterface(const std_msgs::String::ConstPtr& msg)
{
    fprintf(stderr,"[Ros_Sub_FollowInterface] received msg : [ %s ]\n", msg->data.c_str());
    m_QR_localization_cmd = msg->data;

    if( m_QR_localization_cmd == "STOP" )
    {
        m_service_stop = true;
        std_msgs::Float32MultiArray omniwheel_velocity_QR_Marker;  
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        pub_omniwheel_velocity_QR_Marker.publish(omniwheel_velocity_QR_Marker);
        Move_Lift(0);
    }
    else
    {
        m_service_stop = true;
        usleep(1000*1000);
        if( m_QR_localization_cmd == "LEFT_CAM" )
        {
            m_cam_direction = LEFT_CAM;        
        }
        else if( m_QR_localization_cmd == "RIGHT_CAM" )
        {
            m_cam_direction = RIGHT_CAM;
        }
        else if( m_QR_localization_cmd == "FRONT_CAM" )
        {
            m_cam_direction = FRONT_CAM;        
        }
        else if( m_QR_localization_cmd == "BACK_CAM" )
        {
            m_cam_direction = BACK_CAM;        
        }
        else if(m_QR_localization_cmd == "LEFT_LIFT")
        {
            m_cam_direction = LEFT_LIFT;
        }

        m_service_stop = false;
        boost::thread( boost::bind( &FollowMarker::Thread_FollowMarker, this ) );
        fprintf(stderr,"service_QR_localization service call !!!\n");
    }
}

void FollowMarker::Ros_Pub_State()
{
    
}

void FollowMarker::Thread_FollowMarker()
{
    bool result = Run_FollowMarker();
    std_msgs::Bool service_results;
    service_results.data = result;
    pub_QR_localization_Complete.publish(service_results);

    if (m_cam_direction==LEFT_LIFT)
    {
        pub_o_left_lift.data = result;
        pub_only_left_lift_qr_com.publish(pub_o_left_lift);
    }
}

bool FollowMarker::Run_FollowMarker()
{
    tf::Vector3 origin_filter;
    tf::Quaternion quat_filter;
    int filter_tf_sum_cnt = 0;
    while(1)
    {   
        if( m_service_stop == true)
        {
            break;
        }
        usleep(1000*30);//30ms
        bool ret;     
        tf::Transform filter_tf;
        ret = Make_Filtered_Destination(m_camera_child_link_name, DESTINATION_TF_NAME+m_camera_child_link_name , origin_filter , quat_filter, filter_tf, filter_tf_sum_cnt);
        if( ret == false ) 
        {
            std_msgs::Float32MultiArray omniwheel_velocity_QR_Marker;  
            omniwheel_velocity_QR_Marker.data.push_back(0);
            omniwheel_velocity_QR_Marker.data.push_back(0);
            omniwheel_velocity_QR_Marker.data.push_back(0);
            omniwheel_velocity_QR_Marker.data.push_back(0);
            pub_omniwheel_velocity_QR_Marker.publish(omniwheel_velocity_QR_Marker);
            Move_Lift(0);
            return false;
        }
        //ret = Check_Goal_Reached(origin_filter , quat_filter);
        //if( ret == true ) return true; // true : goal reached
        float lin_x, lin_y , lift_z, angular_z;

        ret = Make_Cmd_Vel(origin_filter , quat_filter, filter_tf , lin_x, lin_y , lift_z, angular_z);
        if( ret == true ) 
        {
            return true; // true : goal reached
        }
        Move_Robot(lin_x, lin_y ,angular_z);
        Move_Lift(lift_z);
    }
}

bool FollowMarker::Update_Marker_TF(tf::TransformBroadcaster &br, tf::Transform transform, int marker_id, double limit_dist, double limit_front, double limit_side, double limit_ang, double limit_lift)
{
    std::string marker_frame_with_id = "marker_frame_";// + "_" + std::to_string(marker_id).c_str();
    
    tf::Transform transform_cam_child;

    m_limit_dist = limit_dist;
    m_limit_front = limit_front;
    m_limit_side = limit_side;
    m_limit_ang = limit_ang;
    m_limit_lift = limit_lift;
    
    transform_cam_child.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(-1.57079632679489655, 0, -1.57079632679489655);
    transform_cam_child.setRotation(q);
    if( m_cam_direction == LEFT_CAM || m_cam_direction == LEFT_LIFT)
    {
        m_camera_child_link_name = "left_camera_child";
        br.sendTransform (tf::StampedTransform(transform_cam_child, ros::Time::now(), "left_camera_link", m_camera_child_link_name));
    }
    else if( m_cam_direction == RIGHT_CAM )
    {
        m_camera_child_link_name = "right_camera_child";
        br.sendTransform (tf::StampedTransform(transform_cam_child, ros::Time::now(), "right_camera_link", m_camera_child_link_name));
    }
    else if( m_cam_direction == FRONT_CAM )
    {
        m_camera_child_link_name = "front_camera_child";
        br.sendTransform (tf::StampedTransform(transform_cam_child, ros::Time::now(), "front_camera_link", m_camera_child_link_name));
    }
    else if( m_cam_direction == BACK_CAM )
    {
        m_camera_child_link_name = "back_camera_child";
        br.sendTransform (tf::StampedTransform(transform_cam_child, ros::Time::now(), "back_camera_link", m_camera_child_link_name));
    }
    if( m_cam_direction != NONE_CAM )
    {
        std::string marker_frame_name = marker_frame_with_id+m_camera_child_link_name;
        tf::StampedTransform stampedTransform(transform, ros::Time::now(), m_camera_child_link_name, marker_frame_name);
        br.sendTransform (stampedTransform);
        Make_Destination_TF(br,marker_frame_name,m_limit_dist);
    }
}

bool FollowMarker::Make_Destination_TF(tf::TransformBroadcaster &br, std::string marker_frame_id, double limit_dist)
{
    tf::Transform destination_tf;
    destination_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Vector3 position_xyz(0.0, 0.0, limit_dist); // z축 기준 20cm , default = 0.15
    destination_tf.setOrigin(position_xyz);

    tf::Quaternion dest_quat;
    dest_quat.setRPY(0,0,0);
    destination_tf.setRotation(dest_quat);
    
    tf::StampedTransform stampedTransform_destination(destination_tf, ros::Time::now(), marker_frame_id.c_str(), DESTINATION_TF_NAME+m_camera_child_link_name);
    br.sendTransform(stampedTransform_destination);

    //make_destination_cmd_vel("left_camera_child", "destination_tf");
    
    return true;
}

bool FollowMarker::Make_Filtered_Destination(const std::string camera_tf, const std::string destination_tf, tf::Vector3 &origin_sum, tf::Quaternion &quat_sum, tf::Transform &filter_tf, int &filter_tf_sum_cnt)
{
    static tf::TransformBroadcaster filter_tf_br;
    tf::TransformListener tfListener;
    tf::StampedTransform stampedtf_destination;
    if(camera_tf != destination_tf)
    {
        try
        {
            ROS_INFO("INFO INFO HJPARK %s, %s",camera_tf.c_str(), destination_tf.c_str() );
            if( tfListener.waitForTransform(camera_tf, destination_tf, ros::Time(0), ros::Duration(0.5)) )
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

    //tf::Transform filter_tf;
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

        tf::StampedTransform filter_stamp(filter_tf, ros::Time::now(),camera_tf.c_str(), DESTINATION_TF_FILTERED_NAME+m_camera_child_link_name );

        filter_tf_br.sendTransform(filter_stamp);
    }
    return true;
}

// 
// bool FollowMarker::Check_Goal_Reached(tf::Vector3 origin_sum, tf::Quaternion quad_sum)
// {
    //  tf::Vector3 origin = origin_sum;
    // tf::Quaternion quat = quad_sum;
    // tf::Matrix3x3 m(quat);
    // double roll,pitch,yaw;
    // m.getRPY(roll,pitch,yaw);
    // float calc_dRad = -pitch;
// 
    // if( abs(front_direction) < 0.01 && 
        // abs(side_direction) < 0.01 &&
        // abs((calc_dRad)*180/3.141592) < 1 )
    // {
        // std_msgs::Float32MultiArray omniwheel_velocity_QR_Marker;  
        // omniwheel_velocity_QR_Marker.data.push_back(0);
        // omniwheel_velocity_QR_Marker.data.push_back(0);
        // omniwheel_velocity_QR_Marker.data.push_back(0);
        // omniwheel_velocity_QR_Marker.data.push_back(0);
        // pub_omniwheel_velocity_QR_Marker.publish(omniwheel_velocity_QR_Marker);
        // return true;
    // }
    // return false;
// }


bool FollowMarker::Make_Cmd_Vel(tf::Vector3 origin_sum, tf::Quaternion quad_sum, tf::Transform filter_tf , float &lin_x, float &lin_y ,float &lift_z, float &angular_z)
{
    // tf::Vector3 origin = stampedtf_destination.getOrigin();
    // tf::Quaternion quat = stampedtf_destination.getRotation();
    tf::Vector3 origin = origin_sum;
    tf::Quaternion quat = quad_sum;
    // tf::Vector3 origin = filter_tf.getOrigin();
    // tf::Quaternion quat = filter_tf.getRotation();

    tf::Matrix3x3 m(quat);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    fprintf(stderr, "r p y (%f / %f/ %f)\n",roll, pitch, yaw);

    float max_vel_x = 0.02;
    float max_vel_y = 0.02;
    
    float lift_distance = origin.y();
    float front_direction = origin.x();
    float side_direction = origin.z();
    float calc_dRad = -pitch;
    if( m_cam_direction == RIGHT_CAM )
    {
        front_direction = -origin.x();
        side_direction = -origin.z();
    }
    else if( m_cam_direction == FRONT_CAM )
    {
        front_direction = origin.z();
        side_direction = -origin.x();
    }
    else if( m_cam_direction == BACK_CAM )
    {
        front_direction = -origin.z();
        side_direction = origin.x();
    }
    
    
    if( m_cam_direction == LEFT_CAM || m_cam_direction == RIGHT_CAM )
    {
        max_vel_x = abs(front_direction / 0.3 * 0.1);
        if( max_vel_x > 0.1 ) max_vel_x = 0.1;
        if( max_vel_x < 0.02) max_vel_x = 0.02;

        max_vel_y = abs(side_direction / 0.3 * 0.1);
        if( max_vel_y > 0.1 ) max_vel_y = 0.1;
        if( max_vel_y < 0.02) max_vel_y = 0.02;
        lin_x = front_direction / (abs(front_direction) + abs(side_direction)) * max_vel_x;
        lin_y = (side_direction / (abs(front_direction) + abs(side_direction)) * max_vel_y);
    }
    else if ( m_cam_direction == FRONT_CAM || m_cam_direction == BACK_CAM )
    {
        max_vel_x = abs(front_direction / 0.3 * 0.1);
        if( max_vel_x > 0.1 ) max_vel_x = 0.1;
        if( max_vel_x < 0.02) max_vel_x = 0.02;

        max_vel_y = abs(side_direction / 0.3 * 0.3);
        if( max_vel_y > 0.1 ) max_vel_y = 0.1;
        if( max_vel_y < 0.02) max_vel_y = 0.02;
        lin_x = front_direction / (abs(front_direction) + abs(side_direction)) * max_vel_x;
        lin_y = (side_direction / (abs(front_direction) + abs(side_direction)) * max_vel_y);
    }

    float max_yaw = 0.05;
    if( calc_dRad > max_yaw )
    {
        calc_dRad = max_yaw;
    }
    else if( calc_dRad < -max_yaw)
    {
        calc_dRad = -max_yaw;
    }
    
    angular_z = calc_dRad;
    if (m_cam_direction == BACK_CAM || m_cam_direction == FRONT_CAM) lift_z = 0;
    else lift_z = lift_distance;
    
    fprintf(stderr,"[curtime:%ld] TEST!! %f, %f, %f /  yaw rad(%f) / calcYaw( rad: %f / deg : %f ) , lin_x = %f, lin_y = %f\n",
                ros::Time::now().toNSec(),
                //stampedtf_destination.stamp_.toNSec(),
                front_direction, side_direction, lift_distance, -yaw, calc_dRad , ((calc_dRad)*180/3.141592),
                lin_x, lin_y);


    if( m_cam_direction == LEFT_LIFT)
    {
        if(abs(lift_distance) < 0.01) 
        {
            std_msgs::Float32MultiArray omniwheel_velocity_QR_Marker;  
            omniwheel_velocity_QR_Marker.data.push_back(0);
            omniwheel_velocity_QR_Marker.data.push_back(0);
            omniwheel_velocity_QR_Marker.data.push_back(0);
            omniwheel_velocity_QR_Marker.data.push_back(0);
            pub_omniwheel_velocity_QR_Marker.publish(omniwheel_velocity_QR_Marker);
            Move_Lift(0);
            return true;
        }
        return false;
    }

    fprintf(stderr, "[ limit_dist : %f, limit_front : %f, limit_side : %f, limit_ang : %f, limit_lift : %f ]\n", m_limit_dist, m_limit_front, m_limit_side, m_limit_ang, m_limit_lift);

    static int success_try = 0;
    if( abs(front_direction) < m_limit_front && 
        abs(side_direction) < m_limit_side &&
        abs((calc_dRad)*180/3.141592) < m_limit_ang &&
        abs(lift_distance) < m_limit_lift )
    {
        std_msgs::Float32MultiArray omniwheel_velocity_QR_Marker;  
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        omniwheel_velocity_QR_Marker.data.push_back(0);
        pub_omniwheel_velocity_QR_Marker.publish(omniwheel_velocity_QR_Marker);
        Move_Lift(0);
        success_try++;
        
        fprintf(stderr,"aruco success count = %d\n",success_try);
        if( success_try > 30 )
        {
            return true;
        }
    }
    else
    {
        fprintf(stderr,"success_try reset(%d -> 0)\n",success_try);
        success_try = 0;
    }
    return false;
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

        if (m_cam_direction == LEFT_LIFT)
        {
            w0_vel = 0.00;
            w1_vel = 0.00;
            w2_vel = 0.00;
            w3_vel = 0.00;
        }

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

// lift_up_cnt
// lift_down_cnt
    if ( lift_z < -0.01 )
    {
        lift_bool.data = true;
        lift_val.data = "w";
    }

    else if ( lift_z > 0.01 )
    {
        lift_bool.data = true;
        lift_val.data = "x";
    }

    else if ( lift_z > -0.01 && lift_z < 0.01)
    {
        lift_bool.data = true;
        lift_val.data = "s";
    }
    lift_flag.publish(lift_bool);
    lift_cmd.publish(lift_val);
}