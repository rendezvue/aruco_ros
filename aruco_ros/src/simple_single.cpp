/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file simple_single.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

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

#include <boost/thread.hpp>

#include <std_msgs/Float32MultiArray.h>
class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  std::vector<aruco::Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub;
  ros::Publisher position_pub;
  ros::Publisher marker_pub; // rviz visualization marker
  ros::Publisher pixel_pub;

  //rdv mecanum wheel direct controller
  ros::Publisher pub_omniwheel_velocity_QR_Marker;

  // ros::Publisher marker_rpy_pub;

  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;

  double marker_size;
  int marker_id;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;



public:
  void thread_destination_cmd_vel();
  ArucoSimple() :
      cam_info_received(false), nh("~"), it(nh)
  {

    if (nh.hasParam("corner_refinement"))
      ROS_WARN(
          "Corner refinement options have been removed in ArUco 3.0.0, corner_refinement ROS parameter is deprecated");

    aruco::MarkerDetector::Params params = mDetector.getParameters();
    std::string thresh_method;
    switch (params._thresMethod)
    {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }

    // Print parameters of ArUco marker detector:
    ROS_INFO_STREAM("Threshold method: " << thresh_method);

    float min_marker_size; // percentage of image area
    nh.param<float>("min_marker_size", min_marker_size, 0.02);

    std::string detection_mode;
    nh.param<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST")
      mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
    else if (detection_mode == "DM_VIDEO_FAST")
      mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    else
      // Aruco version 2 mode
      mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);

    ROS_INFO_STREAM("Marker size min: " << min_marker_size << "% of image area");
    ROS_INFO_STREAM("Detection mode: " << detection_mode);

    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
    pixel_pub = nh.advertise<geometry_msgs::PointStamped>("pixel", 10);
    pub_omniwheel_velocity_QR_Marker = nh.advertise<std_msgs::Float32MultiArray>("destination_velocity", 10);
    // marker_rpy_pub = nh.advertise<geometry_msgs::Vector3>("rpy", 100);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("marker_id", marker_id, 300);
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    ROS_ASSERT(camera_frame != "" && marker_frame != "");

    if (reference_frame.empty())
      reference_frame = camera_frame;

    ROS_INFO("ArUco node started with marker size of %f m and marker id to track: %d", marker_size, marker_id);
    ROS_INFO("ArUco node will publish pose to TF with %s as parent and %s as child.", reference_frame.c_str(),
             marker_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
  }

  bool make_destination_cmd_vel(std::string camera_tf, std::string destination_tf)
  {
    tf::TransformListener tfListener;
    tf::StampedTransform camera_to_destination_tf;

    if(camera_tf != destination_tf)
    {
      try
      {
        ROS_INFO("INFO INFO HJPARK %s, %s",camera_tf.c_str(), destination_tf.c_str() );
        if( tfListener.waitForTransform(camera_tf, destination_tf, ros::Time(0), ros::Duration(1)) )
        {
          tfListener.lookupTransform(camera_tf, destination_tf, ros::Time(0), camera_to_destination_tf);
        }
        else
        {
          ROS_ERROR("HJPARK: waitForTransform failed");
        }
      }
      catch ( std::exception &e)
      {
        ROS_ERROR("make_destination_cmd_vel, transform error!");
      }
    }

    tf::Vector3 origin = camera_to_destination_tf.getOrigin();

    
    float dRad = atan2(origin.x(), origin.y());


    float max_vel = 0.3;
    float lin_x = origin.x() / (abs(origin.x()) + abs(origin.y())) * max_vel;
    float lin_y = -(origin.y() / (abs(origin.x()) + abs(origin.y())) * max_vel);
    
    float max_yaw = 0.1;

    float calc_dRad = 3.141592 - dRad;
    if( abs(calc_dRad) > max_yaw )
    {
      calc_dRad = max_yaw;
    }
    else if( abs(calc_dRad) < max_yaw)
    {
      calc_dRad = -max_yaw;
    }

    

    fprintf(stderr,"[curtime:%ld][timestamp:%ld] TEST!! %f, %f, %f /  yaw ( rad: %f / deg : %f ) , lin_x = %f, lin_y = %f\n",
                  ros::Time::now().toNSec(),
                  camera_to_destination_tf.stamp_.toNSec(),
                  origin.x() ,origin.y(), origin.z(), calc_dRad , ((calc_dRad)*180/3.141592),
                  lin_x, lin_y);

    ros::Duration calc_time = ros::Time::now() - camera_to_destination_tf.stamp_;

    long limit_time = 100000000;
    if( calc_time.toNSec() <= 100000000 )
    {
      fprintf(stderr, "duration time : %ld\n", calc_time.toNSec());

      // ros pub( cmd_vel )
      // /omni ( x) 
      // /시간계산전용토픽ㄱ주소

      double wheels_k = 0.521500; // = wheels_k_;
      double wheels_radius_ = 0.089;

      double w0_vel = 1.0 / wheels_radius_ * (lin_x - lin_y - wheels_k * calc_dRad);
      double w1_vel = 1.0 / wheels_radius_ * (lin_x + lin_y - wheels_k * calc_dRad);
      double w2_vel = 1.0 / wheels_radius_ * (lin_x - lin_y + wheels_k * calc_dRad);
      double w3_vel = 1.0 / wheels_radius_ * (lin_x + lin_y + wheels_k * calc_dRad);

      std_msgs::Float32MultiArray omniwheel_velocity_QR_Marker;  
      omniwheel_velocity_QR_Marker.data.push_back(w0_vel);
      omniwheel_velocity_QR_Marker.data.push_back(w1_vel);
      omniwheel_velocity_QR_Marker.data.push_back(w2_vel);
      omniwheel_velocity_QR_Marker.data.push_back(w3_vel);
      pub_omniwheel_velocity_QR_Marker.publish(omniwheel_velocity_QR_Marker);

    }

  }

  bool make_destination_tf(tf::TransformBroadcaster &br, std::string marker_frame_id)
  {
      tf::Transform destination_tf;
      destination_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
      tf::Vector3 position_xyz(0.0, 0.0, 0.2); // z축 기준 20cm
      destination_tf.setOrigin(position_xyz);

      tf::Quaternion dest_quat;
      // 3.141592653589793
      // 1.57079632679489655
      dest_quat.setRPY(0,0,0);
      destination_tf.setRotation(dest_quat);
      
      tf::StampedTransform stampedTransform_destination(destination_tf, ros::Time::now(), marker_frame_id.c_str(), "destination_tf");
      br.sendTransform(stampedTransform_destination);

      //make_destination_cmd_vel("left_camera_child", "destination_tf");
      
      return true;
  }

  bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
  {
    std::string errMsg;

    if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01),
                                      &errMsg))
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), // get latest available
                                    transform);
      }
      catch (const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if ((image_pub.getNumSubscribers() == 0) && (debug_pub.getNumSubscribers() == 0)
        && (pose_pub.getNumSubscribers() == 0) && (transform_pub.getNumSubscribers() == 0)
        && (position_pub.getNumSubscribers() == 0) && (marker_pub.getNumSubscribers() == 0)
        && (pixel_pub.getNumSubscribers() == 0))
    {
      ROS_DEBUG("No subscribers, not looking for ArUco markers");
      return;
    }

    static tf::TransformBroadcaster br;
    if (cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        // detection results will go into "markers"
        markers.clear();
        // ok, let's detect
        mDetector.detect(inImage, markers, camParam, marker_size, false);
        // for each marker, draw info and its boundaries in the image
        for (std::size_t i = 0; i < markers.size(); ++i)
        {
          // only publishing the selected marker
          // if (markers[i].id == marker_id)
          // {
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();

          if (reference_frame != camera_frame)
          {
            fprintf(stderr,"reference_frame != camera_frame\n");
            getTransform(reference_frame, camera_frame, cameraToReference);
          }

          transform = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft) * transform;

          // camera child TF
          static tf::TransformBroadcaster br_left_cam_child;
          tf::Transform transform_left_cam_child;
          transform_left_cam_child.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
          tf::Quaternion q;
          // 3.141592653589793
          // 1.57079632679489655
          q.setRPY(-1.57079632679489655, 0, -1.57079632679489655);
          transform_left_cam_child.setRotation(q);
          br_left_cam_child.sendTransform (tf::StampedTransform(transform_left_cam_child, ros::Time::now(), "left_camera_link", "left_camera_child"));

          q.setRPY(3.141592, 0, -1.57079632679489655);
          transform_left_cam_child.setRotation(q);
          br_left_cam_child.sendTransform (tf::StampedTransform(transform_left_cam_child, ros::Time::now(), "left_camera_link", "left_camera_baselink_direction"));



          // tf2_ros::TransformBroadcaster tfb;
          // geometry_msgs::TransformStamped test_pose;
          // test_pose.header.stamp = ros::Time::now();
          // test_pose.header.frame_id = "left_camera_link";
          // test_pose.child_frame_id ="test";
          // // test_pose.transform.rotation.w = 1.0;
          // tfb.sendTransform(test_pose);
          // transform.setRotation(q);


          // tf::StampedTransform stampedTransform(transform, curr_stamp, reference_frame, marker_frame);
          std::string marker_frame_with_id = marker_frame + "_" + std::to_string(markers[i].id);
          tf::StampedTransform stampedTransform(transform, ros::Time::now(), "left_camera_child", marker_frame_with_id.c_str());


          // // left_camera_child에서 20cm 떨어진 tf 생성 테스트
          // static tf::TransformBroadcaster br_test;
          // tf::Transform transform_test;
          // transform_test.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
          // tf::Vector3 t(0.0, 0.0, 0.2); // z축 기준 20cm
          // transform_test.setOrigin(t);
          // br_test.sendTransform (tf::StampedTransform(transform_test, ros::Time::now(), "left_camera_child", "test_tf"));


          // marker TF 생성
          br.sendTransform(stampedTransform);
          make_destination_tf(br, marker_frame_with_id);

          // just publish topic
          geometry_msgs::PoseStamped poseMsg;
          tf::poseTFToMsg(transform, poseMsg.pose);
          poseMsg.header.frame_id = "left_camera_child";
          // poseMsg.header.frame_id = reference_frame;
          poseMsg.header.stamp = curr_stamp;
          pose_pub.publish(poseMsg);
#if 0
          fprintf(stderr,"poseMsg : \n(x,y,z)=(%f,%f,%f) \n(x,y,z,w)=(%f,%f,%f,%f)\n", 
                  poseMsg.pose.position.x, poseMsg.pose.position.y, poseMsg.pose.position.z,
                  poseMsg.pose.orientation.x, poseMsg.pose.orientation.y, poseMsg.pose.orientation.z, poseMsg.pose.orientation.w);

          // quaternion to rpy
          double roll, pitch, yaw;
          tf::Quaternion q_marker(poseMsg.pose.orientation.x,
                                  poseMsg.pose.orientation.y,
                                  poseMsg.pose.orientation.z,
                                  poseMsg.pose.orientation.w);
          tf::Matrix3x3 m_marker(q_marker);
          m_marker.getRPY(roll, pitch, yaw);
          geometry_msgs::Vector3 rpy;
          rpy.x = roll;
          rpy.y = pitch;
          rpy.z = yaw;
          fprintf(stderr,"RPY : \n(r,p,y)=(%f,%f,%f)\n", roll, pitch, yaw);
          // marker_rpy_pub.publish(rpy);
#endif

          geometry_msgs::TransformStamped transformMsg;
          tf::transformStampedTFToMsg(stampedTransform, transformMsg);
          transform_pub.publish(transformMsg);

          geometry_msgs::Vector3Stamped positionMsg;
          positionMsg.header = transformMsg.header;
          positionMsg.vector = transformMsg.transform.translation;
          position_pub.publish(positionMsg);

          geometry_msgs::PointStamped pixelMsg;
          pixelMsg.header = transformMsg.header;
          pixelMsg.point.x = markers[i].getCenter().x;
          pixelMsg.point.y = markers[i].getCenter().y;
          pixelMsg.point.z = 0;
          pixel_pub.publish(pixelMsg);

          // publish rviz marker representing the ArUco marker patch
          visualization_msgs::Marker visMarker;
          visMarker.header = transformMsg.header;
          visMarker.id = markers[i].id;
          visMarker.type = visualization_msgs::Marker::CUBE;
          visMarker.action = visualization_msgs::Marker::ADD;
          visMarker.pose = poseMsg.pose;
          visMarker.scale.x = marker_size;
          visMarker.scale.y = marker_size;
          visMarker.scale.z = 0.001;
          visMarker.color.r = 1.0;
          visMarker.color.g = 1.0;
          visMarker.color.b = 1.0;
          visMarker.color.a = 1.0;
          visMarker.lifetime = ros::Duration(3.0);
          marker_pub.publish(visMarker);

          // }
          // but drawing all the detected markers
          markers[i].draw(inImage, cv::Scalar(0, 0, 255), 5);
        }

        // draw a 3d cube in each marker if there is 3d info
        if (camParam.isValid() && marker_size != -1)
        {
          for (std::size_t i = 0; i < markers.size(); ++i)
          {
            aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

        if (image_pub.getNumSubscribers() > 0)
        {
          // show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if (debug_pub.getNumSubscribers() > 0)
        {
          // show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CameraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(tf::Vector3(-msg.P[3] / msg.P[0], -msg.P[7] / msg.P[5], 0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }
};

void ArucoSimple::thread_destination_cmd_vel()
{
  while(1)
  {
    usleep(1000*30);
    make_destination_cmd_vel("left_camera_baselink_direction", "destination_tf");
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;
  boost::thread( boost::bind( &ArucoSimple::thread_destination_cmd_vel, &node ) );
  ros::spin();
}
