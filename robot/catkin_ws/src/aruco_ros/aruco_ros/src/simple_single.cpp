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
#include <std_msgs/UInt32MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>
#include <aruco_ros/aruco_id_pose.h>
#include <geometry_msgs/Pose.h>
#include "tf/transform_datatypes.h"
#include <aruco_msgs/MarkerArray.h>

class ArucoSimple
{
private:
  
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  tf::StampedTransform offset;
  ros::Publisher marker_list_pub_;
 ros::Publisher marker_pub_;
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
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;
  std::vector<geometry_msgs::Pose> objectpose;
    std::vector<geometry_msgs::Pose> objectfilterpose;
   double newoldx =0;
    double newoldy =0;
    double newoldz =0;
  std::vector<bool> foundmarker;
  double marker_size;
  int marker_id;
  int counter =0;
  int counter2 =0;
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  std_msgs::UInt32MultiArray marker_list_msg_;
  aruco_msgs::MarkerArray::Ptr marker_msg_;
  tf::TransformListener _tfListener;
  bool marker_list_changes =true;
  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

ros::ServiceServer service;
 int markerno =0;

public:
    
  ArucoSimple() :
      cam_info_received(false), nh("~"), it(nh)
  {
   objectpose.resize(999);
   objectfilterpose.resize(999);
   foundmarker.resize(999);
   this->service= nh.advertiseService("aruco_id_pose", &ArucoSimple::add,this);
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
    marker_list_pub_ = nh.advertise<std_msgs::UInt32MultiArray>("markers_list", 10);
    marker_pub_ = nh.advertise<aruco_msgs::MarkerArray>("markers", 100);


    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("marker_id", marker_id, 500);
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);
        marker_msg_ = aruco_msgs::MarkerArray::Ptr(new aruco_msgs::MarkerArray());
    marker_msg_->header.frame_id = reference_frame;
    marker_msg_->header.seq = 0;
    ROS_ASSERT(camera_frame != "" && marker_frame != "");
 
    if (reference_frame.empty())
      reference_frame = camera_frame;

    ROS_INFO("ArUco node started with marker size of %f m and marker id to track: %d", marker_size, marker_id);
    ROS_INFO("ArUco node will publish pose to TF with %s as parent and %s as child.", reference_frame.c_str(),
             marker_frame.c_str());

 dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));


  }

geometry_msgs::Pose offsetconvert(geometry_msgs::Pose inputpose, double x, double y,double z )
{
  tf::StampedTransform offset;
  offset.setIdentity();
  offset.setOrigin(tf::Vector3(x, y, z));
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(inputpose.position.x,inputpose.position.y,inputpose.position.z));
  transform.setRotation(tf::Quaternion(inputpose.orientation.x,inputpose.orientation.y,inputpose.orientation.z,inputpose.orientation.w));
  transform= transform * static_cast<tf::Transform>(offset);
  geometry_msgs::Pose poseMsg;
  tf::poseTFToMsg(transform, poseMsg);
  return poseMsg;
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
   foundmarker.clear();
   bool publishMarkersList = marker_list_pub_.getNumSubscribers() > 0;
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
                marker_msg_->markers.clear();
        marker_msg_->markers.resize(markers.size());
        marker_msg_->header.stamp = curr_stamp;
        marker_msg_->header.seq++;

        markerno = markers.size();
        for (std::size_t i = 0; i < markers.size(); ++i)
        {	
        marker_msg_->markers.clear();
        marker_msg_->markers.resize(markers.size());
        marker_msg_->header.stamp = curr_stamp;
        marker_msg_->header.seq++;

            tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
            tf::Transform transform2 = aruco_ros::arucoMarker2Tf(markers[i]);
            
            transform2 = static_cast<tf::Transform>(rightToLeft)* transform2;
            geometry_msgs::PoseStamped posefliterMsg;
            tf::poseTFToMsg(transform2, posefliterMsg.pose);
            posefliterMsg.header.frame_id = camera_frame;
            posefliterMsg.header.stamp = curr_stamp;
            objectfilterpose[markers[i].id] =posefliterMsg.pose;
            
            tf::StampedTransform cameraToReference;
            cameraToReference.setIdentity();
            if (reference_frame != camera_frame)
            {
              getTransform(reference_frame, camera_frame, cameraToReference);
            }
            
           transform = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft)
            * transform;

		//transform = static_cast<tf::Transform>(cameraToReference) * transform;
            

            tf::StampedTransform stampedTransform(transform, curr_stamp, reference_frame, marker_frame);
            br.sendTransform(stampedTransform);
            geometry_msgs::PoseStamped poseMsg;
            tf::poseTFToMsg(transform, poseMsg.pose);
            poseMsg.header.frame_id = reference_frame;
            poseMsg.header.stamp = curr_stamp;
            objectpose[markers[i].id] =poseMsg.pose;
            foundmarker[markers[i].id] =true;
            
         // only publishing the selected marker
          if (markers[i].id == marker_id)
          {
            pose_pub.publish(poseMsg);
          }
      }
      
      // publish marker array
      if (marker_msg_->markers.size() > 0) marker_pub_.publish(marker_msg_);

      if (publishMarkersList)
      {
        if(marker_list_msg_.data.size() !=markers.size())
        { 
		marker_list_changes =true;
        }
        marker_list_msg_.data.resize(markers.size());
       
        for (std::size_t i = 0; i < markers.size(); ++i)
        {
            if(marker_list_msg_.data[i] != markers[i].id)
             {
               marker_list_msg_.data[i] = markers[i].id;
               marker_list_changes =true;
             }
        }
        if(marker_list_changes)
        {
         marker_list_pub_.publish(marker_list_msg_);
         marker_list_changes=false;
        }

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

bool add(aruco_ros::aruco_id_pose::Request  &req, aruco_ros::aruco_id_pose::Response &res)
{
  marker_id = req.id;
  if(markerno !=0 && foundmarker[marker_id]){ 
  res.found =true;
  res.posesobject = objectpose[marker_id];
  res.posesfilter = objectfilterpose[marker_id];
  }
  else
  {
    res.found = false;
  }
  return true;
}

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple");
  ArucoSimple node;
  ros::spin();
}
