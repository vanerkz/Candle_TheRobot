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
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <aruco_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt32MultiArray.h>
#include <aruco_ros/aruco_id_pose.h>
#include <geometry_msgs/Pose.h>
#include "tf/transform_datatypes.h"

class ArucoMarkerPublisher
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  aruco::CameraParameters camParam_;
  std::vector<aruco::Marker> markers_;

  // node params
  bool useRectifiedImages_;
  std::string marker_frame_;
  std::string camera_frame_;
  std::string reference_frame_;
  double marker_size_;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_list_pub_;
  ros::Publisher pose_pub;
  tf::TransformListener tfListener_;
    tf::StampedTransform rightToLeft;
  tf::StampedTransform offset;
    std::vector<geometry_msgs::Pose> objectpose;
    std::vector<geometry_msgs::Pose> objectfilterpose;
   double newoldx =0;
    double newoldy =0;
    double newoldz =0;
  std::vector<int> foundmarker;
  int marker_id=0;
  ros::Subscriber cam_info_sub_;
  aruco_msgs::MarkerArray::Ptr marker_msg_;
  cv::Mat inImage_;
  bool useCamInfo_;
  std_msgs::UInt32MultiArray marker_list_msg_;
std_msgs::UInt32MultiArray marker_list_msg_prev;
ros::ServiceServer service;
bool marker_list_changes =true;
bool checkarucoid = false;
public:
  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  {
     objectpose.resize(999);
   objectfilterpose.resize(999);
   foundmarker.resize(999);
   this->service= nh_.advertiseService("aruco_id_pose", &ArucoMarkerPublisher::add,this);
    image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);


    nh_.param<bool>("use_camera_info", useCamInfo_, true);
    if (useCamInfo_)
    {
      sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info", nh_); //, 10.0);

      nh_.param<double>("marker_size", marker_size_, 0.05);
      nh_.param<bool>("image_is_rectified", useRectifiedImages_, true);
      nh_.param<std::string>("reference_frame", reference_frame_, "");
      nh_.param<std::string>("camera_frame", camera_frame_, "");
      camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, useRectifiedImages_);
      ROS_ASSERT(not (camera_frame_.empty() and not reference_frame_.empty()));
      if (reference_frame_.empty())
        reference_frame_ = camera_frame_;
    }
    else
    {
      camParam_ = aruco::CameraParameters();
    }
    marker_pub_ = nh_.advertise<aruco_msgs::MarkerArray>("markers", 100);
    marker_list_pub_ = nh_.advertise<std_msgs::UInt32MultiArray>("markers_list", 10);
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose", 100);
    marker_msg_ = aruco_msgs::MarkerArray::Ptr(new aruco_msgs::MarkerArray());
    marker_msg_->header.frame_id = reference_frame_;
    marker_msg_->header.seq = 0;
  }

  bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
  {
    std::string errMsg;

    if (!tfListener_.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01),
                                      &errMsg))
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        tfListener_.lookupTransform(refFrame, childFrame, ros::Time(0), // get latest available
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
    foundmarker.resize(999);
    //bool publishMarkers = marker_pub_.getNumSubscribers() > 0;
    bool publishMarkersList = marker_list_pub_.getNumSubscribers() > 0;
    bool posepub=pose_pub.getNumSubscribers() > 0;
    //bool publishImage = image_pub_.getNumSubscribers() > 0;
    //bool publishDebug = debug_pub_.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage_ = cv_ptr->image;

      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
      // marker array publish

        marker_msg_->markers.clear();
        marker_msg_->markers.resize(markers_.size());
        marker_msg_->header.stamp = curr_stamp;
        marker_msg_->header.seq++;

        for (std::size_t i = 0; i < markers_.size(); ++i)
        {
          aruco_msgs::Marker & marker_i = marker_msg_->markers.at(i);
          marker_i.header.stamp = curr_stamp;
          marker_i.id = markers_.at(i).id;
          marker_i.confidence = 1.0;
        }
        
        // if there is camera info let's do 3D stuff
        if (useCamInfo_)
        {
          
          // get the current transform from the camera frame to output ref frame
          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();
       
          //offset.setRotation(transform.getRotation());

          if (reference_frame_ != camera_frame_)
          {
            getTransform(reference_frame_, camera_frame_, cameraToReference);
          }
          
          
          offset.setIdentity();
          offset.setOrigin(tf::Vector3(0.025, -0.045, 0.19));
          // now find the transform for each detected marker
          for (std::size_t i = 0; i < markers_.size(); ++i)
          {
            aruco_msgs::Marker & marker_i = marker_msg_->markers.at(i);
            tf::Transform transform = aruco_ros::arucoMarker2Tf(markers_[i]);
            
            //transform = static_cast<tf::Transform>(rightToLeft)* transform;
            geometry_msgs::PoseStamped posefliterMsg;
            tf::poseTFToMsg(transform, posefliterMsg.pose);
            posefliterMsg.header.frame_id = camera_frame_;
            posefliterMsg.header.stamp = curr_stamp;
            objectfilterpose[markers_[i].id] =posefliterMsg.pose;

            
            transform = static_cast<tf::Transform>(cameraToReference) * transform;
            //transform = transform * static_cast<tf::Transform>(offset);
            tf::poseTFToMsg(transform, marker_i.pose.pose);
            marker_i.header.frame_id = reference_frame_;
            objectpose[markers_[i].id] =marker_i.pose.pose;
            foundmarker[markers_[i].id] =1;
          }
         // only publishing the selected marker
          if (foundmarker[marker_id] && posepub)
          {
            geometry_msgs::PoseStamped poseMsg;
            poseMsg.header.frame_id = reference_frame_;
            poseMsg.header.stamp = curr_stamp;
            poseMsg.pose = objectpose[marker_id];
            pose_pub.publish(poseMsg);
          }

        // publish marker array
       if (marker_msg_->markers.size() > 0)
          marker_pub_.publish(marker_msg_);
        }

      if (publishMarkersList)
      {
        if(marker_list_msg_.data.size() !=markers_.size())
        { marker_list_changes =true;
        }
        marker_list_msg_.data.resize(markers_.size());
        for (std::size_t i = 0; i < markers_.size(); ++i)
          {
            if(marker_list_msg_.data[i] != markers_[i].id)
             {
               marker_list_msg_.data[i] = markers_[i].id;
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
    }
  }

bool add(aruco_ros::aruco_id_pose::Request  &req,
         aruco_ros::aruco_id_pose::Response &res)
{
   res.found = false;
  marker_id = req.id;
  if(foundmarker[marker_id] == 1){ 
  res.found =true;
  res.posesobject = objectpose[marker_id];
  res.posesfilter = objectfilterpose[marker_id];
  }
  else
  {
    res.found = false;
  }
    foundmarker.clear();
    foundmarker[marker_id] = 0;
    objectpose.clear();
    foundmarker.resize(999);
    objectpose.resize(999);
    return true;
}

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");
  ArucoMarkerPublisher node;
  ros::spin();
}
