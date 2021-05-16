/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Ridhwan Luthra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Ridhwan Luthra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ridhwan Luthra */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/Empty.h"

class CylinderSegment
{
public:
  void run();
  CylinderSegment()
  {
    // Initialize subscriber to the raw point cloud
    client2 = nh.serviceClient<std_srvs::Empty>("clear_octomap");
    sub2 = nh.subscribe("/filterarea", 30, &CylinderSegment::arrayCallback,this);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/points2", 30);
  }

   protected:
   tf::Transform transform;
   ros::Publisher pub;
   tf::StampedTransform newtransform;
   ros::NodeHandle nh;
   ros::Subscriber sub;
   ros::Subscriber sub2;
   ros::ServiceClient client;
   ros::ServiceClient client2;
   tf::TransformListener listener;
   float xfilter;
   float yfilter;
   float zfilter;
   float oxfilter;
   float oyfilter;
   float ozfilter;
   float condfilter =0;
   bool check =false;
   int oldcondfilter =0;
  
void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
	pcl::CropBox<pcl::PointXYZRGB> crop;
	crop.setInputCloud(cloud);
	double boxsize =0.15;
	double xmin=-boxsize;
	double ymin=-boxsize;
	double zmin=-boxsize;
	double xmax=boxsize;
	double ymax=boxsize;
	double zmax=boxsize;
	Eigen::Vector4f min_point = Eigen::Vector4f(xmin, ymin, zmin, 0);
	Eigen::Vector4f max_point = Eigen::Vector4f(xmax, ymax, zmax, 0);
	Eigen::Vector3f boxTranslatation;
	boxTranslatation[0]=xfilter;  
	boxTranslatation[1]=yfilter+ 0.15;  
        boxTranslatation[2]=zfilter;
	Eigen::Vector3f boxRotation;
        boxRotation[0]=oxfilter;  // rotation around x-axis
        boxRotation[1]=oyfilter;  // rotation around y-axis
        boxRotation[2]=ozfilter;  //in radians rotation around z-axis. this rotates your cube 45deg around z-axis.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut( new pcl::PointCloud<pcl::PointXYZRGB> );
    	crop.setInputCloud (cloud);
    	crop.setMin(min_point);
    	crop.setMax(max_point);
    	crop.setTranslation(boxTranslatation);
    	crop.setRotation(boxRotation);
    	crop.setNegative(true);
    	crop.filter(*cloud);

	/*double xmin2=-0.15;
	double ymin2=0;
	double zmin2=0;
	double xmax2=0.3;
	double ymax2=0.3+0.45;
	double zmax2=0.3;
	Eigen::Vector4f min_point2 = Eigen::Vector4f(xmin2, ymin2, zmin2, 0);
	Eigen::Vector4f max_point2 = Eigen::Vector4f(xmax2, ymax2, zmax2, 0);
    	Eigen::Vector3f boxTranslatation2;
    	boxTranslatation[0]=xfilter;  
    	boxTranslatation[1]=yfilter;  
    	boxTranslatation[2]=zfilter;  
    	crop.setInputCloud (cloud);
    	crop.setMin(min_point2);
    	crop.setMax(max_point2);
    	crop.setRotation(boxRotation);
    	crop.setNegative(true);
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2( new pcl::PointCloud<pcl::PointXYZRGB> );
    	crop.filter(*cloud2);*/
    	pub.publish(cloud);
}

 void clearcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudfunctionclear)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloudfunctionclear);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.0, 0.0);
     pass.setFilterFieldName("y");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.0, 0.0);
     pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.0, 0.0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutclear( new pcl::PointCloud<pcl::PointXYZRGB> );
    pass.filter(*cloudOutclear);
    pub.publish(cloudOutclear);
  }
float Arr[90];

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	condfilter =   array->data[0];
	if (oldcondfilter != condfilter && condfilter ==1)
	{      
        xfilter =   array->data[1];
        yfilter =   array->data[2];
        zfilter =   array->data[3]; 
        oxfilter =   array->data[4];
        oyfilter =   array->data[5];
        ozfilter =   array->data[6]; 
	ROS_INFO("recevied");
        sub = nh.subscribe("/camera/depth/color/points", 1, &CylinderSegment::cloudCB, this);
        check =true;
	}
	if (oldcondfilter != condfilter)
	{
		oldcondfilter = condfilter;
	}
}

void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudinput(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloudinput);

    if(condfilter == 1) 
    {
      passThroughFilter(cloudinput);
    }
    else
    {
      if(check){
      clearcloud(cloudinput);
      std_srvs::Empty srv;
      	if (client2.call(srv))
	{
		ROS_INFO("sucesss");
	}
	else
	{
		ROS_ERROR("Failed to clear");
	}
      check =false;
      }
    }

  }

};

void CylinderSegment::run()
{
ros::Rate r(30);

while(nh.ok()){

ros::spinOnce();
r.sleep();
}

}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cylinder_segment");
  // Start the segmentor
  CylinderSegment newclass;
  newclass.run();

}
