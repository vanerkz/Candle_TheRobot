#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string>  
#include <iostream>  
#include <stdio.h>
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"

class myClass{ 
    public:
    void poseAMCLCallback();

    myClass() {
    sub = n.subscribe("initialpose",1000,&myClass::poseAMCLCallback,this);
    client = n.serviceClient<cartographer_ros_msgs::StartTrajectory>("start_trajectory");
    client2 = n.serviceClient<cartographer_ros_msgs::FinishTrajectory>("finish_trajectory");
    }

    protected:
    ros::Subscriber sub;
    ros::NodeHandle n;
    ros::ServiceClient client, client2;
    cartographer_ros_msgs::StartTrajectory srv;
    cartographer_ros_msgs::FinishTrajectory srv2;
    int state =0;
    double poseAMCLx, poseAMCLy, poseAMCLa, poseAMCLz;
    std::string zs ="";

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    poseAMCLa = msgAMCL->pose.pose.orientation.w;   
    poseAMCLz = msgAMCL->pose.pose.orientation.z;   
    zs = std:: to_string(poseAMCLx);
    ROS_INFO(zs.c_str());
    zs = std:: to_string(poseAMCLy);
    ROS_INFO(zs.c_str());
    zs = std:: to_string(poseAMCLa);
    ROS_INFO(zs.c_str());
    zs = std:: to_string(poseAMCLz);
    ROS_INFO(zs.c_str());
    srv2.request.trajectory_id = state;
    srv.request.use_initial_pose =true;
    srv.request.configuration_directory ="/home/robot/catkin_ws_isolated/src/cartographer_ros/cartographer_ros/configuration_files";
    srv.request.configuration_basename ="backpack_2d_localization.lua";
    srv.request.initial_pose.position =msgAMCL->pose.pose.position;
    srv.request.initial_pose.orientation =msgAMCL->pose.pose.orientation;
    srv.request.relative_to_trajectory_id =0;
   
    if(client2.call(srv2))
      {
        ROS_INFO("Received call 1 !!! ");
      }
     else
        {
            ROS_ERROR("Failed to call service 1 ");
            
        }
    if(client.call(srv))
      {
       state++;
        ROS_INFO("Received call 2!!!");
      }
     else
        {
            ROS_ERROR("Failed to call service 2 ");
            
        }
}

};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "getpose");
  myClass myObject;
  ros::spin();
}
