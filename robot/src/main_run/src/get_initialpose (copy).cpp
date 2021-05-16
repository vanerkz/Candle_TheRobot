#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string>  
#include <iostream>  
#include <stdio.h>
#include "cartographer_ros_msgs/StartTrajectory.h"

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
    
    ros::ServiceClient client = n.serviceClient<cartographer_ros_msg::StartTrajectory>("start_trajectory");
        cartographer_ros_msg::StartTrajectory starttrajectory;
        setmodelstate.request.model_state = modelstate;



}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "err_eval");
    ros::NodeHandle n;
    ros::Subscriber sub_amcl = n.subscribe("initialpose",1000, poseAMCLCallback);
    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok())
    {

        geometry_msgs::Pose error;
        error.position.x = poseAMCLx;
        error.position.y = poseAMCLy;
        error.orientation.w = poseAMCLa;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
