#include "ros/ros.h"
#include <string>  
#include <iostream>  
#include <stdio.h>"
 #include "ros/master.h"
#include "std_srvs/Empty.h"
#include <cstdlib>


bool shutdown(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  std::vector<std::string> v;
  ros::master::getNodes(v);
  for (int i = 0; i < v.size(); ++i)
  {
    ROS_INFO("%s", v[i].c_str()); 
    
    if(!v[i].compare("/ROBOT_hardware_interface_node"))
    {
        
	system("rosnode kill /ROBOT_hardware_interface_node");

    }
    if(!v[i].compare("/controller_spawner"))
    {
	system("rosnode kill /controller_spawner");
    }

  }
  //return true;
  //bool run = true;
  return true;
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "nodecontroller");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("/shutdown", shutdown);
    ros::spin();
    return 0;
}

