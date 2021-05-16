#include "ros/ros.h"
#include <string>  
#include <iostream>  
#include <stdio.h>"
 #include "ros/master.h"
#include "std_msgs/Empty.h"
#include <cstdlib>

class myClass{ 
    public:
    void launchloop();

    myClass() {
    sub = n.subscribe("/launch", 1,&myClass::launch,this);
    }

    protected:

    ros::Subscriber sub;
    ros::NodeHandle n;
    bool recdata =false;

void launch(const std_msgs::Empty &vel)
	{
	    recdata =true;
    
	 }
};

void myClass::launchloop(){

    ros::Rate r(10);

    while(n.ok()){
    ros::spinOnce(); 
    
    	if(recdata)
   	{
	recdata=false;
	bool check =false;
	bool check2=false;
	std::vector<std::string> v;
	ros::master::getNodes(v);
	  for (int i = 0; i < v.size(); ++i)
	  {
	    ROS_INFO("%s", v[i].c_str()); 
            
	    if(!v[i].compare("/controller_spawner"))
	    {
		check=true;
	    }
            if(!v[i].compare("/ROBOT_hardware_interface_node"))
	    {
		check2=true;
	    }
  	  }
        if(!check && !check2)
	{	
	std::system("roslaunch feetech_controls feetech_control_interface.launch ");
	}

        }
    r.sleep();
 }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "nodelaunch");
    ros::NodeHandle nh;
     myClass myObject;
  myObject.launchloop();
  ros::spin();
    return 0;
}

