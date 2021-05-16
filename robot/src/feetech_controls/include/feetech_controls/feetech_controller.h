#ifndef FEETECH_CONTROLLER_H
#define FEETECH_CONTROLLER_H

#include <iostream>
#include "feetechlib/SCServo.h"
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

namespace feetech_controller
{
	class FeetechController
        {
	     public:
	        
		void checkmain(char*);
		void torqenable(int);
		void torqdisable(int);
		double readservo(int);
                void set_joint(double, int);
		int init(int, char**);
	     protected:
	       ros::NodeHandle n;
    	       ros::Publisher feetech_states_pub;
    	       ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    	       ros::Subscriber feetech_listener;
    	       sensor_msgs::JointState joint_state;
	       SMSBL sm;
	       int error =0;
	       u8 ID[10] = {0,1,2,3,4,5, 6,7,8,9};
	       s16 Position[10]={2047, 2047, 2047,2047,2047,2047, 2047, 2047,2047,2047};
	       u16 Speed[10] = {0, 0, 0,0,0,0, 0, 0,0,0};
	       u8 ACC[10] = {0, 0, 0, 0, 0,0, 0, 0,0,0};
               u8 IDtemp[2] = {8,9};
               s16 Positiontemp[2]={2047, 2047};
               u16 Speedtemp[2] = {0,0};
               u8 ACCtemp[2] = {250,250};
	       int Pos;
               

	 };

}

#endif //FEETECH_CONTROLLER_H
