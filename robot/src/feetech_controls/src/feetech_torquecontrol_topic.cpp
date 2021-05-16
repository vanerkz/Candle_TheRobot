#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <string>  
#include <iostream>  
#include "../include/feetech_controls/feetech_controller.h"
#include "feetech_controls/torqcontrol.h"
#include "feetechlib/SCServo.h"
#include "std_srvs/Empty.h"
#include "feetech_controls/jointcontrols.h"
#include "feetech_controls/jointfeedback.h"
#include "feetech_controls/readjointvalues.h"

class myClass{ 
    public:
     void checkmain(char*);
void readservo(int, int);
    bool torqcontrolfunc();
    bool getjointfunc();

    myClass() {
    service= n.advertiseService("torqcontrol", &myClass::torqcontrolfunc,this);
    service2= n.advertiseService("getjointvalues", &myClass::getjointfunc,this);
    joint_pub = n.advertise<feetech_controls::jointfeedback>("joint_output", 1);
    feetech_listener = n.subscribe("joint_input", 1,&myClass::chatterCallback,this);
    feetech_listener_sync = n.subscribe("joint_input_sync", 1,&myClass::chatterCallback2,this);
    client = n.serviceClient<std_srvs::Empty>("/shutdown");
    if(!sm.begin(115200, "/dev/feetech")){
        ROS_INFO("Failed to init smsbl motor!");
	}
       
    }

    protected:
    SMSBL sm;
    ros::ServiceServer service;
    ros::ServiceServer service2;
    ros::ServiceClient client;
    ros::NodeHandle n;
    ros::Subscriber feetech_listener;
    ros::Subscriber feetech_listener_sync;
    ros::Publisher joint_pub;
    feetech_controls::jointfeedback readjoints;
    int move[10] = {0,0,0,0,0,0, 0,0,0,0};
    bool condmove =true;
    int error =0;
	u8 ID[10] = {0,1,2,3,4,5, 6,7,8,9};
	s16 Position[10]={2047, 2047,2047,2047,2047, 2047, 2047,2047,2047,2047};
	u16 Speed[10] = {500, 500,500,500,500, 500, 500,500,500,500};
	u8 ACC[10] = {100, 100, 100, 100 , 100, 100, 100,100,100,100};
	int Pos;

void chatterCallback(const feetech_controls::jointcontrols msg){
double calvalue = (-msg.jointvalues+3.14)*(180/3.14)/(0.08791);
sm.WritePosEx(msg.id, calvalue, Speed[msg.id], ACC[msg.id]);
ROS_INFO("Servo Data Received");
/*condmove= true;
while(condmove)
{
	int checkcount =0;
	for(int i=0;i<10;i++)
	{
		if(sm.FeedBack(i)!=-1){
		Pos = sm.ReadPos(i);
		double out= ((-(Pos*0.08791)*(6.283185307/360))+3.14);
		readjoints.jointvalues.push_back(out);
		ROS_INFO("%d:%f",i,out);
		}
		if(sm.ReadMove(i) ==0)
		{
		   checkcount++;
		}
        }	
	if (checkcount ==10) condmove=false;
joint_pub.publish(readjoints);
}*/
}

void chatterCallback2(const feetech_controls::jointfeedback &msg){
if(msg.jointvalues.size() ==8){
s16 Positionarm[8]={2047, 2047,2047,2047,2047, 2047, 2047,2047};
Positionarm[0] = (-msg.jointvalues[0]+3.14)*(180/3.14)/(0.08791);
Positionarm[1] = (-msg.jointvalues[1]+3.14)*(180/3.14)/(0.08791);
Positionarm[2] = (-msg.jointvalues[2]+3.14)*(180/3.14)/(0.08791);
Positionarm[3] = (-msg.jointvalues[3]+3.14)*(180/3.14)/(0.08791);
Positionarm[4] = (-msg.jointvalues[4]+3.14)*(180/3.14)/(0.08791);
Positionarm[5] = (-msg.jointvalues[5]+3.14)*(180/3.14)/(0.08791);
Positionarm[6] = (-msg.jointvalues[6]+3.14)*(180/3.14)/(0.08791);
Positionarm[7] = (-msg.jointvalues[7]+3.14)*(180/3.14)/(0.08791);
u8 IDarm[8] = {0,1,2,3,4,5, 6,7};
u16 Speed[8] = {500, 500,500,500,500, 500, 500,500};
u8 ACC[8] = {100, 100, 100, 100 , 100, 100, 100,100};
sm.SyncWritePosEx(IDarm, 8, Position, Speed, ACC);
}
else if(msg.jointvalues.size() ==2){
s16 Position[2]={2047,2047};
Position[8] = (-msg.jointvalues[8]+3.14)*(180/3.14)/(0.08791);
Position[9] = (-msg.jointvalues[9]+3.14)*(180/3.14)/(0.08791);
u8 IDhead[2] = {8,9};
u16 Speed[2] = {500,500};
u8 ACC[2] = {100,100};
sm.SyncWritePosEx(IDhead, 2, Position, Speed, ACC);
}
ROS_INFO("Servo Data Received");
}
    
bool torqcontrolfunc(feetech_controls::torqcontrol::Request  &req, feetech_controls::torqcontrol::Response &res)
  {
  for(int i=0  ; i<10; i++)
	{
	  if (req.idrec[i] == 0)
	  {
	   sm.EnableTorque(i,0);
	   ROS_INFO("disable %d",i);
	  } 
	  else
	  {
	   sm.EnableTorque(i,1);
	   ROS_INFO("enable %d",i);
	  }
	}
  return true;
}

bool getjointfunc(feetech_controls::readjointvalues::Request  &req, feetech_controls::readjointvalues::Response &res)
{;
condmove= true;
while(condmove)
{
        res.jointreadvalues.clear();
	int checkcount =0;
	int size = req.idend -req.idstart +1;
	for(int i=req.idstart;i<=req.idend;i++)
	{
		
		if(sm.FeedBack(i)!=-1){
		Pos = sm.ReadPos(i);
		double out= ((-(Pos*0.08791)*(6.283185307/360))+3.14);
		res.jointreadvalues.push_back(out);
		ROS_INFO("%d:%f",i,out);
		}
		if(sm.ReadMove(i) ==0)
		{
		   checkcount++;
		}
        }	
	if (checkcount ==size) condmove=false;
}
	return true;
}

};

void myClass::checkmain(char* argv)
{
ros::Rate loop_rate(10);
	while (n.ok())
	{
	ros::spinOnce();
	
	loop_rate.sleep();
	
	}
}



int main (int argc, char **argv)
{
  ros::init(argc, argv, "torqcontrolnode");
  myClass object;
  object.checkmain("/dev/feetech");
  ros::spin();
}
