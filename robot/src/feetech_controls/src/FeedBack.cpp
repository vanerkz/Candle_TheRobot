#include <iostream>
#include "feetechlib/SCServo.h"
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

SMSBL sm;


int main(int argc, char** argv)
{

ros::init(argc, argv, "state_publisher");
ros::NodeHandle n;
ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
ros::Rate loop_rate(10);
 // robot state
sensor_msgs::JointState joint_state;

	if(argc<2){
        ROS_INFO("argc error");
        return 0;
	}
	std::cout<< "serial:"<<argv[1]<<std::endl;
     if(!sm.begin(1000000, argv[1])){
        ROS_INFO("Failed to init smsbl motor!");
        return 0;
        }


while (ros::ok()) {
joint_state.header.stamp = ros::Time::now();
joint_state.name.resize(1);
joint_state.position.resize(1);
joint_state.name[0] ="servo";
//sm.CalibrationOfs(1);

		int Pos;
		int Speed;
		int Load;
		int Voltage;
		int Temper;
		int Move;
		int Current;
		if(sm.FeedBack(1)!=-1){
			Pos = sm.ReadPos(1);
			//Speed = sm.ReadSpeed(1);
			//Load = sm.ReadLoad(1);
			//Voltage = sm.ReadVoltage(1);
			//Temper = sm.ReadTemper(1);
			//Move = sm.ReadMove(1);
			//Current = sm.ReadCurrent(1);
			std::cout<< "pos ="<<Pos<<std::endl;
                        double out= (Pos*0.088)*(3.14/180);
                       joint_state.position[0] = out;
			////std::cout<< "Speed ="<<Speed<<std::endl;
			//std::cout<< "Load ="<<Load<<std::endl;
			//std::cout<< "Voltage ="<<Voltage<<std::endl;
			//std::cout<< "Temper ="<<Temper<<std::endl;
			//std::cout<< "Move ="<<Move<<std::endl;
			//std::cout<< "Current ="<<Current<<std::endl;
			//usleep(10*1000);
		}
                else{
			std::cout<< "read err ="<<std::endl;
			//sleep(2);
		}
//send the joint state and transform
joint_pub.publish(joint_state);
loop_rate.sleep();
	}
	sm.end();
	return 1;
}


