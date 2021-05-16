#include <iostream>
#include "feetechlib/SCServo.h"
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

class myClass
{
public:
void checkmain(char*);
void readservo(int, int);

myClass() {
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    feetech_listener = n.subscribe("/move_group/fake_controller_joint_states", 1,&myClass::chatterCallback,this);
    }

protected:
ros::NodeHandle n;
    ros::Publisher feetech_states_pub;
    ros::Publisher joint_pub;
    ros::Subscriber feetech_listener;
    sensor_msgs::JointState joint_state;
SMSBL sm;
int error =0;
u8 ID[7] = {1,2,3,4,5, 6,7};
s16 Position[7];
u16 Speed[7] = {500, 0,500,500,500, 500, 500};
u8 ACC[7] = {100, 100, 100, 100 , 100, 100, 100};
int Pos;

void chatterCallback(const sensor_msgs::JointState::ConstPtr &msg){
sensor_msgs::JointState newmsg= *msg;
Position[0] = (-newmsg.position[0]+3.14)*(180/3.14)/(0.08791);
Position[1] = (-newmsg.position[1]+3.14)*(180/3.14)/(0.08791);
Position[2] = (-newmsg.position[2]+3.14)*(180/3.14)/(0.08791);
Position[3] = (-newmsg.position[3]+3.14)*(180/3.14)/(0.08791);
Position[4] = (-newmsg.position[4]+3.14)*(180/3.14)/(0.08791);
Position[5] = (-newmsg.position[5]+3.14)*(180/3.14)/(0.08791);
Position[6] = (-newmsg.position[6]+3.14)*(180/3.14)/(0.08791);
sm.SyncWritePosEx(ID, 7, Position, Speed, ACC);
ROS_INFO("Servo Data Received");
 }
};

void myClass::readservo(int start, int end)
{

if(sm.FeedBack(start)!=-1){
Pos = sm.ReadPos(start);
double out= ((-(Pos*0.08791)*(6.283185307/360))+3.14);
joint_state.position[start-1] = out;
}
else{
std::cout<< "read err ="<<std::endl;
//sleep(2);
}

if (start ==end)
return;
start++;
myClass::readservo(start, end);
}

void myClass::checkmain(char* argv)
{
sm.begin(115200, argv);
ros::Rate loop_rate(10);
while (n.ok())
{
ros::spinOnce();
joint_state.header.stamp = ros::Time::now();
joint_state.name.resize(7);
joint_state.position.resize(7);
joint_state.name[0] ="joint_1";
joint_state.name[1] ="joint_2";
joint_state.name[2] ="joint_3";
joint_state.name[3] ="joint_4";
joint_state.name[4] ="joint_5";
joint_state.name[5] ="joint_6";
joint_state.name[6] ="joint_7";
myClass::readservo(1, 7);
		/*int Pos;
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
			//std::cout<< "pos ="<<Pos<<std::endl;
                        double out= (Pos*0.08791)*(3.14/180);
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
                //send the joint state and transform*/
joint_pub.publish(joint_state);
                loop_rate.sleep();
                }
//return 0;
	sm.end();
}


SMSBL sm;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_publisher");

	if(argc<2){
        ROS_INFO("argc error!");
        return 0;
	}
	std::cout<< "serial:"<<argv[1]<<std::endl;

    if(!sm.begin(115200, argv[1])){
        ROS_INFO("Failed to init smsbl motor!");
        return 0;
    }

  myClass myObject;
  myObject.checkmain(argv[1]);
  ros::spin();
}

