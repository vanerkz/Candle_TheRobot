#include <iostream>
#include "feetechlib/SCServo.h"
#include "ros/ros.h"
SMSBL sm;

int main(int argc, char** argv)
{

	if(argc<2){
        ROS_INFO("argc error");
        return 0;
	}
	std::cout<< "serial:"<<argv[1]<<std::endl;
    if(!sm.begin(115200, argv[1])){
        ROS_INFO("Failed to init smsbl motor!");
        return 0;
    }
	sm.CalibrationOfs(1);
//sm.EnableTorque(1, 1);

	sm.end();
	return 1;
}

