#include <sstream>
#include "../include/ROBOT_hardware_interface/ROBOT_hardware_interface.h"
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <ros/ros.h>
#include <string> 


//#include <ROBOTcpp/ROBOT.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;



namespace ROBOT_hardware_interface
{
bool runrec = false;
    ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh, int argc, char** argv) : nh_(nh) {
        feetechcontroller.init(argc,argv);
        init();
	// Get joint names
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/feetech_arm/loop_hz", loop_hz_, 0.1);
        ROS_INFO("%f", loop_hz_);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
 }

  

    ROBOTHardwareInterface::~ROBOTHardwareInterface() {
	
    }
    

	  
    

    void ROBOTHardwareInterface::init() {
        nh_.getParam("/feetech_arm/joints", joint_names_);
        double check = joint_names_.size();
        num_joints_ = joint_names_.size();
	ROS_INFO("%f", check);

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);


        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {


            joint_position_[i] = feetechcontroller.readservo(i);
	    //joint_position_[i] = 0;

            hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
             
            joint_state_interface_.registerHandle(jointStateHandle);

            

            // Create position joint interface
            hardware_interface::JointHandle jointPositionHandle(joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]);
            position_joint_interface_.registerHandle(jointPositionHandle);

            JointLimits limits;
            SoftJointLimits softLimits;
            getJointLimits(joint_names_[i], nh_, limits);
            PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
            positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
            joint_position_command_[i] = 0;
            if(i==9) joint_position_command_[9] = -0.35;

        }
        
	registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        //registerInterface(&effort_joint_interface_);
        registerInterface(&positionJointSoftLimitsInterface);
        read();
        initvalues();
    }


    void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
        read();
       
    }

    

    void ROBOTHardwareInterface::read() {
         for (int i = 0; i < num_joints_; i++) {
            joint_position_[i] = feetechcontroller.readservo(i);
        }
    }

    double Position_pre[10]={0,0,0,0,0,0,0,0,0,0};
    double Position_send[10]={0,0,0,0,0,0,0,0,0,0};
    double display [10];
    bool inital2 =true;

    void ROBOTHardwareInterface::initvalues(){


         for (int i = 0; i < num_joints_-2; i++) {
            joint_position_command_[i]=joint_position_[i];
            }

        } 
   bool inital= true;  

    void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
        positionJointSoftLimitsInterface.enforceLimits(elapsed_time);
         position_joint_saturation_interface_.enforceLimits(elapsed_time);
       position_joint_limits_interface_.enforceLimits(elapsed_time);
       
       for (int i = 0; i < num_joints_; i++) {
       feetechcontroller.set_joint(joint_position_command_[i], i);
   
           
            Position_send[i] = (-joint_position_command_[i]+3.14)*(180/3.14)/(0.088);
	    display[i] =Position_send[i];

	    //joint_position_command_[i] = 0.0;
	    
        }
    }
}
