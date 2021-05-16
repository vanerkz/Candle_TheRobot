#ifndef ROS_CONTROL__ROBOT_HARDWARE_H
#define ROS_CONTROL__ROBOT_HARDWARE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <feetechlib/SCServo.h>
#include "../feetech_controls/feetech_controller.h"

namespace ROBOT_hardware_interface
{
    /// \brief Hardware interface for a robot
    class ROBOTHardware : public hardware_interface::RobotHW 
    {
        protected:
            // Interfaces
            hardware_interface::JointStateInterface joint_state_interface_;
            hardware_interface::PositionJointInterface position_joint_interface_;
            hardware_interface::VelocityJointInterface velocity_joint_interface_;
            hardware_interface::EffortJointInterface effort_joint_interface_;

            joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
            joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limits_interface_;
            joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
            joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
            joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
            joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;

            // Custom or available transmissions
            // transmission_interface::RRBOTTransmission rrbot_trans_;
            // std::vector<transmission_interface::SimpleTransmission> simple_trans_;

            // Shared memory
            int num_joints_;
            int joint_mode_; // position, velocity, or effort
            std::vector<std::string> joint_names_;
            std::vector<int> joint_types_;
            std::vector<double> joint_position_;
            std::vector<double> joint_velocity_;
            std::vector<double> joint_effort_;
            std::vector<double> joint_position_command_;
            std::vector<double> joint_velocity_command_;
            std::vector<double> joint_effort_command_;
            std::vector<double> joint_lower_limits_;
            std::vector<double> joint_upper_limits_;
            std::vector<double> joint_effort_limits_;
	    
	    //For feetech arm control
            SMSBL sm;
	    u8 ID[10] = {0,1,2,3,4,5, 6,7,8,9};
	    double Position[10] ={0,0,0,0,0,0, 0,0,0,0};
	    u16 Speed[10] = {1000,1000, 1000,1000,1000,1000, 1000, 1000,1000,1000};
	    u8 ACC[10] = {100, 100, 100, 100, 100 , 100, 100, 100,100,100};
            
    }; // class

} // namespace

#endif
