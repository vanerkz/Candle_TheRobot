#include <moveit/move_group_interface/move_group_interface.h>  
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "ros/ros.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <stdlib.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aruco_ros/aruco_id_pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/Int16.h"
#include "std_srvs/Empty.h"
#include "feetech_controls/headcontrols.h"
#include "feetech_controls/jointfeedback.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

class myClass{
    public:
    
    myClass() {
    sub = node_handle.subscribe("/aruco/getid", 1000, &myClass::getidcallback,this);
   sub2 = node_handle.subscribe("/jointarrayseq", 1000, &myClass::jointarraycontrol,this);
    client = node_handle.serviceClient<aruco_ros::aruco_id_pose>("/aruco_single/aruco_id_pose");
    pub = node_handle.advertise<std_msgs::Float32MultiArray>("/filterarea", 10);
    pub2 = node_handle.advertise<std_msgs::String>("/arm_status_aruco", 10);
    //pub3 = node_handle.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    pub4 = node_handle.advertise<std_msgs::Bool>("/setrepubaruco", 10);
    static const std::string PLANNING_GROUP2 = "feetech_head";
    static const std::string PLANNING_GROUP = "feetech_arm";
    move_group2 = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP2);
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    }
    
    protected:
    tf::TransformListener listener;
    aruco_ros::aruco_id_pose srv;
    ros::ServiceClient client;
    moveit::planning_interface::MoveGroupInterface *move_group2;
    moveit::planning_interface::MoveGroupInterface *move_group;
    bool check=false;
    double newoldx =0;
    double newoldy =0;
    double newoldz =0;
    double newoldx2 =0;
    double newoldy2 =0;
    double newoldz2 =0;
    double setwaist =0;
    double objdistance;
    const robot_state::JointModelGroup* joint_model_group2;
    std::vector<double> joint_group_positions2;
    std::vector<double> joint_group_positions1;
    std::vector<double> joint_group_positions3;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    geometry_msgs::Pose target_pose1;
    geometry_msgs::Pose tfconvert;
    geometry_msgs::Pose recepostconvert;
    tf2::Vector3 posevector;
    tf2::Vector3 xvector;
    tf2::Vector3 newvector;
    tf2::Vector3 hookvector;
    geometry_msgs::Point vertialvector;
    geometry_msgs::Point offsetpoint;
    geometry_msgs::Point retpoint;
    geometry_msgs::Point temppoint;
    ros::NodeHandle node_handle;
    ros::Subscriber sub;
    ros::Subscriber sub2;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Publisher pub4;
    tf::StampedTransform transform;
    std_msgs::Float32MultiArray array;
    tf::TransformListener _tfListener;
    std_msgs::Bool pub4cond;
    std_msgs::String pubstring1;
    

geometry_msgs::Pose offsetconvert(geometry_msgs::Pose inpose, double x, double y,double z )
{
  tf::StampedTransform offset;
  offset.setIdentity();
  offset.setOrigin(tf::Vector3(x, y, z));
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(inpose.position.x,inpose.position.y,inpose.position.z));
  transform.setRotation(tf::Quaternion(inpose.orientation.x,inpose.orientation.y,inpose.orientation.z,inpose.orientation.w));
  transform= transform * static_cast<tf::Transform>(offset);
  geometry_msgs::Pose poseMsg;
  tf::poseTFToMsg(transform, poseMsg);
  return poseMsg;
}

bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
{
    std::string errMsg;
    if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01),
                                      &errMsg))
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), // get latest available
                                    transform);
      }
      catch (const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }
    }
    return true;
}

void replanpath(geometry_msgs::Pose navposeinput)
{
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static tf::TransformBroadcaster br;
  geometry_msgs::Pose newnav=offsetconvert(navposeinput,0,0,0.7);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(newnav.position.x,newnav.position.y,newnav.position.z));   	 		transform.setRotation(tf::Quaternion(newnav.orientation.x,newnav.orientation.y,newnav.orientation.z,newnav.orientation.w));
tf::StampedTransform newreference;
	getTransform("/map", "/base_link", newreference);
	transform = static_cast<tf::Transform>(newreference) * transform;
  geometry_msgs::PoseStamped navposeMsg;
  tf::poseTFToMsg(transform, navposeMsg.pose);
  tf::Quaternion q = newreference.getRotation();
  navposeMsg.pose.orientation.x = q.x();
  navposeMsg.pose.orientation.y = q.y();
  navposeMsg.pose.orientation.z = q.z();
  navposeMsg.pose.orientation.w = q.w();
  navposeMsg.pose.position.z = 0;
  navposeMsg.header.frame_id = "map";
  navposeMsg.header.stamp = ros::Time(0);
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.seq=1;
  goal.target_pose.pose = navposeMsg.pose;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header= navposeMsg.header;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal); 
  //visualise from RVIZ
  //tf::Transform pubtransform = transform;
  //pubtransform.setOrigin(tf::Vector3(pubtransform.getOrigin().x(),pubtransform.getOrigin().y(),0));   	 
  //pubtransform.setRotation(q);
  //tf::StampedTransform stampedTransform(pubtransform, ros::Time(0), "map", "newnav");
  //br.sendTransform(stampedTransform);
}

void jointarraycontrol(const feetech_controls::jointfeedback &msg){

 ROS_INFO("Joint Seq Recevied %d",msg.jointvalues.size());
bool success;
static const std::string PLANNING_GROUP = "feetech_arm";
//moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
const robot_state::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
			move_group->setMaxVelocityScalingFactor(0.25);
			move_group->setMaxAccelerationScalingFactor(1.0);
			moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
			std::vector<double> joint_group_positions;
			current_state->copyJointGroupPositions(joint_model_group, joint_group_positions1);
			moveit::planning_interface::MoveGroupInterface::Plan my_plan;
			geometry_msgs::Pose currentpose = move_group->getCurrentPose().pose;

for(int i = 0; i < msg.jointvalues.size(); i=i+8)
{

			

				joint_group_positions1[0] = msg.jointvalues[i];  // radians
				joint_group_positions1[1] = msg.jointvalues[i+1];  // radians
				joint_group_positions1[2] = msg.jointvalues[i+2];  // radians
				joint_group_positions1[3] = msg.jointvalues[i+3];  // radians
				joint_group_positions1[4] = msg.jointvalues[i+4];  // radians
				joint_group_positions1[5] = msg.jointvalues[i+5];  // radians
				joint_group_positions1[6] = msg.jointvalues[i+6];  // radians
				joint_group_positions1[7] = msg.jointvalues[i+7];  // radians
				move_group->setJointValueTarget(joint_group_positions1);
				success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO_NAMED("tutorial", "Visualizing plan %d (pose goal) %s", i, success ? "" : "FAILED");
				move_group->execute(my_plan);
}

                                ROS_INFO("Arm motion completed");
                                pub2.publish(pubstring1);	
}

void getidcallback(const feetech_controls::headcontrols msg)
{
   headupdate(msg.headjointvalues1,msg.headjointvalues2);
   sleep(1);
   srv.response.found = false;
   srv.request.id =msg.arucoid;

	if (client.call(srv))
  	{
    client.call(srv);
		if(!srv.response.found)
		{
			double newheadjointvalues =msg.headjointvalues1 -0.4;
			if(newheadjointvalues >3.14) newheadjointvalues =3.14;
			headupdate(newheadjointvalues,msg.headjointvalues2);
			sleep(1);
		  client.call(srv);
			if(!srv.response.found)
			{
				double newheadjointvalues =msg.headjointvalues1 +0.4;
		    if(newheadjointvalues < -3.14) newheadjointvalues =-3.14;
				headupdate(newheadjointvalues,msg.headjointvalues2);
				sleep(1);
		    client.call(srv);
			}
		}
	
   if(srv.response.found && !check)
		{
		ROS_INFO("Found");
		tf::Quaternion q(srv.response.posesfilter.orientation.x, srv.response.posesfilter.orientation.y, 	srv.response.posesfilter.orientation.z,  srv.response.posesfilter.orientation.w);
		double roll, pitch, yaw;
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		array.data.clear();
		array.data.push_back(1.0);
		array.data.push_back(srv.response.posesfilter.position.x);
		array.data.push_back(srv.response.posesfilter.position.y);
		array.data.push_back(srv.response.posesfilter.position.z);
		array.data.push_back(roll);
		array.data.push_back(pitch);
		array.data.push_back(yaw);
		pub.publish(array);
		target_pose1=srv.response.posesobject;
		objdistance = sqrt((abs(target_pose1.position.y)*abs(target_pose1.position.y))+(abs(target_pose1.position.x)*abs(target_pose1.position.x)));
		check = true;
                std::stringstream convertstring;
		convertstring<< msg.arucoid;
                pubstring1.data = "Aurco Id"+convertstring.str()+ "collected";
		switch(msg.type){
		case 1:
		armupdate();
		break;
		case 2:
		armupdate2();
		break;
		case 3:
		//armupdate3();
		break;
		}
    headupdatenonblock(0,-0.34);
		
		
		
		
		}
		else if(check)
		{
		        ROS_INFO("planning in progress");
		}
		else
		{
			ROS_INFO("Id not found");
			headupdatenonblock(0,-0.34);
			std_msgs::String pubstring2;
		pubstring2.data = "Aruco Id not found";
		pub2.publish(pubstring2);		
		}
	}
	else
	{
		ROS_INFO("calling failed");
	}
}

void headupdate(float firstjoint, float secondjoint)
{
	joint_group_positions2 =move_group2->getCurrentJointValues();
	bool success2;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
	move_group2->setMaxVelocityScalingFactor(1.0);
	move_group2->setMaxAccelerationScalingFactor(1.0);
	joint_group_positions2[0] = firstjoint;  // radians
	joint_group_positions2[1] = secondjoint;  // radians			
	move_group2->setJointValueTarget(joint_group_positions2);
	success2 = (move_group2->plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("headjoint", "Visualizing plan 1 (pose goal) %s", success2 ? "" : "FAILED");
	move_group2->execute(my_plan2);

}

void headupdatenonblock(float firstjoint, float secondjoint)
{
	joint_group_positions2 =move_group2->getCurrentJointValues();
	bool success2;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
	move_group2->setMaxVelocityScalingFactor(1.0);
	move_group2->setMaxAccelerationScalingFactor(1.0);
	joint_group_positions2[0] = firstjoint;  // radians
	joint_group_positions2[1] = secondjoint;  // radians		
	move_group2->setJointValueTarget(joint_group_positions2);
	success2 = (move_group2->plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("headjoint", "Visualizing plan 1 (pose goal) %s", success2 ? "" : "FAILED");
	move_group2->asyncExecute(my_plan2);
}

void armupdate2()
{
bool success;
static const std::string PLANNING_GROUP = "feetech_arm";
const robot_state::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
move_group->setMaxVelocityScalingFactor(0.1);
move_group->setMaxAccelerationScalingFactor(0.75);

			moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
			std::vector<double> joint_group_positions;
			current_state->copyJointGroupPositions(joint_model_group, joint_group_positions1);
			moveit::planning_interface::MoveGroupInterface::Plan my_plan;
			geometry_msgs::Pose currentpose = move_group->getCurrentPose().pose;

			if(objdistance <0.8)
			{
      target_pose1 = offsetconvert(target_pose1, 0.025, 0.115, 0.19); // leftright, updown ,front
			move_group->setPoseTarget(target_pose1);
 			success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");
      array.data.clear();
			array.data.push_back(0);
			pub.publish(array);
			move_group->execute(my_plan);			
			move_group->clearPoseTargets(); 

				//go front
				if(success)
				{
					success= false;
					std::vector<geometry_msgs::Pose> waypoints;
					waypoints.clear();
					//waypoints.push_back(target_pose1);
					geometry_msgs::Pose target_pose3=offsetconvert(target_pose1, 0,0,-0.105);
					waypoints.push_back(target_pose3);  

					target_pose3=offsetconvert(target_pose3, 0,-0.06,0);
					//target_pose3.position.z -= 0.05;
					waypoints.push_back(target_pose3);

					target_pose3 = offsetconvert(target_pose3, -0.045, 0, 0);
					waypoints.push_back(target_pose3); 
					
					target_pose3=offsetconvert(target_pose3, 0,-0.16,0);
					//target_pose3.position.z -= 0.11;
					waypoints.push_back(target_pose3);

					target_pose3 = offsetconvert(target_pose3, 0, 0, 0.105);
					waypoints.push_back(target_pose3); 

					moveit_msgs::RobotTrajectory trajectory;
				  	const double jump_threshold = 0;
				  	const double eef_step = 0.01;                            
				  	double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
				 	// The trajectory needs to be modified so it will include velocities as well.
				  	// First to create a RobotTrajectory object
				  	robot_trajectory::RobotTrajectory rt(move_group->getCurrentState()->getRobotModel(), "feetech_arm");

				  	// Second get a RobotTrajectory from trajectory
				  	rt.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);
				 
				  	// Thrid create a IterativeParabolicTimeParameterization object
				  	trajectory_processing::IterativeParabolicTimeParameterization iptp;
				  
				  	// Fourth compute computeTimeStamps
				  	success = iptp.computeTimeStamps(rt, 0.030, 1);
				  
				  	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

				  	// Get RobotTrajectory_msg from RobotTrajectory
				  	rt.getRobotTrajectoryMsg(trajectory);

				  	// Finally plan and execute the trajectory
				  	my_plan.trajectory_ = trajectory;
				  	ROS_INFO("Visualizing plan 5 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
				  	move_group->execute(my_plan);
                                        
				//-----------------------------------------------------------------------------------------------------  
				move_group->setMaxVelocityScalingFactor(0.25);                                    
					
				joint_group_positions1[0] = 0;  // radians
				joint_group_positions1[1] = 0;  // radians
				joint_group_positions1[2] = 1.57;  // radians
				joint_group_positions1[3] = 0;  // radians
				joint_group_positions1[4] = -1.55;  // radians
				joint_group_positions1[5] = 0;  // radians
				joint_group_positions1[6] = -1.55;  // radians
				joint_group_positions1[7] = 0;  // radians
				move_group->setJointValueTarget(joint_group_positions1);
				success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
				move_group->execute(my_plan);
                                pub2.publish(pubstring1);	
				}//if sucesss
			}//object too far
			else
			{     
				ROS_INFO("Object too far !!, replanning robot goal");
				array.data.clear();
				array.data.push_back(0);
				pub.publish(array);
                                  pub4cond.data =true;
				pub4.publish(pub4cond);
				replanpath(target_pose1);
			}

check =false;	
}

void armupdate()
{
 
bool success;
static const std::string PLANNING_GROUP = "feetech_arm";
//moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
const robot_state::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
			move_group->setMaxVelocityScalingFactor(0.25);
			move_group->setMaxAccelerationScalingFactor(1.0);
			moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
			std::vector<double> joint_group_positions;
			current_state->copyJointGroupPositions(joint_model_group, joint_group_positions1);
			moveit::planning_interface::MoveGroupInterface::Plan my_plan;
			geometry_msgs::Pose currentpose = move_group->getCurrentPose().pose;
			ROS_INFO("%f,%f,%f",currentpose.position.x,currentpose.position.y,currentpose.position.z);
			double offsetposevalue = 0.1;
			double offsetposevaluexmin =-0.312-offsetposevalue;
			double offsetposevaluexmax =-0.312+offsetposevalue;
			double offsetposevalueymin =-0.002-offsetposevalue;
			double offsetposevalueymax =-0.002+offsetposevalue;
			double offsetposevaluezmin =0.695-offsetposevalue;
			double offsetposevaluezmax =0.695+offsetposevalue;

			if(currentpose.position.x < offsetposevaluexmin || currentpose.position.x >  offsetposevaluexmax)
			{
				joint_group_positions1[0] = 0;  // radians
				joint_group_positions1[1] = 0;  // radians
				joint_group_positions1[2] = 1.57;  // radians
				joint_group_positions1[3] = 0;  // radians
				joint_group_positions1[4] = -1.55;  // radians
				joint_group_positions1[5] = 0;  // radians
				joint_group_positions1[6] = -1.55;  // radians
				joint_group_positions1[7] = 0;  // radians
				
				move_group->setJointValueTarget(joint_group_positions1);
				success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
				move_group->execute(my_plan);
			}

			if(objdistance <0.8)
			{
			target_pose1 = offsetconvert(target_pose1, 0.0, -0.135, 0.175);// leftright, updown ,front
			move_group->setPoseTarget(target_pose1);
 			success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");
			      			
			array.data.clear();
			array.data.push_back(0);
			pub.publish(array);
			move_group->execute(my_plan);			
			move_group->clearPoseTargets(); 
				//go front
				if(success)
				{
					success= false;
					std::vector<geometry_msgs::Pose> waypoints;
					waypoints.clear();
					geometry_msgs::Pose target_pose3=offsetconvert(target_pose1, 0,0,-0.0875);
					waypoints.push_back(target_pose3);  

					target_pose3=offsetconvert(target_pose3, 0,0.130,0);
					//target_pose3.position.z += 0.125;
					waypoints.push_back(target_pose3);  
					
					target_pose3 = offsetconvert(target_pose3, 0, 0, 0.08);
					waypoints.push_back(target_pose3);  

					moveit_msgs::RobotTrajectory trajectory;
				  const double jump_threshold = 0;
				  const double eef_step = 0.01;
                                        
				  double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
					
				 	// The trajectory needs to be modified so it will include velocities as well.
				  	// First to create a RobotTrajectory object
				  	robot_trajectory::RobotTrajectory rt(move_group->getCurrentState()->getRobotModel(), "feetech_arm");

				  	// Second get a RobotTrajectory from trajectory
				  	rt.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);
				 
				  	// Thrid create a IterativeParabolicTimeParameterization object
				  	trajectory_processing::IterativeParabolicTimeParameterization iptp;
				  
				  	// Fourth compute computeTimeStamps
				  	success = iptp.computeTimeStamps(rt, 0.1, 1);
				  
				  	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

				  	// Get RobotTrajectory_msg from RobotTrajectory
				  	rt.getRobotTrajectoryMsg(trajectory);

				  	// Finally plan and execute the trajectory
				  	my_plan.trajectory_ = trajectory;
				  	ROS_INFO("Visualizing plan 5 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
				  	move_group->execute(my_plan);
					//-----------------------------------------------------------------------------------------------------                                      
					moveit::planning_interface::MoveGroupInterface::Plan my_planhome;
					//move_group.clearPathConstraints();				 	
					//move_group.setStartStateToCurrentState();
					move_group->setMaxVelocityScalingFactor(0.15);
					current_state = move_group->getCurrentState();
					current_state->copyJointGroupPositions(joint_model_group, joint_group_positions1);			
					joint_group_positions1[0] = 0;  // radians
					joint_group_positions1[1] = 0;  // radians
					joint_group_positions1[2] = 1.57;  // radians
					joint_group_positions1[3] = 0;  // radians
					joint_group_positions1[4] = -1.55;  // radians
					joint_group_positions1[5] = 0;  // radians
					joint_group_positions1[6] = -1.95;  // radians
					joint_group_positions1[7] = 0;  // radians
					
					//joint_group_positions1[0] = 0;  // radians
					move_group->setMaxVelocityScalingFactor(0.25);
					move_group->setMaxAccelerationScalingFactor(1.0);
					move_group->setJointValueTarget(joint_group_positions1);
					success = (move_group->plan(my_planhome) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
					ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal) %s", success ? "" : "FAILED");
                                       
					move_group->execute(my_planhome);
                                        pub2.publish(pubstring1);
		//-----------------------------------------------------------------------------------------------------                                      
					/*moveit::planning_interface::MoveGroupInterface::Plan my_planhome2;
					//move_group.clearPathConstraints();				 	
					//move_group.setStartStateToCurrentState();
					  current_state = move_group->getCurrentState();
					  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions1);			
					joint_group_positions1[1] = 0;  // radians
					
					move_group->setMaxVelocityScalingFactor(0.35);
					move_group->setMaxAccelerationScalingFactor(1.0);
					move_group->setJointValueTarget(joint_group_positions1);
					success = (move_group->plan(my_planhome2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
					ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal) %s", success ? "" : "FAILED");
					move_group->execute(my_planhome2);*/
				//--------------------------------------------------------------------------------------------

					//move_group.setStartStateToCurrentState();
					/*geometry_msgs::Pose target_pose_last= move_group.getCurrentPose().pose;
					//target_pose_last.orientation.x=0.5;
					//target_pose_last.orientation.y=0.5;
					//target_pose_last.orientation.z=0.5;
					//target_pose_last.orientation.w=0.5;
					move_group.setPoseTarget(target_pose_last);
					success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
					ROS_INFO_NAMED("tutorial", "Visualizing plan reset (pose goal) %s", success ? "" : "FAILED");
					move_group.move();*/
					
					
					std::vector<geometry_msgs::Pose> waypoints2;
					move_group->setStartStateToCurrentState();
					geometry_msgs::Pose target_pose_last2;
					waypoints2.clear();
					/*target_pose_last2.orientation.x=0.575;
					target_pose_last2.orientation.y=0.553;
					target_pose_last2.orientation.z=0.422;
					target_pose_last2.orientation.w=0.431;
					target_pose_last2.position.x =-0.324;
					target_pose_last2.position.y =-0.021;
					target_pose_last2.position.z =0.900;
					waypoints2.push_back(target_pose_last2);
					moveit_msgs::RobotTrajectory trajectory2;
                                       

					  //const double jump_threshold = 0.0;
					  //const double eef_step = 0.01;
					  double fraction2 = move_group->computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);

					  // The trajectory needs to be modified so it will include velocities as well.
					  // First to create a RobotTrajectory object
					  robot_trajectory::RobotTrajectory rt2(move_group->getCurrentState()->getRobotModel(), "feetech_arm");

					  // Second get a RobotTrajectory from trajectory
					  rt2.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory2);
					 
					  // Thrid create a IterativeParabolicTimeParameterization object
					 trajectory_processing::IterativeParabolicTimeParameterization iptp2;
					  
					  // Fourth compute computeTimeStamps
					  success = iptp2.computeTimeStamps(rt2, 0.35, 1);
					  
					  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

					  // Get RobotTrajectory_msg from RobotTrajectory
					  rt2.getRobotTrajectoryMsg(trajectory2);

					  // Finally plan and execute the trajectory
					  my_plan.trajectory_ = trajectory2;
					  ROS_INFO("Visualizing plan 7 (cartesian path) (%.2f%% acheived)",fraction2 * 100.0);
                                          move_group->execute(my_plan);

joint_group_positions1[0] = 0;  // radians
				joint_group_positions1[1] = 0;  // radians
				joint_group_positions1[2] = 1.57;  // radians
				joint_group_positions1[3] = 0;  // radians
				joint_group_positions1[4] = -1.55;  // radians
				joint_group_positions1[5] = 0;  // radians
				joint_group_positions1[6] = -1.95;  // radians
				joint_group_positions1[7] = 0;  // radians
				
				move_group->setJointValueTarget(joint_group_positions1);
				success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
				move_group->execute(my_plan);*/
				
					
					
				}//if sucesss
			}//object too far
			else
			{
				ROS_INFO("Object too far !!, replanning robot goal");
				array.data.clear();
				array.data.push_back(0);
				pub.publish(array);
				  pub4cond.data =true;
				pub4.publish(pub4cond);
				replanpath(target_pose1);
			}

check =false;	
}
};


int main(int argc, char** argv)
{
   
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::MultiThreadedSpinner s(0);   // Use 4 threads  
  myClass myObject;
  ros::spin(s);
}
