#include <moveit/move_group_interface/move_group_interface.h>  
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include "ros/ros.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometry_msgs/PoseStamped.h"
#include <stdlib.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aruco_ros/aruco_id_pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"

class myClass{
    public:
    
    void armupdate();

    myClass() {
    sub = node_handle.subscribe("/aruco/getid", 1000, &myClass::getidcallback,this);
    client = node_handle.serviceClient<aruco_ros::aruco_id_pose>("/aruco_single/aruco_id_pose");
    pub = node_handle.advertise<std_msgs::Float32MultiArray>("/filterarea", 1);
    }
    
    protected:
    tf::TransformListener listener;
    aruco_ros::aruco_id_pose srv;
    ros::ServiceClient client;
    bool check=false;
    double newoldx =0;
    double newoldy =0;
    double newoldz =0;
    double newoldx2 =0;
    double newoldy2 =0;
    double newoldz2 =0;
    double setwaist =0;
    double objdistance;
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
    ros::Publisher pub;
    tf::StampedTransform transform;
    std_msgs::Float32MultiArray array;

geometry_msgs::Pose offsetconvert(geometry_msgs::Pose inputpose, double x, double y,double z )
{
  tf::StampedTransform offset;
  offset.setIdentity();
  offset.setOrigin(tf::Vector3(x, y, z));
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(inputpose.position.x,inputpose.position.y,inputpose.position.z));
  transform.setRotation(tf::Quaternion(inputpose.orientation.x,inputpose.orientation.y,inputpose.orientation.z,inputpose.orientation.w));
  transform= transform * static_cast<tf::Transform>(offset);
  geometry_msgs::Pose poseMsg;
  tf::poseTFToMsg(transform, poseMsg);
  return poseMsg;
}



void getidcallback(const std_msgs::Int16::ConstPtr& msg)
{

   srv.request.id =msg->data;
if (client.call(srv))
  {
    ROS_INFO("received");
  }
  else
  {
    ROS_ERROR("Failed");
  }

array.data.clear();
array.data.push_back(1.0);
array.data.push_back(srv.response.fx);
array.data.push_back(srv.response.fy);
array.data.push_back(srv.response.fz);
array.data.push_back(srv.response.ofx);
array.data.push_back(srv.response.ofy);
array.data.push_back(srv.response.ofz);
pub.publish(array);

     retpoint.x=srv.response.px;
     retpoint.y=srv.response.py;
     retpoint.z=srv.response.pz;
     recepostconvert.orientation.x=srv.response.ox;
     recepostconvert.orientation.y=srv.response.oy;
     recepostconvert.orientation.z=srv.response.oz;
     recepostconvert.orientation.w=srv.response.ow;
     tf2::Vector3 xvector(0,-1,0);
     tf2::Vector3 newvector(abs(srv.response.px),srv.response.py,0);
     double waistangle= newvector.angle(xvector);
     double newy =0;

objdistance = sqrt((abs(srv.response.py)*abs(srv.response.py))+(abs(srv.response.px)*abs(srv.response.px)));

ROS_INFO("y-> [%f]", srv.response.py);


if (objdistance > 0.25)
{
newy=(objdistance)-0.25;
}
double offsetangle;

if (newy <0)
{
offsetangle = 0;
}
else if (newy>0.10035 && srv.response.ox<=0)
{
offsetangle =1.57;
}
else if(newy<0.10035 && srv.response.ox<=0)
{
offsetangle = 1.57-acos(newy/0.10035);//mid to arm base 100.35mm
}
else if(newy>0.10035 && srv.response.ox>0)
{
offsetangle =1.57;
}
else if(newy<0.10035 && srv.response.ox>0)
{
offsetangle = 1.57+acos(newy/0.10035);//mid to arm base 100.35mm
}
if(srv.response.px<0)
{
//if (waistangle>0) waistangle =0;
setwaist = waistangle-offsetangle;
ROS_INFO("setangle1-> [%f] objdistance-> [%f], offsetangle-> [%f]", waistangle,objdistance, offsetangle);
}
else
{
setwaist = -waistangle-offsetangle;
ROS_INFO("setangle2-> [%f] objdistance-> [%f]offsetangle-> [%f]", waistangle,objdistance, offsetangle);
}
  target_pose1.orientation=recepostconvert.orientation;
  target_pose1.position = retpoint;
  check = true;

}

};

void myClass::armupdate()
{

static const std::string PLANNING_GROUP = "feetech_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

moveit::core::RobotStatePtr current_state1 = move_group.getCurrentState();

current_state1->copyJointGroupPositions(joint_model_group, joint_group_positions1);

static const std::string PLANNING_GROUP2 = "feetech_waist";
     moveit::planning_interface::MoveGroupInterface move_group2(PLANNING_GROUP2);

const robot_state::JointModelGroup* joint_model_group2 = move_group2.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);

moveit::core::RobotStatePtr current_state2 = move_group2.getCurrentState();

current_state2->copyJointGroupPositions(joint_model_group2, joint_group_positions2);

static const std::string PLANNING_GROUP3 = "feetech_head";
     moveit::planning_interface::MoveGroupInterface move_group3(PLANNING_GROUP3);

const robot_state::JointModelGroup* joint_model_group3 = move_group3.getCurrentState()->getJointModelGroup(PLANNING_GROUP3);

moveit::core::RobotStatePtr current_state3 = move_group3.getCurrentState();

current_state3->copyJointGroupPositions(joint_model_group3, joint_group_positions3);

moveit::planning_interface::MoveGroupInterface::Plan my_plan3;

/*joint_group_positions3[0] = 0;  // radians
joint_group_positions3[1] = 0;  // radians

move_group3.setJointValueTarget(joint_group_positions3);
bool success3;
success3 = (move_group3.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
move_group3.setMaxVelocityScalingFactor(1.0);
ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success3 ? "" : "FAILED");
 move_group3.move();*/

move_group.setMaxVelocityScalingFactor(1.0);
move_group.setMaxAccelerationScalingFactor(1.0);
move_group2.setMaxVelocityScalingFactor(1.0);
move_group2.setMaxAccelerationScalingFactor(1.0);

ros::Rate r(10);

bool success2;
bool success;

while(node_handle.ok()){
ros::spinOnce();
if(check){
ROS_INFO("set");
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

geometry_msgs::Pose currentpose = move_group.getCurrentPose().pose;
ROS_INFO("%f,%f,%f",currentpose.position.x,currentpose.position.y,currentpose.position.z);
double offsetposevalue = 0.05;
double offsetposevaluexmin =-0.342-offsetposevalue;
double offsetposevaluexmax =-0.342+offsetposevalue;
double offsetposevalueymin =-0.004-offsetposevalue;
double offsetposevalueymax =-0.004+offsetposevalue;
double offsetposevaluezmin =0.181-offsetposevalue;
double offsetposevaluezmax =0.181+offsetposevalue;
if(currentpose.position.x < offsetposevaluexmin || currentpose.position.x >  offsetposevaluexmax)
{
joint_group_positions1[0] = 0;  // radians
joint_group_positions1[1] = 1.57;  // radians
joint_group_positions1[2] = 0;  // radians
joint_group_positions1[3] = -1.55;  // radians
joint_group_positions1[4] = 0;  // radians
joint_group_positions1[5] = 1.55;  // radians
joint_group_positions1[6] = 0;  // radians

move_group.setJointValueTarget(joint_group_positions1);
success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
move_group.move();
}

std::vector<double> joints2;
joints2 = move_group2.getCurrentJointValues();
ROS_INFO("%f",joints2[0]);

if(joints2[0]> 0.01 || joints2[0]<-0.01)
{
joint_group_positions2[0] = 0;  // radians
move_group2.setJointValueTarget(joint_group_positions2);
success2 = (move_group2.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success2 ? "" : "FAILED");
move_group2.move();
}

if(objdistance <0.55)
{

joint_group_positions2[0] = setwaist;  // radians
move_group2.setJointValueTarget(joint_group_positions2); 
success2 = (move_group2.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success2 ? "" : "FAILED");
move_group2.move();
if (success2)
{
success2= false; 
move_group.setStartStateToCurrentState();
move_group.setPoseTarget(target_pose1);
success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (pose goal) %s", success ? "" : "FAILED");
if(success)
{
move_group.move();
}

}

array.data.clear();
array.data.push_back(0);
pub.publish(array);

//go front

if(success)
{
success= false;
move_group.setStartStateToCurrentState();
std::vector<geometry_msgs::Pose> waypoints;
waypoints.clear();

waypoints.push_back(target_pose1);
geometry_msgs::Pose poseMsgret=offsetconvert(target_pose1, 0,0,-0.08);
geometry_msgs::Pose target_pose3 = target_pose1;
target_pose3.position = poseMsgret.position;
waypoints.push_back(target_pose3);  

geometry_msgs::Pose poseMsgret2=offsetconvert(target_pose3, 0,0.1,0);
waypoints.push_back(poseMsgret2);  
double newoffset =0;

//if(abs(target_pose1.position.y) >0.4)
//{
//newoffset = objdistance-0.4;
//}
//else
//{
//newoffset =0.125;
//}
//if (abs(target_pose1.position.y)>0.25) newoffset =0.15;

//geometry_msgs::Pose poseMsgret3=offsetconvert(poseMsgret2, 0,0,newoffset);

poseMsgret2.position.y += 0.15;

waypoints.push_back(poseMsgret2); 

moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

 // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "feetech_arm");

  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
 
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  
  // Fourth compute computeTimeStamps
  success = iptp.computeTimeStamps(rt, 0.20, 1);
  
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory);

  // Finally plan and execute the trajectory
  my_plan.trajectory_ = trajectory;
  ROS_INFO("Visualizing plan 5 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
  move_group.execute(my_plan);
//-----------------------------------------------------------------------------------------------------

joint_group_positions2[0] = 0;  // radians
move_group2.setJointValueTarget(joint_group_positions2);
success2 = (move_group2.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal) %s", success ? "" : "FAILED");
move_group2.move();
//--------------------------------------------------------------------------------------------

move_group.clearPathConstraints();
move_group.setStartStateToCurrentState();
std::vector<geometry_msgs::Pose> waypoints2;
geometry_msgs::Pose target_pose_last;
geometry_msgs::Pose target_pose_last2;
waypoints2.clear();
/*target_pose_last2.orientation.x=0.499;
target_pose_last2.orientation.y=0.507;
target_pose_last2.orientation.z=0.499;
target_pose_last2.orientation.w=0.494;
target_pose_last2.position.x =-0.554;
target_pose_last2.position.y =-0.004;
target_pose_last2.position.z =0.354;
waypoints2.push_back(target_pose_last2);*/

target_pose_last2.orientation.x=0.485;
target_pose_last2.orientation.y=0.479;
target_pose_last2.orientation.z=0.509;
target_pose_last2.orientation.w=0.526;
target_pose_last2.position.x =-0.259;
target_pose_last2.position.y =-0.003;
target_pose_last2.position.z =0.181;
waypoints2.push_back(target_pose_last2);
moveit_msgs::RobotTrajectory trajectory2;

  //const double jump_threshold = 0.0;
  //const double eef_step = 0.01;
  double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);

  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt2(move_group.getCurrentState()->getRobotModel(), "feetech_arm");

  // Second get a RobotTrajectory from trajectory
  rt2.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory2);
 
  // Thrid create a IterativeParabolicTimeParameterization object
 trajectory_processing::IterativeParabolicTimeParameterization iptp2;
  
  // Fourth compute computeTimeStamps
  success = iptp2.computeTimeStamps(rt2, 0.5, 1);
  
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  // Get RobotTrajectory_msg from RobotTrajectory
  rt2.getRobotTrajectoryMsg(trajectory2);

  // Finally plan and execute the trajectory
  my_plan.trajectory_ = trajectory2;
  ROS_INFO("Visualizing plan 7 (cartesian path) (%.2f%% acheived)",fraction2 * 100.0);   
  move_group.execute(my_plan);

if(fraction2*100.0 <50)
{
moveit::core::RobotStatePtr current_statelast = move_group.getCurrentState();
current_statelast->copyJointGroupPositions(joint_model_group, joint_group_positions1);
joint_group_positions1[2] = 0;  // radians
joint_group_positions1[0] = 0;  // radians
//move_group.setMaxVelocityScalingFactor(1);
move_group.setJointValueTarget(joint_group_positions1);
success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 8 (pose goal) %s", success ? "" : "FAILED");
move_group.move();
move_group.clearPathConstraints();
move_group.setStartStateToCurrentState();
double fraction3 = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);
robot_trajectory::RobotTrajectory rt3(move_group.getCurrentState()->getRobotModel(), "feetech_arm");
rt3.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory2);
success = iptp2.computeTimeStamps(rt3, 0.35, 1);
ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
rt3.getRobotTrajectoryMsg(trajectory2);
my_plan.trajectory_ = trajectory2;
  ROS_INFO("Visualizing plan 9 (cartesian path) (%.2f%% acheived)",fraction3 * 100.0);  
move_group.execute(my_plan);
}
//------------------------------------------------------------------------------------
}
}
else
{
ROS_INFO("Object too far !!");
}
 check =false;
}
r.sleep();
}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
    ros::AsyncSpinner spinner(2);
    spinner.start();
  myClass myObject;
  myObject.armupdate();
}
