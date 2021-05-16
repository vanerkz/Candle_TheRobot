#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <string>  
#include <iostream>  
#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"

class myClass{ 
    public:
    void odomupdate();

    myClass() {
    pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    sub = n.subscribe("dynamixel_workbench/dynamixel_state", 100,&myClass::chatterCallback,this);
    }

    protected:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle n;
    ros::Time current_time, last_time;
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    double accel_x =0.0;
    double vx_old =0.0;
    double vx_new =0.0;
    double accel_th =0.0;
    double th_old =0.0;
    double th_new =0.0;
    std::string zs ="";
    std::string zs1 ="";

    double Leftw;//dynamixelreading*constant rpm step*rpmtorad/s
    double Rightw;


    void chatterCallback(const dynamixel_workbench_msgs::DynamixelStateList ::ConstPtr &vel){
    dynamixel_workbench_msgs::DynamixelStateList new_vel = *vel;

    double Leftrpm =new_vel.dynamixel_state[0].present_velocity;
    if (Leftrpm >1023) Leftrpm=(-1)*(Leftrpm -1023);
    double rightrpm =-new_vel.dynamixel_state[1].present_velocity;
    if (rightrpm < -1023) rightrpm=(-1)*(rightrpm+1023);
  
    //zs1 = std:: to_string(Leftrpm);
    //ROS_INFO(zs1.c_str());
    //zs1 = std:: to_string(rightrpm);
    //ROS_INFO(zs1.c_str());

    Leftw =Leftrpm*0.11443*0.10471975511 ;//dynamixelreading*constant rpm step*rpmtorad/s
    Rightw =rightrpm*0.11443*0.10471975511 ;
    ROS_INFO("Left wheel: %f rad/s ",Leftw);
    ROS_INFO("Right wheel: %f rad/s ",Rightw);
    
  }
};

void myClass::odomupdate(){

    tf::TransformBroadcaster odom_broadcaster;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(10);

    while(n.ok()){
    ros::spinOnce();               // check for incoming messages
    //double vy = 1;
    double v = 0.033*((Rightw+Leftw)/2) ;
    ROS_INFO("forward speed: %f m/s",v);
    vx_new = v;

    double vth = 0.033*((Rightw - Leftw)/ 0.310); // 0.31 -> wheel separate
    ROS_INFO("Th speed: %f rad/s",vth);
    th_new = vth;

    current_time = ros::Time::now();
    
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    
    if(vx_new > vx_old){
    accel_x = (vx_new - vx_old)/dt;
    ROS_INFO("x Accel: %f m/s^2", accel_x);
    }
    vx_old = vx_new;

    accel_th = (th_new - th_old)/dt;
    ROS_INFO("th Accel: %f rad/s^2",accel_th);
    th_old = th_new;

    //double delta_x = (vx * cos(th) - vy * sin(th)) * dt; due to vy = 0
    //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_x = (v * cos(th)) * dt;
    double delta_y = (v * sin(th)) * dt;
    double delta_th = vth * dt;
 
    x += delta_x;
    y += delta_y;
    th += delta_th;

    
    //since all odometry is 6DOF we'll need a quaternion created from yaw
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
  
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
  
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
  
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
  
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v * cos(th);
    odom.twist.twist.linear.y = v * sin(th);
    odom.twist.twist.angular.z = vth;
    //publish the message
    pub.publish(odom);
    last_time = current_time;
    r.sleep();
    }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "odomnode");
  myClass myObject;
  myObject.odomupdate();
  ros::MultiThreadedSpinner spinner(0); // Use 4 threads
  spinner.spin();
}
