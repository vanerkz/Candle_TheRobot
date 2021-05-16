#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string>  
#include <iostream>  
#include <stdio.h>
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "aruco_ros/aruco_id_pose.h"
#include "feetech_controls/headcontrols.h"
#include "std_msgs/Bool.h"
#include "move_base_msgs/MoveBaseAction.h"

class myClass{ 
    public:
    void getidcallback();
    void repubarucocheck();
    void setrecheckaruco();

    myClass() {
    sub = n.subscribe("/aruco/getid", 1, &myClass::getidcallback,this);
    sub2 = n.subscribe("/move_base/result",1,&myClass::repubarucocheck,this);
    sub3 = n.subscribe("/setrepubaruco",1,&myClass::setrecheckaruco,this);
    pub = n.advertise<feetech_controls::headcontrols>("/aruco/getid", 10);
    pub2 = n.advertise<std_msgs::Bool>("/navicond", 10);
    }

    protected:
    ros::Subscriber sub, sub2, sub3;
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::NodeHandle n;
    int state =0;
    double head1, head2;
    int markerid;
    bool condreplan =false;
    feetech_controls:: headcontrols arucofind;
    std_msgs::Bool pub2con;
    
void setrecheckaruco(const std_msgs::Bool::ConstPtr& msg)
{
 condreplan = msg->data;
}

void getidcallback(const feetech_controls::headcontrols msg)
{
arucofind.headjointvalues1 = msg.headjointvalues1;
arucofind.headjointvalues2 = msg.headjointvalues2;
arucofind.arucoid =msg.arucoid;
arucofind.type =msg.type;
}

void repubarucocheck(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
 
 if(!condreplan)
 {
   if(msg->status.status == 3)
   {
   pub2con.data =true;
   pub2.publish(pub2con);
   }
   else
   {
   pub2con.data =false;
   pub2.publish(pub2con);
   }
 }
 else if(condreplan)
 {
 if(msg->status.status == 3)
   {
   pub.publish(arucofind);
   condreplan=false;
   }
   else
   {
   pub2con.data =false;
   pub2.publish(pub2con);
   }
 }
}

};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "navicontroltopic");
  myClass myObject;
  ros::spin();
}
