#!/usr/bin/env python
import rospy
import roslaunch
from std_srvs.srv import Empty

def launch(req):
	rospy.loginfo("starting..")
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/robot/catkin_ws/src/feetech_controls/launch/feetech_control_interface.launch"])
	launch.start()
	rospy.loginfo("started")
    	return true


def main():
	rospy.init_node('main_node')
	s = rospy.Service('/launch', Empty, launch)
	rospy.spin()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	
	
