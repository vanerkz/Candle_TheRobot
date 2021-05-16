#!/usr/bin/env python
import rospy

import roslaunch
def start_task();
	
	package = 'main_run'
	executable = 'script_1.py'
	node = roslaunch.core.Node(package, executable)

	launch =roslaunch.scriptapi.ROSLaunch()
	launch.start()

	task = launch.launch(node)
	print task.is_alive()

def main():
	rospy.init_node('main_node')
	start_task()
	rospy.spin()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	

	
