#!/usr/bin/env python

import rospy
import random
from assignment1.srv import Target, TargetResponse

def random_target(req):
	"""	
	This function generates two random corrdinates (x,y) between a 
	minimum and a maximum value (min,max)
	"""

	#target response (robot_target_x, robot_target_y)
	res = TargetResponse()
	
	#coordinates of the target position chosen randomly
	res.robot_target_x = random.uniform(req.min, req.max)
	res.robot_target_y = random.uniform(req.min, req.max)
	
	rospy.loginfo("Target position [%.2f,%.2f]",res.robot_target_x,res.robot_target_y)
	
	return res
	
def main():
	"""	
	Main function that init a node called target_server and create
	a service called robot_target
	"""
	
	rospy.init_node('target_server') 
	
	#service that uses random_target
	serv = rospy.Service('robot_target',Target,random_target)
	
	rospy.spin()
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
