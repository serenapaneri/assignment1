#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from assignment1.srv import Target

#ROS publisher created as global to modify the velocity inside the callBack function
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

#ROS client service generating a new target for the robot
random_target_client = rospy.ServiceProxy('robot_target',Target)

#random coordinates of the new target (x,y)
robot_target = Odometry()

def CallBack(robot_position):
	"""
	The CallBack function reads the actual position of the robot and
	checks if the target has been reached from the robot.
	When the robot reaches the target, a new target is generated, and
	also the velocity of the robot is modified.
	Instead if the target hasn't been achieved by the robot the velocity
	of the robot is changed taking into account the distance of the 
	target with respect the position of the robot.

	"""

	x = robot_position.pose.pose.position.x # x coordinate of robot
	y = robot_position.pose.pose.position.y # y coordinate of robot
	

	#if the target is reached a new random target position is generated
	if(isReached(x,y)):
		new = random_target_client(-6.0,6.0)
		robot_target.pose.pose.position.x = new.robot_target_x
		robot_target.pose.pose.position.y = new.robot_target_y
	
	#velocity of the robot based on the distance of the 
	#target with respect to the robot position
	velocity = Twist() #the velocity of the robot is given by Twist
	velocity.linear.x = (robot_target.pose.pose.position.x - x)
	velocity.linear.y = (robot_target.pose.pose.position.y - y)
		
	pub.publish(velocity)
		
#function that checks if the position is reached 
def isReached(x,y):
	"""
	This function is created with the aim of checking if the target
	has been reached from the robot with a maximum error of 0.1
	
	"""
	
	#take into account a tollerance of +-0.1 on the position 
	#with respect x and y 
	if(x >= robot_target.pose.pose.position.x - 0.1 and x <= robot_target.pose.pose.position.x + 0.1):
		if(y >= robot_target.pose.pose.position.y - 0.1 and y <= robot_target.pose.pose.position.y + 0.1):
			rospy.loginfo("The target is reached\n")
			return True #the target is reached
	
	return False #the target is not reached
	
def control():
	"""
	This is the main function that generates a node called control_robot.
	It generates a new random target in the 2D environment and creates 
	a subscriber to know the actual position of the robot.
	"""
	
	rospy.init_node('control_robot')

	new = random_target_client(-6.0,6.0)
	robot_target.pose.pose.position.x = new.robot_target_x
	robot_target.pose.pose.position.y = new.robot_target_y
	rospy.loginfo("New target: [%.2f,%.2f]",robot_target.pose.pose.position.x,robot_target.pose.pose.position.y)
    
	rospy.Subscriber("/odom", Odometry,CallBack)
    
	rospy.spin()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
