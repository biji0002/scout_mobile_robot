#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

right_obstacle = 0
left_obstacle = 0
backward_right = 0 
backward_left = 0 

#going backward
def callback1(data):
	global right_obstacle, backward_right
	if (data.range<0.4 and data.range>0):
		right_obstacle = 1
		backward_right = 1

	elif (data.range>0.5 or data.range<0):
		right_obstacle = 0


def callback2(data):
	global left_obstacle, backward_left
    	if (data.range<0.4 and data.range>0):
		left_obstacle = 1
		backward_left = 1

    	elif (data.range>0.5 or data.range<0):
		left_obstacle = 0


def switcher_back(right_obstacle,left_obstacle):
	global backward_right, backward_left
	print(right_obstacle,left_obstacle,backward_right,backward_left)
	#global right_obstacle,left_obstacle,backward_right,backward_left
	if (right_obstacle or left_obstacle):
		pub.publish(back_msg)
		print('switch back')
	#elif ((right_obstacle and left_obstacle and backward_right and backward_left) == 1):
		#pub.publish(brake_msg)
		#print('switch brake')
	elif ((right_obstacle and left_obstacle and backward_left) == 0 and backward_right == 1):
		pub.publish(left_msg)
		print('turn left')
		rospy.sleep(3.0)
		backward_right =0
		pub.publish(brake_msg)
		print('turn left done')
	elif ((right_obstacle and left_obstacle) == 0 and backward_left == 1):
		pub.publish(right_msg)
		print('turn right')
		rospy.sleep(3.0)
		backward_left =0
		pub.publish(brake_msg)
		print('turn right done')
	#print('bibibobo')
def ultrasonic_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

	global back_msg, rate, pub, brake_msg, left_msg, right_msg

	pub = rospy.Publisher('backward_vel',Twist, queue_size=10)
	
	# send going back message
	back_msg = Twist()
	back_msg.linear.x = -0.3
	back_msg.linear.y = 0
	back_msg.linear.z = 0
	back_msg.angular.x = 0
	back_msg.angular.y = 0
	back_msg.angular.z = 0

	brake_msg = Twist()
	brake_msg.linear.x = 0
	brake_msg.linear.y = 0
	brake_msg.linear.z = 0
	brake_msg.angular.x = 0
	brake_msg.angular.y = 0
	brake_msg.angular.z = 0
	
	right_msg = Twist()
	right_msg.linear.x = 0
	right_msg.linear.y = 0
	right_msg.linear.z = 0
	right_msg.angular.x = 0
	right_msg.angular.y = 0
	right_msg.angular.z = -0.3

	left_msg = Twist()
	left_msg.linear.x = 0
	left_msg.linear.y = 0
	left_msg.linear.z = 0
	left_msg.angular.x = 0
	left_msg.angular.y = 0
	left_msg.angular.z = 0.3



	rospy.init_node('ultrasonic_side', anonymous=False)
	print('node initialized')
	rate = rospy.Rate(10)
	rospy.Subscriber("ultrasonic_range_2", Range, callback1) #front right
	rospy.Subscriber("ultrasonic_range_4", Range, callback2) #front left


	while not rospy.is_shutdown():
		switcher_back(right_obstacle,left_obstacle)
		rate.sleep()

	rospy.spin()

if __name__ == '__main__':
    try:
        ultrasonic_listener()
    except rospy.ROSInterruptException:
        pass
    
