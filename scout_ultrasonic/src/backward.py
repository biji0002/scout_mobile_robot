#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


#going backward
def callback1(data):
    if (data.range<0.5 and data.range>0):
	#for i in range (10):
		#pub.publish(back_msg)
		#
		#rate.sleep()
	print ('back2')	
	pub.publish(back_msg)
	
    #else:
	#pub.publish(brake_msg)


def callback2(data):
    if (data.range<0.5 and data.range>0):
        pub.publish(back_msg)
        print ('back4')
 
    #else:
	#print('range4:',data.range)



def ultrasonic_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

	global back_msg, rate, pub, brake_msg
	
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

	rospy.init_node('ultrasonic_side', anonymous=False)
	print('node initialized')	
	rate = rospy.Rate(100)
	rospy.Subscriber("ultrasonic_range_2", Range, callback1) #front right
	rospy.Subscriber("ultrasonic_range_4", Range, callback2) #front left

	

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    try:
        ultrasonic_listener()
    except rospy.ROSInterruptException:
        pass
    
