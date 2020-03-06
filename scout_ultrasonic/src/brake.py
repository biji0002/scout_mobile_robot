#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


#front emergency brake
def callback0(data):
    if (data.range<0.6 and data.range>0):
        pub.publish(brake_msg)
        print ('front_brake')
 
    #else:
	#print ('front:', data.range)

#back emergency brake
def callback2(data):
    if (data.range<0.3 and data.range>0):
        pub.publish(brake_msg)
        print ('back_brake')
 
    #else:
        #print ('back:', data.range)


def ultrasonic_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

	global pub, brake_msg, back_msg

	pub = rospy.Publisher('brake_vel',Twist, queue_size=10)

	# send emergenct brake message
	brake_msg = Twist()
	brake_msg.linear.x = 0
	brake_msg.linear.y = 0
	brake_msg.linear.z = 0
	brake_msg.angular.x =0
	brake_msg.angular.y =0
	brake_msg.angular.z =0

	rospy.init_node('ultrasonic_front_back', anonymous=False)
	print('node initialized')
	rospy.Subscriber("ultrasonic_range_0", Range, callback0) #front   
	rospy.Subscriber("ultrasonic_range_1", Range, callback0) #front
	rospy.Subscriber("ultrasonic_range_6", Range, callback2) #back
	rospy.Subscriber("ultrasonic_range_7", Range, callback2) #back


	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    try:
        ultrasonic_listener()
    except rospy.ROSInterruptException:
        pass
    
