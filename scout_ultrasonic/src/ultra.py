#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

def callback0(data):
    if (data.range<0.6 and data.range>0):
        pub.publish(vel_msg)
        print ('emergency brake')
 
    else:
        print ('Range0:', data.range)

def callback1(data):
    if (data.range<0.6 and data.range>0):
        pub.publish(vel_msg)
        print ('emergency brake')
 
    else:
        print ('Range1:', data.range)

def ultrasonic_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    global pub, vel_msg

    pub = rospy.Publisher('ultra_vel',Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x =0
    vel_msg.angular.y =0
    vel_msg.angular.z =0



    rospy.init_node('ultrasonic_listener', anonymous=False)
    rospy.Subscriber("ultrasonic_range_0", Range, callback0)    
    rospy.Subscriber("ultrasonic_range_1", Range, callback1)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        ultrasonic_listener()
    except rospy.ROSInterruptException:
        pass
    
