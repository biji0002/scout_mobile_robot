#!/usr/bin/env python
import rospy
from scout_msgs.msg import ScoutLightCmd

def send_light_cmd():
    pub = rospy.Publisher('/scout_light_control', ScoutLightCmd, queue_size=5)
    rospy.init_node('scout_light_commander', anonymous=True)
    r = rospy.Rate(1) #10hz
    msg = ScoutLightCmd()
    
    msg.front_mode = msg.LIGHT_CONST_ON
    msg.rear_mode = msg.LIGHT_CONST_ON

    while not rospy.is_shutdown():
        # rospy.loginfo(msg)
        pub.publish(msg)
        print("command sent")
        r.sleep()

if __name__ == '__main__':
    try:
        send_light_cmd()
    except rospy.ROSInterruptException: pass