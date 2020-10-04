#!/usr/bin/env python
import roslib
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import time



if __name__ == '__main__':
    pub_emergency = rospy.Publisher('tello/emergency', Empty, queue_size=10)
    pub_land = rospy.Publisher('tello/land', Empty, queue_size=10)
    pub_takeoff = rospy.Publisher('tello/takeoff', Empty, queue_size=10)
    pub_ref_cmd = rospy.Publisher('ref_cmd', Int32, queue_size=10)
    rospy.init_node('controlUAV', anonymous=True)
    while not rospy.is_shutdown():
        incom=raw_input("input command\n t=takeoff \n l=land \n e=emergency\n $??")
        if incom=="e":
            msg=Empty()
            pub_emergency.publish(msg)
        elif incom=="t":
            msg=Empty()
            pub_takeoff.publish(msg)
            msg=Int32()
            msg=0
            pub_ref_cmd.publish(msg)
        elif incom=="l":
            msg=Empty()
            pub_land.publish(msg)
        elif incom=="1":
            msg=Int32()
            msg=1
            pub_ref_cmd.publish(msg)
        elif incom=="0":
            msg=Int32()
            msg=0
            pub_ref_cmd.publish(msg)
        elif incom=="exit":
            break
