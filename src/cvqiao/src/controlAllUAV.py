#!/usr/bin/env python
import roslib
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import time



if __name__ == '__main__':
    pub_emergency = rospy.Publisher('tello/emergency', Empty, queue_size=1)
    pub_land = [rospy.Publisher('/drone1/tello/land', Empty, queue_size=1),
                rospy.Publisher('/drone2/tello/land', Empty, queue_size=1),
                rospy.Publisher('/drone3/tello/land', Empty, queue_size=1),
                rospy.Publisher('/drone4/tello/land', Empty, queue_size=1),
                rospy.Publisher('/drone5/tello/land', Empty, queue_size=1),
                rospy.Publisher('/drone6/tello/land', Empty, queue_size=1),
                rospy.Publisher('/drone7/tello/land', Empty, queue_size=1),
                rospy.Publisher('/drone8/tello/land', Empty, queue_size=1)]
    pub_takeoff = [rospy.Publisher('/drone1/tello/takeoff', Empty, queue_size=1),
                    rospy.Publisher('/drone2/tello/takeoff', Empty, queue_size=1),
                    rospy.Publisher('/drone3/tello/takeoff', Empty, queue_size=1),
                    rospy.Publisher('/drone4/tello/takeoff', Empty, queue_size=1),
                    rospy.Publisher('/drone5/tello/takeoff', Empty, queue_size=1),
                    rospy.Publisher('/drone6/tello/takeoff', Empty, queue_size=1),
                    rospy.Publisher('/drone7/tello/takeoff', Empty, queue_size=1),
                    rospy.Publisher('/drone8/tello/takeoff', Empty, queue_size=1)]
    pub_ref_cmd = rospy.Publisher('ref_cmd', Int32, queue_size=1)
    rospy.init_node('controlUAV', anonymous=True)
    while not rospy.is_shutdown():
        incom=raw_input("input command\n t=takeoff \n l=land \n e=emergency\n $??")
        if incom=="e":
            msg=Empty()
            pub_emergency.publish(msg)
        elif incom=="t":
            msg=Empty()
            for i in pub_takeoff:
                i.publish(msg)
            msg=Int32()
            msg=0

            pub_ref_cmd.publish(msg)
        elif incom=="l":
            msg=Empty()
            for i in pub_land:
                i.publish(msg)

        elif incom=="exit":
            break
