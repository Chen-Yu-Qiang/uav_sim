#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
import rospy
import threading
import time
import numpy as np


def cb_updata_ref(data):
    global pub_ref
    leader=Twist()
    leader.linear.x=data.pose.position.x
    leader.linear.y=data.pose.position.y
    leader.linear.z=data.pose.position.z
    leader.angular.z =np.arctan2(data.pose.orientation.z,data.pose.orientation.w)*2
    while leader.angular.z<0:
        leader.angular.z=leader.angular.z+2*np.pi
    # for drone1
    outmsg=Twist()
    outmsg.linear.x=leader.linear.x-1
    outmsg.linear.y=leader.linear.y
    outmsg.linear.z=leader.linear.z
    outmsg.angular.z=leader.angular.z 
    pub_ref[0].publish(outmsg)

    # for drone3
    outmsg=Twist()
    outmsg.linear.x=leader.linear.x+1
    outmsg.linear.y=leader.linear.y
    outmsg.linear.z=leader.linear.z
    outmsg.angular.z=leader.angular.z 
    pub_ref[1].publish(outmsg)

    # for drone4
    outmsg=Twist()
    outmsg.linear.x=leader.linear.x
    outmsg.linear.y=leader.linear.y+1
    outmsg.linear.z=leader.linear.z
    outmsg.angular.z=leader.angular.z 
    pub_ref[2].publish(outmsg)


    # for drone5
    outmsg=Twist()
    outmsg.linear.x=leader.linear.x
    outmsg.linear.y=leader.linear.y-1
    outmsg.linear.z=leader.linear.z
    outmsg.angular.z=leader.angular.z 
    pub_ref[3].publish(outmsg)


rospy.init_node('genFollowerRef', anonymous=True)
pub_ref = [rospy.Publisher('/drone2/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone3/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone4/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone5/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone6/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone7/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone8/genRef', Twist, queue_size=1)]
rospy.Subscriber('/drone1/mixIMUMarker', PoseStamped, cb_updata_ref)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")

