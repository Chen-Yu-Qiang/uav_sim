#!/usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import time
import rospy
import tf
import numpy as np
import message_filters
import copy
rospy.init_node('mix', anonymous=True)

d=[PoseStamped() for i in range(5)]
def callback_Marker(data):
    global marker_mean, last_marker, mix_pub, d
    d[4]=d[3]
    d[3]=d[2]
    d[2]=d[1]
    d[1]=d[0]
    d[0]=data
    x=0
    y=0
    z=0
    qx=0
    qy=0
    qz=0
    qw=0
    for i in range(5):
        x=x+d[i].pose.position.x
        y=y+d[i].pose.position.y
        z=z+d[i].pose.position.z
        qx=qx+d[i].pose.orientation.x
        qy=qy+d[i].pose.orientation.y
        qz=qz+d[i].pose.orientation.z
        qw=qw+d[i].pose.orientation.w
    outmsg=PoseStamped()
    outmsg.pose.position.x=x/5
    outmsg.pose.position.y=y/5
    outmsg.pose.position.z=z/5
    outmsg.pose.orientation.x=qx/5
    outmsg.pose.orientation.y=qy/5
    outmsg.pose.orientation.z=qz/5
    outmsg.pose.orientation.w=qw/5
    if abs(outmsg.pose.position.x)>100 or abs(outmsg.pose.position.y)>100 or abs(outmsg.pose.position.z)>100:
        return
    mix_pub.publish(outmsg)



rospy.Subscriber("mixMarker", PoseStamped, callback_Marker)
mix_pub = rospy.Publisher("mixIMUMarker", PoseStamped, queue_size=1)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")

