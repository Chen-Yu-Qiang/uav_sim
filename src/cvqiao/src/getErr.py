#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import rospy
import numpy as np
x=0
y=0
z=0
t=0
ref_x=0
ref_y=0
ref_z=0
ref_t=0

def cb_updata_ref(data):
    global ref_x,ref_y,ref_z,ref_t
    ref_x=data.linear.x
    ref_y=data.linear.y
    ref_z=data.linear.z
    ref_t=data.angular.z

def cb_updata_now(data):
    global x,y,z,t,pub_err
    x=data.pose.position.x
    y=data.pose.position.y
    z=data.pose.position.z
    t = np.arctan2(data.pose.orientation.z,data.pose.orientation.w)*2
    if t<0:
        t=t+np.pi*2
    errmsg=Twist()
    errmsg.linear.x=ref_x-x
    errmsg.linear.y=ref_y-y
    errmsg.linear.z=ref_z-z
    e1=ref_t+2*np.pi-t
    e2=ref_t-2*np.pi-t
    e3=ref_t-t
    if abs(e1)<=abs(e2) and abs(e1)<=abs(e3):
        errmsg.angular.z=e1
    elif abs(e2)<=abs(e1) and abs(e2)<=abs(e3):
        errmsg.angular.z=e2
    else:
        errmsg.angular.z=e3
    #print(e1*180/np.pi,e2*180/np.pi,e3*180/np.pi,errmsg.angular.z*180/np.pi)
    pub_err.publish(errmsg)

rospy.init_node('getErr', anonymous=True)
pub_err = rospy.Publisher('err', Twist, queue_size=10)
rospy.Subscriber('mixIMUMarker', PoseStamped, cb_updata_now)
rospy.Subscriber('genRef', Twist, cb_updata_ref)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
