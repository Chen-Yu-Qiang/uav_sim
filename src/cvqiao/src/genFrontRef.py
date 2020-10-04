#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import rospy
import threading
import time
import numpy as np
m=-1
t=-1.0
def cb_updata_ref(data):
    global m
    m=float(data.data)
    print(m)

rospy.init_node('genLeaderRef', anonymous=True)
pub_ref = rospy.Publisher('/drone1/genRef', Twist, queue_size=1)
rospy.Subscriber('/ref_cmd', Int32, cb_updata_ref)
rate = rospy.Rate(10)


leader=Twist()
while not rospy.is_shutdown():
    if m==-1:
        continue
    elif m==0:
        if t==50:
            m=1
            t=0.0
        else:
            t=t+1
            leader.linear.x=4
            leader.linear.y=0
            leader.linear.z=1
            leader.angular.z=0
    elif m==1:
        if t==200:
            continue
            m=0
            t=0.0
        else:
            t=t+1
            theta=(t/200.0)*(2*np.pi)*0.25
            leader.linear.x=4+10*np.sin(theta)
            leader.linear.y=10*np.sin(theta)
            leader.linear.z=1
            leader.angular.z=theta
    elif m==2:
        if t==200:
            m=0
            t=0.0
        else:
            t=t+1
            leader.linear.x=4
            leader.linear.y=0
            leader.linear.z=1
            leader.angular.z=(t/100.0)*(2*np.pi)
    elif m==3:
        if t==50:
            m=0
            t=0.0
        else:
            t=t+1
            leader.linear.x=9-t/10
            leader.linear.y=5-t/10
            leader.linear.z=1
            leader.angular.z=0

    else:
        leader.linear.x=2
        leader.linear.y=0
        leader.linear.z=1
        leader.angular.z=np.pi
    pub_ref.publish(leader)
    
    
    rate.sleep()
