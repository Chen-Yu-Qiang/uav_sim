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
            leader.linear.x=0
            leader.linear.y=0
            leader.linear.z=1
            leader.angular.z=-np.pi/4
            theta=-np.pi/4
    elif m==1:
        theta=theta+0.1*0.05*np.sqrt(np.cos(2*theta))
        if theta>np.pi/4:
            m=2
            theta=5*np.pi/4
        else:
            a=theta
            leader.linear.x=10*np.sqrt(np.cos(2*a))*np.cos(a)
            leader.linear.y=10*np.sqrt(np.cos(2*a))*np.sin(a)
            leader.linear.z=1
            leader.angular.z=np.arctan2(np.cos(3*a)/np.sqrt(np.cos(2*a)), -np.sin(3*a)/np.sqrt(np.cos(2*a)))
                
    elif m==2:
        theta=theta-0.1*0.05*np.sqrt(np.cos(2*theta))
        if theta<3*np.pi/4:
            m=3
        else:
            a=theta
            leader.linear.x=10*np.sqrt(np.cos(2.0*a))*np.cos(a)
            leader.linear.y=10*np.sqrt(np.cos(2.0*a))*np.sin(a)
            leader.linear.z=1
            leader.angular.z=np.arctan2(-np.cos(3*a)/np.sqrt(np.cos(2*a)), np.sin(3*a)/np.sqrt(np.cos(2*a)))
    elif m==3:
        if t==50:
            m=0
            t=0.0
        else:
            t=t+1
            leader.linear.x=0
            leader.linear.y=0
            leader.linear.z=1
            leader.angular.z=-np.pi/4

    else:
        leader.linear.x=2
        leader.linear.y=0
        leader.linear.z=1
        leader.angular.z=np.pi
    pub_ref.publish(leader)
    
    rate.sleep()
