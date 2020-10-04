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

rospy.init_node('genBackRef', anonymous=True)
pub_ref2 = rospy.Publisher('/drone2/genRef', Twist, queue_size=1)
pub_ref3 = rospy.Publisher('/drone3/genRef', Twist, queue_size=1)
pub_ref4 = rospy.Publisher('/drone4/genRef', Twist, queue_size=1)
pub_ref5 = rospy.Publisher('/drone5/genRef', Twist, queue_size=1)
rospy.Subscriber('/ref_cmd', Int32, cb_updata_ref)
rate = rospy.Rate(10)


RefBackR=Twist()
RefBackL=Twist()
while not rospy.is_shutdown():

    if m==-1:
        continue
    elif m==0:
        if t==50:
            m=0
            t=0.0
        else:
            t=t+1
            RefBackR.linear.x=1.5
            RefBackR.linear.y=0.5
            RefBackR.linear.z=0
            RefBackR.angular.z=np.pi
    else:
        RefBackR.linear.x=2
        RefBackR.linear.y=0
        RefBackR.linear.z=1
        RefBackR.angular.z=np.pi
    pub_ref2.publish(RefBackR)
    pub_ref4.publish(RefBackR)



    if m==-1:
        continue
    elif m==0:
        if t==50:
            m=0
            t=0.0
        else:
            t=t+1
            RefBackL.linear.x=1.5
            RefBackL.linear.y=-0.5
            RefBackL.linear.z=0
            RefBackL.angular.z=np.pi
    else:
        RefBackL.linear.x=2
        RefBackL.linear.y=0
        RefBackL.linear.z=1
        RefBackL.angular.z=np.pi
    pub_ref3.publish(RefBackL)
    pub_ref5.publish(RefBackL)
    
    rate.sleep()
