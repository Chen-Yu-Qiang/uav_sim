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

rospy.init_node('genRef', anonymous=True)
pub_ref = [rospy.Publisher('/drone1/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone2/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone3/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone4/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone5/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone6/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone7/genRef', Twist, queue_size=1),
            rospy.Publisher('/drone8/genRef', Twist, queue_size=1)]
rospy.Subscriber('/ref_cmd', Int32, cb_updata_ref)
rate = rospy.Rate(10)


leader=Twist()
while not rospy.is_shutdown():
    if m==-1:
        continue
    elif m==0:
        if t==150:
            m=1
            t=0.0
        else:
            t=t+1
            leader.linear.x=3
            leader.linear.y=0
            leader.linear.z=1
            leader.angular.z=np.pi
    elif m==1:
        if t==150:
            m=0
            t=0.0
        else:
            t=t+1
            leader.linear.x=4
            leader.linear.y=0
            leader.linear.z=1
            leader.angular.z=np.pi
    elif m==2:
        if t==200:
            m=0
            t=0.0
        else:
            t=t+1
            leader.linear.x=5
            leader.linear.y=0
            leader.linear.z=1
            leader.angular.z=np.pi

    else:
        leader.linear.x=2
        leader.linear.y=0
        leader.linear.z=1
        leader.angular.z=np.pi
    
    
    
    if m==-1:
        continue
    elif m==0 or m==1:
        # for drone1
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x-1
        outmsg.linear.y=leader.linear.y
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z 
        pub_ref[0].publish(outmsg)

        # for drone2
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x
        outmsg.linear.y=leader.linear.y
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z 
        pub_ref[1].publish(outmsg)

        # for drone3
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x+1
        outmsg.linear.y=leader.linear.y
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z 
        pub_ref[2].publish(outmsg)


        # for drone4
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x
        outmsg.linear.y=leader.linear.y+1
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z 
        pub_ref[3].publish(outmsg)

        # for drone5
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x
        outmsg.linear.y=leader.linear.y-1
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z 
        pub_ref[4].publish(outmsg)

    elif m==2:
        # for drone1
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x+np.cos(t/100*2*np.pi+np.pi)*2
        outmsg.linear.y=leader.linear.y+np.sin(t/100*2*np.pi+np.pi)*2
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z+t/100*2*np.pi+np.pi
        pub_ref[0].publish(outmsg)

        # for drone2
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x
        outmsg.linear.y=leader.linear.y
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z 
        pub_ref[1].publish(outmsg)

        # for drone3
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x+np.cos(t/100*2*np.pi)*2
        outmsg.linear.y=leader.linear.y+np.sin(t/100*2*np.pi)*2
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z+t/100*2*np.pi
        pub_ref[2].publish(outmsg)


        # for drone4
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x+np.cos(t/100*2*np.pi+np.pi/2)*2
        outmsg.linear.y=leader.linear.y+np.sin(t/100*2*np.pi+np.pi/2)*2
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z+t/100*2*np.pi+np.pi/2
        pub_ref[3].publish(outmsg)

        # for drone5
        outmsg=Twist()
        outmsg.linear.x=leader.linear.x+np.cos(t/100*2*np.pi-np.pi/2)*2
        outmsg.linear.y=leader.linear.y+np.sin(t/100*2*np.pi-np.pi/2)*2
        outmsg.linear.z=leader.linear.z
        outmsg.angular.z=leader.angular.z+t/100*2*np.pi-np.pi/2
        pub_ref[4].publish(outmsg)


    rate.sleep()
