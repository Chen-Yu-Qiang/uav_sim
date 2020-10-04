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
def job():
    global m,waypoint
    time.sleep(5)
    while 0:
        if m==len(waypoint)-1:
            m=0
        else:
            m=m+1
        time.sleep(10)

rospy.init_node('genRef', anonymous=True)
pub_ref = rospy.Publisher('genRef', Twist, queue_size=1)
rospy.Subscriber('ref_cmd', Int32, cb_updata_ref)
rate = rospy.Rate(10)
#t = threading.Thread(target = job)
#t.start()

refmsg=Twist()
while not rospy.is_shutdown():
    if m==-1:
        continue
    elif m==0:
        if t==100:
            m=1
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=2
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==1:
        if t==100:
            m=2
            t=0.0
        else:
            t=t+1
            #refmsg.linear.x=3
            #refmsg.linear.y=0
            #refmsg.linear.z=1
            #refmsg.angular.z=np.pi
            refmsg.linear.x=np.cos(t/100*2*np.pi)*2
            refmsg.linear.y=np.sin(t/100*2*np.pi)*2
            refmsg.linear.z=1
            refmsg.angular.z=t/100*2*np.pi+np.pi
    elif m==2:
        if t==100:
            m=3
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=2
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==3:
        if t==80:
            m=4
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=t/10+2
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==4:
        if t==100:
            m=5
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=10
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==5:
        if t==80:
            m=6
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=-(t/10)+10
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==6:
        if t==50:
            m=14
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=2
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==7:
        if t==30:
            m=8
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=(t/10)**2+2
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==8:
        if t==50:
            m=9
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=11
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==9:
        if t==30:
            m=10
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=-(t/10)**2+11
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==10:
        if t==50:
            m=11
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=2
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==11:
        if t==100:
            m=12
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=2*np.sin(t/20*np.pi)+4
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==12:
        if t==50:
            m=13
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=1
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    elif m==13:
        if t==100:
            m=14
            t=0.0
        else:
            t=t+1
            refmsg.linear.x=2*np.sin(t/40*np.pi)+4
            refmsg.linear.y=0
            refmsg.linear.z=1
            refmsg.angular.z=np.pi
    else:
        refmsg.linear.x=2
        refmsg.linear.y=0
        refmsg.linear.z=1
        refmsg.angular.z=np.pi
    pub_ref.publish(refmsg)
    rate.sleep()

#t.join()