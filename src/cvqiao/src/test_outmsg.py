#!/usr/bin/env python
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
import time
import rospy

import numpy as np
rospy.init_node('inout_str', anonymous=True)
x_c=0
y_c=0
z_c=0
t_c=0
x_g=0
y_g=0
z_g=0
t_g=0
vx_g=0
vy_g=0
vz_g=0
vt_g=0
def callback_cmd(data):
    global x_c,y_c,z_c,t_c
    x_c=data.linear.x
    y_c=data.linear.y
    z_c=data.linear.z
    t_c=data.angular.z

def callback_gazebo(data):
    global x_g,y_g,z_g,t_g,vx_g,vy_g,vz_g,vt_g
    drone_num=0
    for i in data.name:
        if i=="drone1":
            x_g=data.pose[drone_num].position.x
            y_g=data.pose[drone_num].position.y
            z_g=data.pose[drone_num].position.z
            t_g=np.arctan2(data.pose[drone_num].orientation.z,data.pose[drone_num].orientation.w)*2 
            vx_g=data.twist[drone_num].linear.x
            vy_g=data.twist[drone_num].linear.y
            vz_g=data.twist[drone_num].linear.z
            vt_g=data.twist[drone_num].angular.z
            return
        else:
            drone_num=drone_num+1
    

rospy.Subscriber('/drone1/tello/cmd_vel', Twist, callback_cmd)
rospy.Subscriber('/gazebo/model_states', ModelStates, callback_gazebo)
out_pub = rospy.Publisher("inout_str", String, queue_size=1)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    outmsg="%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"%(time.time(),x_c,y_c,z_c,t_c,x_g,y_g,z_g,t_g,vx_g,vy_g,vz_g,vt_g)
    print(outmsg)
    out_pub.publish(outmsg)
    rate.sleep()