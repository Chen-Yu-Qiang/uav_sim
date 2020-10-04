#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import rospy
import time
import numpy as np


q0=1
q1=0
q2=0
q3=0
def callback_IMU(data):
    global q0,q1,q2,q3
    q0=data.orientation.w
    q1=data.orientation.x
    q2=data.orientation.y
    q3=data.orientation.z
#t   x+  x-  y+  y-
#====================
#0   x-  x+  y+  y-
#90  y-  y+  x-  x+
#180 x+  x-  y-  y+
#270 y+  y-  x+  x-
#x=+cos*(x)-sin*(y)
#y=-sin*(x)-cos*(y)
def callback_nav(data):
    global q0,q1,q2,q3,pub_odom
    outmsg=Odometry()
    outmsg.header=data.header
    theta = np.arctan2(q3,q0)*2
    if theta<0:
        theta=theta+np.pi*2
    outmsg.twist.twist.angular.x=(-np.cos(theta)*data.vx+np.sin(theta)*data.vy)/1000
    outmsg.twist.twist.angular.y=(np.sin(theta)*data.vx+np.cos(theta)*data.vy)/1000
    outmsg.twist.twist.angular.z=-data.vz/1000
    outmsg.pose.pose.orientation.w=q0
    outmsg.pose.pose.orientation.x=q1
    outmsg.pose.pose.orientation.y=q2
    outmsg.pose.pose.orientation.z=q3
    pub_odom.publish(outmsg)

def callback_cmd(data):
    global pub_cmd
    outmsg=Twist()
    outmsg.linear.x=data.linear.y
    outmsg.linear.y=-data.linear.x
    outmsg.linear.z=data.linear.z
    outmsg.angular.z=data.angular.z
    pub_cmd.publish(outmsg)

def callback_takeoff(data):
    global pub_takeoff
    pub_takeoff.publish(Empty())
def callback_land(data):
    global pub_land
    pub_land.publish(Empty())

rospy.init_node('tf_topic', anonymous=True)
pub_odom=rospy.Publisher("tello/odom", Odometry, queue_size=1)
rospy.Subscriber("ardrone/navdata", Navdata, callback_nav)
rospy.Subscriber("ardrone/imu", Imu, callback_IMU)
rospy.Subscriber("tello/cmd_vel", Twist, callback_cmd)


rospy.Subscriber("tello/takeoff", Empty, callback_takeoff)
pub_takeoff=rospy.Publisher("ardrone/takeoff", Empty, queue_size=1)
rospy.Subscriber("tello/land", Empty, callback_land)
pub_land=rospy.Publisher("ardrone/land", Empty, queue_size=1)
pub_cmd=rospy.Publisher("cmd_vel", Twist, queue_size=1)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
