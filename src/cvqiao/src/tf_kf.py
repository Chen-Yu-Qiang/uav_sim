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

from gazebo_msgs.msg import ModelStates

def check2pi(a):
    while a>np.pi:
        a=a-2*np.pi
    while a<-np.pi:
        a=a+2*np.pi
    return a
last_th_1=1.57
last_th_2=1.57
last_r_1=0
last_r_2=0
def cb_gazebo_state(data):
    global pub_p_kf_list,msg_p_kf_list,last_th_1,last_th_2,last_r_1,last_r_2
    Tello_list=[1,2]
    # print(data)
    for i in range(len(data.name)):
        if data.name[i]=="drone1":
            out_msg=Twist()
            out_msg.linear.x=data.pose[i].position.x
            out_msg.linear.y=data.pose[i].position.y
            out_msg.linear.z=data.pose[i].position.z
            out_msg.angular.z=np.arctan2(data.pose[i].orientation.z,data.pose[i].orientation.w)*2-0.5*np.pi
            out_msg.angular.z=check2pi(out_msg.angular.z)
            if last_th_1-out_msg.angular.z>6:
                last_r_1=last_r_1+1
            if last_th_1-out_msg.angular.z<-6:
                last_r_1=last_r_1-1            
            last_th_1=out_msg.angular.z
            out_msg.angular.z=out_msg.angular.z+6.2831*last_r_1
            # pub_p_kf_list[0].publish(out_msg)
            msg_p_kf_list[0]=out_msg
            # print("1",out_msg)
        if data.name[i]=="drone2":
            out_msg=Twist()
            out_msg.linear.x=data.pose[i].position.x
            out_msg.linear.y=data.pose[i].position.y
            out_msg.linear.z=data.pose[i].position.z
            out_msg.angular.z=np.arctan2(data.pose[i].orientation.z,data.pose[i].orientation.w)*2-0.5*np.pi
            out_msg.angular.z=check2pi(out_msg.angular.z)

            if last_th_2-out_msg.angular.z>6:
                last_r_2=last_r_2+1
            if last_th_2-out_msg.angular.z<-6:
                last_r_2=last_r_2-1            
            last_th_2=out_msg.angular.z
            out_msg.angular.z=out_msg.angular.z+6.2831*last_r_2
            # pub_p_kf_list[1].publish(out_msg)
            msg_p_kf_list[1]=out_msg
            # print("2",out_msg)



rospy.init_node('tf_kf', anonymous=True)


Tello_list=[1,2]

pub_p_kf_list=[rospy.Publisher("/drone"+str(i)+"/from_kf", Twist, queue_size=1) for i in Tello_list]
msg_p_kf_list=[Twist() for i in Tello_list]
rospy.Subscriber("/gazebo/model_states", ModelStates, cb_gazebo_state)
rate=rospy.Rate(30)
time.sleep(2)
while not rospy.is_shutdown():
    for i in range(len(pub_p_kf_list)):
        if msg_p_kf_list[i].linear.x*msg_p_kf_list[i].linear.y*msg_p_kf_list[i].linear.z==0:
            continue
        pub_p_kf_list[i].publish(msg_p_kf_list[i])
    rate.sleep()
