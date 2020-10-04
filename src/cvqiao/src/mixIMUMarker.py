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

marker_mean = {"t": [], "x": [], "y": [], "z": []}
imu_data = {"t": [], "x": [], "y": [], "z": []}
last_t = None
TIMELONG = 2
last_IMU = None
px = rospy.get_param('~px0')
py = rospy.get_param('~py0')
pz = rospy.get_param('~pz0')

def callback_Marker(data):
    global marker_mean, last_marker
    t = (data.header.stamp.secs+data.header.stamp.nsecs*(10**-9))
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    marker_mean["t"].append(t)
    marker_mean["x"].append(x)
    marker_mean["y"].append(y)
    marker_mean["z"].append(z)
    save_len=100
    if len(marker_mean["x"])>save_len:
        marker_mean["x"]=marker_mean["x"][save_len*(-1):-1]
    if len(marker_mean["y"])>save_len:
        marker_mean["y"]=marker_mean["y"][save_len*(-1):-1]
    if len(marker_mean["z"])>save_len:
        marker_mean["z"]=marker_mean["z"][save_len*(-1):-1]
    if len(marker_mean["t"])>save_len:
        marker_mean["t"]=marker_mean["t"][save_len*(-1):-1]
    last_marker=data
def callback_IMU(data):
    global imu_data, last_IMU, marker_mean
    t = (data.header.stamp.secs+data.header.stamp.nsecs*(10**-9))
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    imu_data["t"].append(t)
    imu_data["x"].append(x)
    imu_data["y"].append(y)
    imu_data["z"].append(z)
    save_len=1000
    if len(imu_data["x"])>save_len:
        imu_data["x"]=imu_data["x"][save_len*(-1):-1]
    if len(imu_data["y"])>save_len:
        imu_data["y"]=imu_data["y"][save_len*(-1):-1]
    if len(imu_data["z"])>save_len:
        imu_data["z"]=imu_data["z"][save_len*(-1):-1]
    if len(imu_data["t"])>save_len:
        imu_data["t"]=imu_data["t"][save_len*(-1):-1]
    last_IMU = data
last_t=time.time()
imu_v_x=0
imu_v_y=0
imu_v_z=0

def callback_IMU_raw(data):
    global imu_v_x,imu_v_y,imu_v_z
    imu_v_x = -data.twist.twist.angular.x
    imu_v_y = data.twist.twist.angular.y
    imu_v_z = -data.twist.twist.angular.z
    


rospy.Subscriber("mixMarker", PoseStamped, callback_Marker)
rospy.Subscriber("from_IMU", PoseStamped, callback_IMU)
rospy.Subscriber("tello/odom", Odometry, callback_IMU_raw)
mix_pub = rospy.Publisher("mixIMUMarker", PoseStamped, queue_size=1)
mix_tf = tf.TransformBroadcaster()

rate = rospy.Rate(20)
filter_x_marker = 0
filter_y_marker = 0
filter_z_marker = 0
dif_x = 0
dif_y = 0
dif_z = 0
outmsg = PoseStamped()
outmsg.pose.position.x = px
outmsg.pose.position.y = py
outmsg.pose.position.z = pz
while not rospy.is_shutdown():
    imu_data_copy = copy.deepcopy(imu_data)
    marker_mean_copy = copy.deepcopy(marker_mean)
    last_IMU_copy = copy.deepcopy(last_IMU)
    x_imu_sel = []
    y_imu_sel = []
    z_imu_sel = []
    if len(marker_mean_copy["t"])>1:
        for i in range(len(imu_data_copy["t"])-1, 0, -1):
            if imu_data_copy["t"][i] > imu_data_copy["t"][-1]-TIMELONG:
                min_i = min(len(imu_data_copy["x"]), len(
                    imu_data_copy["y"]), len(imu_data_copy["z"]))
                if i >= min_i:
                    i = min_i-1
                x_imu_sel.append(imu_data_copy["x"][i])
                y_imu_sel.append(imu_data_copy["y"][i])
                z_imu_sel.append(imu_data_copy["z"][i])
            elif imu_data_copy["t"][i] > marker_mean_copy["t"][-1]:
                continue
            else:
                break
        x_Marker_sel = []
        y_Marker_sel = []
        z_Marker_sel = []
        for i in range(len(marker_mean_copy["t"])-1, 0, -1):
            #print("i",i,marker_mean_copy["t"][-1]-imu_data_copy["t"][-1])
            if marker_mean_copy["t"][i] > imu_data_copy["t"][-1]-TIMELONG :
                min_i = min(len(marker_mean_copy["x"]), len(
                    marker_mean_copy["y"]), len(marker_mean_copy["z"]))
                if i >= min_i:
                    i = min_i-1
                x_Marker_sel.append(marker_mean_copy["x"][i])
                y_Marker_sel.append(marker_mean_copy["y"][i])
                z_Marker_sel.append(marker_mean_copy["z"][i])
            else:
                break
        marker_N = len(x_Marker_sel)
        print(marker_N)
    elif len(imu_data_copy["t"])>0:
        for i in range(len(imu_data_copy["t"])-1, 0, -1):
            if imu_data_copy["t"][i] >= imu_data_copy["t"][-1]-TIMELONG:
                min_i = min(len(imu_data_copy["x"]), len(
                    imu_data_copy["y"]), len(imu_data_copy["z"]))
                if i >= min_i:
                    i = min_i-1
                x_imu_sel.append(imu_data_copy["x"][i])
                y_imu_sel.append(imu_data_copy["y"][i])
                z_imu_sel.append(imu_data_copy["z"][i])
            else:
                break
        marker_N=0
    imu_N = len(x_imu_sel)
    
    
    if imu_N == 0:
        last_t=time.time()
        continue
    elif marker_N < 5:
        #filter_x_imu = imu_data_copy["x"][-1]
        #filter_y_imu = imu_data_copy["y"][-1]
        #filter_z_imu = imu_data_copy["z"][-1]
        #outmsg.pose.position.x = filter_x_imu-dif_x
        #outmsg.pose.position.y = filter_y_imu-dif_y
        #outmsg.pose.position.z = filter_z_imu-dif_z
        dt=time.time()-last_t
        aaa=dt
        #outmsg.pose.position.x = outmsg.pose.position.x+imu_v_x*dt*0.5
        #outmsg.pose.position.y = outmsg.pose.position.y+imu_v_y*dt*0.5
        #outmsg.pose.position.z = outmsg.pose.position.z+imu_v_z*dt*0.5
        outmsg.pose.position.x = imu_data_copy["x"][-1]
        outmsg.pose.position.y = imu_data_copy["y"][-1]
        outmsg.pose.position.z = imu_data_copy["z"][-1]
        outmsg.header.frame_id="i"
    else:
        filter_x_marker = sum(x_Marker_sel)/marker_N
        filter_x_imu = imu_data_copy["x"][-1]-sum(x_imu_sel)/imu_N
        filter_y_marker = sum(y_Marker_sel)/marker_N
        filter_y_imu = imu_data_copy["y"][-1]-sum(y_imu_sel)/imu_N
        filter_z_marker = sum(z_Marker_sel)/marker_N
        filter_z_imu = imu_data_copy["z"][-1]-sum(z_imu_sel)/imu_N
        dif_x = sum(x_imu_sel)/imu_N - sum(x_Marker_sel)/marker_N
        dif_y = sum(y_imu_sel)/imu_N - sum(y_Marker_sel)/marker_N
        dif_z = sum(z_imu_sel)/imu_N - sum(z_Marker_sel)/marker_N
        outmsg.pose.position.x = filter_x_imu+filter_x_marker
        outmsg.pose.position.y = filter_y_imu+filter_y_marker
        outmsg.pose.position.z = filter_z_imu+filter_z_marker
        outmsg.header.frame_id="m"
        aaa=0
    last_t=time.time()
    outmsg.pose.orientation = last_IMU_copy.pose.orientation
    #outmsg.header = last_IMU_copy.header
    mix_pub.publish(outmsg)
    #print(imu_N, marker_N, aaa)
    mix_tf.sendTransform((outmsg.pose.position.x, outmsg.pose.position.y, outmsg.pose.position.z),
                         (outmsg.pose.orientation.w, outmsg.pose.orientation.x,
                          outmsg.pose.orientation.y, outmsg.pose.orientation.z),
                         rospy.Time.now(), "Mix_IMU_Marker", "world")
    rate.sleep()
