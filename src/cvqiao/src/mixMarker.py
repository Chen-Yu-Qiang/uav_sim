#!/usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import Marker
import time
import rospy
import tf
import numpy as np
rospy.init_node('mixMarker', anonymous=True)

N_marker = 20
marker_data = {}


last_t = None
MARKER_STD_TH = 0.2


def callback_Marker(data):
    global marker_data, mix_marker_pub, last_t
    t = (data.header.stamp.secs+data.header.stamp.nsecs*(10**-9))

    if not str(t) in marker_data.keys():
        marker_data[str(t)] = []
        if not last_t is None:
            x, y, z, th = getEachMean(last_t)
            if not (x, y, z, th) == (0, 0, 0, 0):
                outmsg=PoseStamped()
                #outmsg.header=marker_data[last_t][0].header
                outmsg.pose.position.x=x
                outmsg.pose.position.y=y
                outmsg.pose.position.z=z
                outmsg.pose.orientation.z=np.sin(th/2)
                outmsg.pose.orientation.w=np.cos(th/2)
                mix_marker_pub.publish(outmsg)
    last_t = str(t)
    marker_data[str(t)].append(data)




def getEachMean(t):
    global marker_data, MARKER_STD_TH, mix_marker_std_pub
    x = []
    y = []
    z = []
    th = []
    for i in marker_data[t]:
        x.append(i.pose.position.x)
        y.append(i.pose.position.y)
        z.append(i.pose.position.z)
        th_val=np.arctan2(i.pose.orientation.z,i.pose.orientation.w)*2
        while th_val<0:
            th_val=th_val+2*np.pi
        th.append(th_val)
    outmsg=PoseStamped()
    outmsg.pose.position.x=np.std(x)
    outmsg.pose.position.y=np.std(y)
    outmsg.pose.position.z=np.std(z)
    mix_marker_std_pub.publish(outmsg)
    if np.std(x) > MARKER_STD_TH or np.std(y) > MARKER_STD_TH or np.std(z) > MARKER_STD_TH:
        return 0, 0, 0, 0
    return np.mean(x), np.mean(y), np.mean(z), np.mean(th)



marker_sub = [rospy.Subscriber(
    "cam_from_marker_"+str(i), PoseStamped, callback_Marker) for i in range(N_marker)]

mix_marker_pub = rospy.Publisher("mixMarker", PoseStamped, queue_size=1)
mix_marker_std_pub = rospy.Publisher("mixMarkerSTD", PoseStamped, queue_size=1)
#imu_tf = tf.TransformBroadcaster()

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
