#!/usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time
import rospy
import tf
rospy.init_node('int_imu', anonymous=True)
px = rospy.get_param('~px0')
py = rospy.get_param('~py0')
pz = rospy.get_param('~pz0')


imu_pub = rospy.Publisher("from_IMU", PoseStamped, queue_size=1)
imu_tf = tf.TransformBroadcaster()
last_time=None
def callback(data):
    global px, py, pz, last_time, imu_pub, imu_tf
    t=data.header.stamp.secs+data.header.stamp.nsecs*(10**-9)
    if last_time is None:
        last_time = t 
    dt = t-last_time
    last_time = t
    px = -data.twist.twist.angular.x*dt+px
    py = data.twist.twist.angular.y*dt+py
    pz = -data.twist.twist.angular.z*dt+pz
    q0 = data.pose.pose.orientation.w
    q1 = data.pose.pose.orientation.x
    q2 = data.pose.pose.orientation.y
    q3 = data.pose.pose.orientation.z
    pubmsg = PoseStamped()
    pubmsg.pose.position.x = px
    pubmsg.pose.position.y = py
    pubmsg.pose.position.z = pz
    pubmsg.pose.orientation.w = q0
    pubmsg.pose.orientation.x = q1
    pubmsg.pose.orientation.y = q2
    pubmsg.pose.orientation.z = q3
    pubmsg.header = data.header
    imu_pub.publish(pubmsg)
    imu_tf.sendTransform((px, py, pz), (q1, q2, q3, q0),
                         rospy.Time.now(), "Tello_IMU", "world")


imu_sub = rospy.Subscriber("tello/odom", Odometry, callback)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
