#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def cbfun(data):
    x=data.twist.twist.angular.x
    y=data.twist.twist.angular.y
    z=data.twist.twist.angular.z
    ts=
    print(str(x)+","+str(y)+","+str(z))


rospy.init_node('readdata', anonymous=True)
sub = rospy.Subscriber("/tello/odom", Odometry, cbfun)

rospy.spin()