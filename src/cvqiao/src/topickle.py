#!/usr/bin/env python
from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import cv2
from cv2 import aruco
import rospy
import sys
import numpy as np
import time
import roslib
import pickle
import threading
import signal
import Queue
roslib.load_manifest('cvqiao')
i=0
f=raw_input("path: ")
def callback(data):
    global i,f
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imwrite(str(f)+"/"+str(data.header.stamp.secs)+"."+str(data.header.stamp.nsecs)+".jpg",cv_image)
    cv2.imshow("show",cv_image)
    cv2.waitKey(1)
    print(i)
    i=i+1

rospy.init_node('onearuco', anonymous=True)
image_sub = rospy.Subscriber("/image_converter/output_video", Image, callback)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
