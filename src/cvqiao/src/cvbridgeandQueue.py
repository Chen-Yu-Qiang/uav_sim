#!/usr/bin/env python
from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import cv2
import rospy
import sys
import numpy as np
import time
import roslib

import threading,signal
import Queue
roslib.load_manifest('cvqiao')

ISEXIT=0
def find_inner_pam():
    cbraw = 10
    cbcol = 6
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((cbraw*cbcol, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cbraw, 0:cbcol].T.reshape(-1, 2)
    objp = objp*20
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = ["60.png",
              "80.png",
              "100.png",
              "120.png",
              "140.png"]
    for fname in images:
        print(fname)
        # source image
        img = cv2.imread("/home/yuqiang/catkin_ws/src/cvqiao/src/"+fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (10, 6), None)
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners2)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    print(mtx)
    print(dist)
    print(rvecs)
    print(tvecs)
    return mtx, dist


def find_outer(mtx, dist, img):
    cbraw = 10
    cbcol = 6
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((cbraw*cbcol, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cbraw, 0:cbcol].T.reshape(-1, 2)
    objp = objp*20

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(
        gray, (10, 6), None, cv2.CALIB_CB_FAST_CHECK)
    if(not ret):
        return 0, 0, 0
    ret, corners = cv2.findChessboardCorners(gray, (10, 6), None)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.01)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    retval, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
    # print(rvec)
    # print(tvec)
    return rvec, tvec, 1

def quit(signum, frame):
    print('You choose to stop me.')
    sys.exit()

class msg():
    def __init__(self):
        self.i = 0
        self.cv_image = 0
        self.s=0
        self.ns=0
        self.x=0
        self.y=0
        self.z=0


class image_converter:

    def __init__(self):
        #self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.i = 0
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/image_converter/output_video", Image, self.callback)
        self.mtx, self.dist = find_inner_pam()
        self.timelate = time.time()
        self.my_queue = Queue.Queue()
        self.output_queue = Queue.Queue()
        self.worker1=Worker(self.my_queue,1,self.mtx, self.dist,self.output_queue)
        self.worker2=Worker(self.my_queue,2,self.mtx, self.dist,self.output_queue)
        self.worker3=Worker(self.my_queue,3,self.mtx, self.dist,self.output_queue)
        self.worker4=Worker(self.my_queue,4,self.mtx, self.dist,self.output_queue)
        self.worker5=Worker(self.my_queue,5,self.mtx, self.dist,self.output_queue)
        self.worker6=Worker(self.my_queue,6,self.mtx, self.dist,self.output_queue)
        self.worker7=Worker(self.my_queue,7,self.mtx, self.dist,self.output_queue)
        self.worker8=Worker(self.my_queue,8,self.mtx, self.dist,self.output_queue)
        self.worker1.setDaemon(True)
        self.worker2.setDaemon(True)
        self.worker3.setDaemon(True)
        self.worker4.setDaemon(True)
        self.worker5.setDaemon(True)
        self.worker6.setDaemon(True)
        self.worker7.setDaemon(True)
        self.worker8.setDaemon(True)
        self.worker1.start()
        self.worker2.start()
        self.worker3.start()
        self.worker4.start()
        self.worker5.start()
        self.worker6.start()
        self.worker7.start()
        self.worker8.start()
        self.outputworker=threading.Thread(target=self.printoutput)
        self.outputworker.setDaemon(True)
        self.outputworker.start()
    def printoutput(self):
        while 1:    
            while self.output_queue.qsize()==0:
                pass
            amsg=self.output_queue.get()
            print(str(amsg.s)+"."+str(amsg.ns), ",",
                  amsg.x, ",", amsg.y, ",", amsg.z)

    def callback(self, data):
        t0 = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        self.i = self.i+1
        t1 = time.time()
        t2 = time.time()
        #thread.start_new_thread(find_outer,(self.mtx, self.dist, cv_image))
        #a,b,c=find_outer(self.mtx, self.dist, cv_image)
        frame = msg()
        frame.cv_image = cv_image
        frame.i = self.i
        frame.s = data.header.stamp.secs
        frame.ns = data.header.stamp.nsecs
        self.my_queue.put(frame)
        t3 = time.time()
        if 0:
            self.i = self.i-1
        #print(self.i, ",", t1-t0, ",", t2-t1, ",",
        #      t3-t2, ",", t3-t0, ",", t0-self.timelate)
        self.timelate = time.time()

        # try:
        #    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #    print(e)


class Worker(threading.Thread):
    def __init__(self, queue, num, mtx, dist,output_queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.num = num
        self.mtx = mtx
        self.dist = dist
        self.t=time.time()
        self.output_queue=output_queue
    def run(self):
        global pub
        while 1:
            while self.queue.qsize() == 0 :
                pass
            msg = self.queue.get()
            out_msg=Imu()
            a, b, c = find_outer(self.mtx, self.dist, msg.cv_image)
            out_msg.header.stamp.nsecs=msg.ns
            out_msg.header.stamp.secs=msg.s
            if c==1:
                rr, _ = cv2.Rodrigues(np.array([a[0][0], a[1][0], a[2][0]]))
                tt = np.array([b[0][0], b[1][0], b[2][0]])
                msg.r = rr.transpose()
                msg.t = -msg.r.dot(tt)
                msg.x = msg.t[0]
                msg.y = msg.t[1]
                msg.z = msg.t[2]
            out_msg.linear_acceleration.x=msg.x
            out_msg.linear_acceleration.y=msg.y
            out_msg.linear_acceleration.z=msg.z  
                     
            #print(msg.s,",",msg.ns,",",out_msg.linear_acceleration.x,",",out_msg.linear_acceleration.y,",",out_msg.linear_acceleration.z)
            self.output_queue.put(msg)
            pub.publish(out_msg)
            
            #print(self.t-time.time())
            self.t=time.time()

signal.signal(signal.SIGINT, quit)
signal.signal(signal.SIGTERM, quit)
ic = image_converter()
rospy.init_node('find_matrix', anonymous=True)
pub = rospy.Publisher('out_val', Imu, queue_size=10)
try:
    rospy.spin()
except KeyboardInterrupt:
    ISEXIT=1
    print("Shutting down")
cv2.destroyAllWindows()
ic.worker1.join()
ic.worker2.join()
ic.worker3.join()
ic.worker4.join()
ic.worker5.join()
ic.worker6.join()
ic.worker7.join()
ic.worker8.join()

