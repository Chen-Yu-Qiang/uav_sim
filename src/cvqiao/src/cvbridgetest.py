#!/usr/bin/env python
from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import rospy
import sys
import numpy as np
import time
import roslib
import thread
roslib.load_manifest('cvqiao')


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
    ret, corners = cv2.findChessboardCorners(gray, (10, 6), None,cv2.CALIB_CB_FAST_CHECK)
    if(not ret):
        return 0,0,0
    ret, corners = cv2.findChessboardCorners(gray, (10, 6), None)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.01)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    retval, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
    #print(rvec)
    #print(tvec)
    return rvec, tvec,1


class image_converter:

    def __init__(self):
        #self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        self.i=0
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/image_converter/output_video", Image, self.callback)
        self.mtx, self.dist = find_inner_pam()
        self.timelate=time.time()
    def callback(self, data):
        t0=time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        self.i=self.i+1
        t1=time.time()
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
        t2=time.time()
        thread.start_new_thread(find_outer,(self.mtx, self.dist, cv_image))
        #a,b,c=find_outer(self.mtx, self.dist, cv_image)
        t3=time.time()
        if 0:
            self.i=self.i-1
        print(self.i,",",t1-t0,",",t2-t1,",",t3-t2,",",t3-t0,",",t0-self.timelate)
        self.timelate=time.time()
        # try:
        #    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #    print(e)


def main(args):
    ic = image_converter()
    rospy.init_node('find_matrix', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
