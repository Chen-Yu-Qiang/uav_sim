#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import Queue
import threading
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import math
from geometry_msgs.msg import Point
from cv2 import aruco
import time
import numpy as np
def ROSdata2CV(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        return cv_image
    except CvBridgeError as e:
        print(e)
        return

def quaternion_from_matrix(matrix):
    T1=np.append(matrix,np.array([[0,0,0]]),axis=0)
    T2=np.append(T1,np.array([[0],[0],[0],[1]]),axis=1)
    

    q = np.empty((4, ), dtype=np.float64)
    M = np.array(T2, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q

def qmulq(q1,q2):
    q3=[0,0,0,0]

    q3[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]
    q3[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2]
    q3[2]=q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1]
    q3[3]=q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]
    return q3


N_worker = 1


class image_converter:
    def __init__(self):
        self.image_sub = rospy.Subscriber(
            "tello_raw", Image, self.callback)
        self.input_queue = Queue.LifoQueue()
        self.output_queue = Queue.LifoQueue()
        self.worker = [Worker(self.input_queue, self.output_queue)
                       for i in range(N_worker)]
        for i in range(N_worker):
            self.worker[i].setDaemon(True)
            self.worker[i].start()
        self.pub_marker = [rospy.Publisher(
            "marker_"+str(i), Marker, queue_size=1) for i in range(20)]
        self.outputworker = threading.Thread(target=self.pub_output)
        self.outputworker.setDaemon(True)
        self.outputworker.start()
    def callback(self, data):
        self.input_queue.put((data,time.time()))

    def pub_output(self):
        while 1:
            while self.output_queue.qsize() == 0:
                pass
            res = self.output_queue.get()
            self.pub_marker[int(res.id)].publish(res)
            

last_frame=0
class Worker(threading.Thread):
    def __init__(self, _input_queue, _output_queue):
        threading.Thread.__init__(self)
        self.input_queue = _input_queue
        self.output_queue = _output_queue

    def run(self):
        global last_frame,cam_pub
        while 1:
            while self.input_queue.qsize() == 0:
                pass
            
            msg = self.input_queue.get()
            if msg[1]<last_frame:
                continue
            else:
                last_frame=msg[1]
            img = ROSdata2CV(msg[0])
            corners, ids, img = detect(img)
            if ids.__str__() == "None":
                continue
            for i in range(len(ids)):
                if ids[i] >= 20:
                    continue
                a = Marker()
                a.id = ids[i]
                a.header = msg[0].header
                a.points = [Point() for k in range(4)]
                for j in range(4):
                    a.points[j].x = corners[i][0][j][0]
                    a.points[j].y = corners[i][0][j][1]
                    a.points[j].z = 1
                self.output_queue.put(a)

            camera_matrix = np.array([[562, 0.000000, 480.5], [0.000000, 562, 360.5], [0.000000, 0.000000, 1.000000]])
            dist_coeffs = np.array([0, 0, 0, 0, 0])
            aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            board_corners = [np.array([[0,-0.5,0.3],[0,-0.3,0.3],[0,-0.3,0.1],[0,-0.5,0.1]], dtype=np.float32),
                             np.array([[0,-0.5,0.7],[0,-0.3,0.7],[0,-0.3,0.5],[0,-0.5,0.5]], dtype=np.float32),
                             np.array([[0,-0.5,1.1],[0,-0.3,1.1],[0,-0.3,0.9],[0,-0.5,0.9]], dtype=np.float32),
                             np.array([[0,-0.1,0.9],[0,0.1,0.9],[0,0.1,0.7],[0,-0.1,0.7]], dtype=np.float32),
                             np.array([[0,0.3,0.3],[0,0.5,0.3],[0,0.5,0.1],[0,0.3,0.1]], dtype=np.float32),
                             np.array([[0,0.3,0.7],[0,0.5,0.7],[0,0.5,0.5],[0,0.3,0.5]], dtype=np.float32),
                             np.array([[0,0.3,1.1],[0,0.5,1.1],[0,0.5,0.9],[0,0.3,0.9]], dtype=np.float32)]
            board_ids = np.array([0,1,2,3,4,5,6], dtype=np.int32)
            board = aruco.Board_create(board_corners,
                               aruco.getPredefinedDictionary(
                               aruco.DICT_ARUCO_ORIGINAL),
                               board_ids)
            retval, rvec, tvec =   aruco.estimatePoseBoard(corners,ids, board, camera_matrix,dist_coeffs)
            if retval == 0:
                continue
            a=rvec
            b=tvec
            rr, _ = cv2.Rodrigues(np.array([a[0][0], a[1][0], a[2][0]]))
            tt = np.array([b[0][0], b[1][0], b[2][0]+0.21])
            cam_r = rr.transpose()
            cam_t = -cam_r.dot(tt)
            cam_x = cam_t[0]
            cam_y = cam_t[1]
            cam_z = cam_t[2]
            cam_q=quaternion_from_matrix(cam_r)
            tq=[-0.5, 0.5, 0.5, 0.5]
            q3=qmulq(tq,cam_q)
            pubmsg=PoseStamped()
            pubmsg.pose.position.x=cam_x
            pubmsg.pose.position.y=cam_y
            pubmsg.pose.position.z=cam_z
            pubmsg.pose.orientation.w=q3[3]
            pubmsg.pose.orientation.x=q3[0]
            pubmsg.pose.orientation.y=q3[1]
            pubmsg.pose.orientation.z=q3[2]
            pubmsg.header.frame_id="world"
            cam_pub.publish(pubmsg)


aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
#aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)



def detect(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #img=cv2.GaussianBlur(img, (11, 11), 0)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementMaxIterations=100
    params.cornerRefinementMinAccuracy=0.01
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=params)
    dr=img

    # theID = 99
    # thePoint = [[0, -0.03, 0.03],
    #         [0, 0.03, 0.03],
    #         [0, 0.03, -0.03],
    #         [0, -0.03, -0.03]]
    # board_corners = [np.array(thePoint, dtype=np.float32)]
    # board_ids = np.array([theID], dtype=np.int32)
    # board = aruco.Board_create(board_corners,aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL),board_ids)
    # camera_matrix = np.array([[562, 0.000000, 480.5], [0.000000, 562, 360.5], [0.000000, 0.000000, 1.000000]], dtype=np.float32)
    # dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
    # dr=cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    dr=aruco.drawDetectedMarkers(dr,corners,ids)
    # retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, None, None)
    # if retval:
    #     dr=aruco.drawAxis(dr, camera_matrix, dist_coeffs, rvec, tvec, 0.5)
    cv2.imshow("a",dr)
    cv2.waitKey(1)
    return corners, ids, img

rospy.init_node('tim', anonymous=True)
ic = image_converter()
cam_pub=rospy.Publisher("cam_from_all_marker",PoseStamped,queue_size=1)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
