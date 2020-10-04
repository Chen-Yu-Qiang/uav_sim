#!/usr/bin/env python

import rospy

import cv2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from cv2 import aruco
import numpy as np
import math
import tf

rospy.init_node('findRt', anonymous=True)
theID = 0
thePoint = []
theID = rospy.get_param('~theID')
thePoint = [[rospy.get_param('~p1x'), rospy.get_param('~p1y'), rospy.get_param('~p1z')],
            [rospy.get_param('~p2x'), rospy.get_param(
                '~p2y'), rospy.get_param('~p2z')],
            [rospy.get_param('~p3x'), rospy.get_param(
                '~p3y'), rospy.get_param('~p3z')],
            [rospy.get_param('~p4x'), rospy.get_param('~p4y'), rospy.get_param('~p4z')]]

print(theID, thePoint)


def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True
    """
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
#[921.170702, 0.000000, 459.904354, 0.000000, 919.018377, 351.238301,0.000000, 0.000000, 1.000000]
#[921.170702, 0.000000, 459.904354,0, 0.000000, 919.018377, 351.238301,0,0.000000, 0.000000, 1.000000,0]
def qmulq(q1,q2):
    q3=[0,0,0,0]

    q3[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]
    q3[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2]
    q3[2]=q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1]
    q3[3]=q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]
    return q3
def callback(data):
    global theID, thePoint,cam_tf,cam_pub
    ####for real tello
    #camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [
    #    0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
    #dist_coeffs = np.array(
    #    [-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])
    ####for sim
    camera_matrix = np.array([[562, 0.000000, 480.5], [
        0.000000, 562, 360.5], [0.000000, 0.000000, 1.000000]])
    dist_coeffs = np.array([0, 0, 0, 0, 0])
    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    board_corners = [np.array(thePoint, dtype=np.float32)]
    board_ids = np.array([theID], dtype=np.int32)
    board = aruco.Board_create(board_corners,
                               aruco.getPredefinedDictionary(
                               aruco.DICT_ARUCO_ORIGINAL),
                               board_ids)
    i = 0
    corners = []
    ids = np.array([data.id], dtype=np.int32)
    corners = np.array([[[data.points[0].x, data.points[0].y],
                         [data.points[1].x, data.points[1].y],
                         [data.points[2].x, data.points[2].y],
                         [data.points[3].x, data.points[3].y]]], dtype=np.float32)
    retval, rvec, tvec = aruco.estimatePoseBoard(
        corners, ids, board, camera_matrix, dist_coeffs, None, None)
    if retval == 0:
        return 0, 0, 0
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
    cam_tf.sendTransform((cam_x,cam_y,cam_z),cam_q,rospy.Time.now(),"Tello_cam_from_marker"+str(theID),"world")
    cam_tf_t.sendTransform((0,0,0),(0.5, -0.5, 0.5, 0.5),rospy.Time.now(),"Tello_cam_from_marker_world"+str(theID),"Tello_cam_from_marker"+str(theID))
    pubmsg=PoseStamped()
    pubmsg.pose.position.x=cam_x
    pubmsg.pose.position.y=cam_y
    pubmsg.pose.position.z=cam_z
    pubmsg.pose.orientation.w=q3[3]
    pubmsg.pose.orientation.x=q3[0]
    pubmsg.pose.orientation.y=q3[1]
    pubmsg.pose.orientation.z=q3[2]
    pubmsg.header=data.header
    pubmsg.header.frame_id="world"
    cam_pub.publish(pubmsg)
    #print(rvec, tvec)


marker_sub = rospy.Subscriber("marker_"+str(theID), Marker, callback)
cam_tf_t = tf.TransformBroadcaster()
cam_tf = tf.TransformBroadcaster()
cam_pub=rospy.Publisher("cam_from_marker_"+str(theID),PoseStamped,queue_size=1)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
