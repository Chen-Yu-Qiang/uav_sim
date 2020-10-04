#!/usr/bin/env python
from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco
import sys
import numpy as np
import time
import Queue
import pickle
import os
"""
board_corners = [np.array([[0.14142, 0.05858, 0.205], [0.05, 0.2, 0.205], [0.05, 0.2, 0.005], [0.14142, 0.05858, 0.005]], dtype=np.float32),
                 np.array([[0, 0, 0.7], [0, 0.2, 0.7], [0, 0.2, 0.5],
                           [0, 0, 0.5]], dtype=np.float32),
                 np.array([[0, 0, 1.2], [0, 0.2, 1.2], [0, 0.2, 1],
                           [0, 0, 1]], dtype=np.float32),
                 np.array([[0.2, 0.25, 0], [0.2, 0.45, 0], [0.4, 0.45, 0],
                           [0.4, 0.25, 0]], dtype=np.float32),
                 np.array([[0, 0.25, 0.95], [0, 0.45, 0.95], [
                          0, 0.45, 0.75], [0, 0.25, 0.75]], dtype=np.float32),
                 np.array([[0.05, 0.5, 0.145], [0.05, 0.7, 0.145], [0.19142, 0.7, 0.0035],
                           [0.19142, 0.5, 0.0035]], dtype=np.float32),
                 np.array([[0, 0.5, 0.7], [0, 0.7, 0.7], [0, 0.7, 0.5],
                           [0, 0.5, 0.5]], dtype=np.float32),
                 np.array([[0, 0.5, 1.2], [0, 0.7, 1.2], [0, 0.7, 1],
                           [0, 0.5, 1]], dtype=np.float32)]
board_ids = np.array([[0], [6], [7], [4], [5], [1], [2], [3]], dtype=np.int32)"""


def create_board(cor, a_id):
    board_corners = [np.array(cor, dtype=np.float32)]
    board_ids = np.array([[a_id]], dtype=np.int32)
    board = aruco.Board_create(board_corners,
                               aruco.getPredefinedDictionary(
                                   aruco.DICT_5X5_250),
                               board_ids)
    return board


def find_aruco(img):
    frame = img
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gus = cv2.GaussianBlur(gray, (15, 15), 0)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
    if len(corners) < 1:
        return 0, 0, 0
    sub_corner = []
    for corner in corners:
        corner = cv2.cornerSubPix(gray, corner, (5, 5), (-1, 1),
                                  (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        sub_corner = sub_corner + [corner]

    return sub_corner, ids


def cal_aruco(corners, ids,i):
    retval, rvec, tvec = aruco.estimatePoseBoard(
        corners, ids, board[i], camera_matrix, dist_coeffs, None, None)

    if not retval:
        return 0, 0, 0
    return rvec, tvec, 1


def drwa_aruco(frame, corners, ids, rvec, tvec):
    gray = aruco.drawAxis(frame, camera_matrix, dist_coeffs,
                          rvec, tvec, aruco_marker_length_meters)
    frame = aruco.drawDetectedMarkers(gray, corners, ids)
    cv2.imshow("A", frame)
    cv2.waitKey(10)


aruco_marker_length_meters =1
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [
                         0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])
board = [None for i in range(8)]
board[0] = create_board([[0.2, 0, 0.21], [0.0585, 0.1414, 0.21], [
                        0.0585, 0.1414, 0.01], [0.2, 0, 0.01]], 0)
board[6] = create_board(
    [[0, 0, 0.7], [0, 0.2, 0.7], [0, 0.2, 0.5], [0, 0, 0.5]], 6)
board[7] = create_board(
    [[0, 0, 1.2], [0, 0.2, 1.2], [0, 0.2, 1], [0, 0, 1]], 7)
board[4] = create_board([[0.2, 0.25, 0], [0.2, 0.45, 0], [
                        0.4, 0.45, 0], [0.4, 0.25, 0]], 4)
board[5] = create_board([[0, 0.25, 0.95], [0, 0.45, 0.95], [
                        0, 0.45, 0.75], [0, 0.25, 0.75]], 5)
board[1] = create_board([[0.058, 0.5, 0.148], [0.058, 0.7, 0.148], [
                        0.2, 0.7, 0.0071], [0.2, 0.5, 0.0071]], 1)
board[2] = create_board([[0, 0.5, 0.7], [0, 0.7, 0.7], [
                        0, 0.7, 0.5], [0, 0.5, 0.5]], 2)
board[3] = create_board(
    [[0, 0.5, 1.2], [0, 0.7, 1.2], [0, 0.7, 1], [0, 0.5, 1]], 3)
f = "1345/"
filelist=os.listdir(f)
filelist.sort()
t0=float(filelist[0][:-4])
for i in filelist:
    tim=float(i[:-4])-t0
    cv_image = cv2.imread(f+i)
    cor, ids = find_aruco(cv_image)
    for j in range(8):
        a, b, c = cal_aruco(cor, ids,j)
        if c == 1:
            #drwa_aruco(cv_image, cor, ids, a, b)
            rr, _ = cv2.Rodrigues(np.array([a[0][0], a[1][0], a[2][0]]))
            tt = np.array([b[0][0], b[1][0], b[2][0]])
            r = rr.transpose()
            t = -r.dot(tt)
            x = t[0]
            y = t[1]
            z = t[2]
            print("%f,%f,%f,%f,%d,%f,%f,%f"%(tim,x,y,z,j,a[0][0], a[1][0], a[2][0]))
        else:
            print("%f,0,0,0,%d,0,0,0"%(tim,j))
