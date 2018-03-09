#!/usr/bin/python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

mtx = np.array( [530.0468491632084, 0, 305.7908398719642, 0, 536.6970704459519, 227.5366146063226, 0, 0, 1]).reshape(3,3)
dist = np.array([0.04159931021864422, 0.05317735597022381, 0.007979039551103254, -0.01515462749100357, 0])

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)


bridge = cv_bridge.CvBridge()
cv2.namedWindow("window", 1)
image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback, queue_size=1, buffer_size = 2**24)
ret = False
corners = []

def image_callback(self, msg):
    print("image received")
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (6,8), None)

    if ret == True:
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Find the rotation and translation vectors.
        rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

        draw(image,corners,imgpts)
        cv2.imshow('img',image)
        k = cv2.waitKey(0) & 0xff
        if k == 's':
            cv2.imwrite(fname[:6]+'.png', image)


