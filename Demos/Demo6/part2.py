#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
import glob
import math

from sensor_msgs.msg import Image, CameraInfo
from ar_track_alvar_msgs.msg import AlvarMarkers

class lineDraw:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.cam_info_sub = rospy.Subscriber('/cv_camera/camera_info', CameraInfo, self.info_cam)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.ar_pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)
        self.K = np.array([611.3861169294532, 0, 349.5989504734198, 0, 617.8556465275495, 190.4231910503, 0, 0, 1]).reshape(3,3)
        self.D = np.array([0.1452856819290304, 0.1794432829997692, -0.05502919448787864, 0.009347880730169251, 0])
        self.tvecs = None
        self.rvecs = None
        self.markers = None
        self.axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1],[0,0,0]]).reshape(-1,3)

    def info_cam(self, msg):
        self.K = np.array(msg.K).reshape(3,3)
        self.D = np.array(msg.D)

    def ar_callback(self, msg):
        self.markers = msg

    def image_callback(self, msg):
        print(self.rvecs)
        img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        if not self.markers == None:
            if not (self.markers.markers==[]):
                pose = self.markers.markers[0].pose.pose
                px = pose.position.x
                py = pose.position.y
                pz = pose.position.z
                ox = pose.orientation.x
                oy = pose.orientation.y
                oz = pose.orientation.z
                ow = pose.orientation.w

                self.tvecs = np.array([px, py, pz])

                angle = 2 * math.acos(ow)
                x = ox / math.sqrt(1 - ow*ow)
                y = oy / math.sqrt(1 - ow*ow)
                z = oz / math.sqrt(1 - ow*ow)
                ratio = math.sqrt(x*x + y*y + z*z)

                #normalize them
                x = x / ratio*angle
                y = y / ratio*angle
                z = z / ratio*angle
                
                self.rvecs = np.array([x, y, z])

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(self.axis, self.rvecs, self.tvecs, self.K, self.D)

        cv2.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[0].ravel()), (255,0,0), 5)
        cv2.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[1].ravel()), (0,255,0), 5)
        cv2.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[2].ravel()), (0,0,255), 5)
        cv2.imshow('img',img)
        cv2.waitKey(1)

if __name__ == "__main__":
	rospy.init_node('lineDraw')
	lineDraw = lineDraw()
	rospy.spin()