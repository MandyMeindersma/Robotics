#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import cv2, cv_bridge
import numpy as np
import math

def scan_callback(msg):
    global g_range_ahead
    global g_range_left
    global g_range_right
    g_range_ahead = msg.ranges[len(msg.ranges)/2]
    g_range_left = msg.ranges[len(msg.ranges)/4]
    g_range_right = msg.ranges[len(msg.ranges)*3/4]

g_range_ahead = 1
g_range_left = 1
g_range_right = 1

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback, queue_size = 1, buff_size = 2**24)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        h, w, d = image.shape
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(grey, (6,8), None)
        print("g-range", g_range_ahead)
        self.twist.linear.x = 0
        self.twist.angular.x = 0
        squaresize = 0
        if ret:
            squaresize = corners[3][0][0] - corners[2][0][0]
            print("square", squaresize)
            mid = corners[2][0][0]
        if squaresize < 40:
            print("square good")
            if ret == True:
                if mid < w/2-180:
                    print("go left")
                    self.twist.angular.z = 0.2
                    self.twist.linear.x = 0
                elif mid > w/2+180:
                    print("go right")
                    self.twist.angular.z = -0.2
                    self.twist.linear.x = 0
                else:
                    print("go straight")
                    self.twist.linear.x = 0.1
                    #print(g_range_left)
                    #print(g_range_right)
                    if g_range_right - g_range_left > 0.05:
                        print("drift left")
                        self.twist.angular.z = 0.4
                    elif g_range_left - g_range_right > 0.05:
                        print("drift right")
                        self.twist.angular.z = -0.4
            elif g_range_ahead > 1:
                print("blind")
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
        else:
            while True:
                True
        cv2.imshow("window", image)
        cv2.waitKey(3)

rospy.init_node('follower')
rospy.Subscriber('scan', LaserScan, scan_callback)
follower = Follower()
rospy.spin()
