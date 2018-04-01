#!/usr/bin/env python

import rospy
import actionlib
import time
import cv2, cv_bridge
import numpy as np
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from ar_track_alvar_msgs.msg import AlvarMarkers



class lineDraw:
    def __init__(self):
        self.goals = [
    [(34.8270530701, 21.1348762512, 0.0), (0.0, 0.0, -0.967432649012, 0.253128563431)],
    [(35.9640197754, 18.6812362671, 0.0), (0.0, 0.0, -0.961414753787, 0.27510301925)],
    [(37.5730323792, 16.0557155609, 0.0), (0.0, 0.0, -0.979576526523, 0.201071700358)],
    [(37.8687438965, 14.8478679657, 0.0), (0.0, 0.0, -0.53267074317, 0.846322562248)],
    [(39.1145324707, 15.6891212463, 0.0), (0.0, 0.0, -0.267870520349, 0.963454920756)],
    [(38.5109367371, 16.8916435242, 0.0), (0.0, 0.0, 0.32939375092, 0.944192648168)],
    [(37.3425521851, 18.9704246521, 0.0), (0.0, 0.0, 0.256098529929, 0.966650683012)],
    [(36.3644256592, 21.5547428131, 0.0), (0.0, 0.0, 0.292296415224, 0.956327771032)],
    [(36.3235473633, 22.2742328644, 0.0), (0.0, 0.0, 0.691313402643, 0.7225550355)],
    [(34.8143348694, 22.1438808441, 0.0), (0.0, 0.0, 0.841190673753, 0.540738615591)]]
        self.bridge = cv_bridge.CvBridge()
        self.see_ar = False
        self.cx = None
        self.cy = None
        self.w = None
        self.cam_info_sub = rospy.Subscriber('/cv_camera/camera_info', CameraInfo, self.info_cam)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.ar_pose_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback)
        self.joy_sub=rospy.Subscriber('joy', Joy, self.sense_joystick)
        self.K = np.array([611.3861169294532, 0, 349.5989504734198, 0, 617.8556465275495, 190.4231910503, 0, 0, 1]).reshape(3,3)
        self.D = np.array([0.1452856819290304, 0.1794432829997692, -0.05502919448787864, 0.009347880730169251, 0])
        self.tvecs = None
        self.rvecs = None
        self.markers = None
        self.axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1],[0,0,0]]).reshape(-1,3)
        # global localization
        rospy.wait_for_service('global_localization')
        global_localization = rospy.ServiceProxy('global_localization', Empty)
        global_localization()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.tw = Twist()
        print('turning...')
        index = 0
        while index < 40:
	    time.sleep(0.5)
	    self.tw.angular.z = 0.5
            self.tw.linear.x = 0
            self.cmd_vel_pub.publish(self.tw)
            index += 1
        print("done localizing")

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        while True:
            for pose in self.goals:
                goal = self.goal_pose(pose)
                self.client.send_goal(goal)
                self.client.wait_for_result()
                print(self.see_ar)
                if self.see_ar == True:
                    i = 0
                    while i < 40 and self.see_ar == True:
                        time.sleep(0.1)
		        err = self.cx - self.w/2
                        self.tw.linear.x = 2.3
                        self.tw.angular.z = -float(err)/100
                        self.cmd_vel_pub.publish(self.tw)
                        i += 1


	

    def sense_joystick(self, data):
        if data.buttons[1] == 1:
            print("stopping")
            #self.client.cancelGoal()
            while True:
                time.sleep(5)

    def goal_pose(self, pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

    def info_cam(self, msg):
        self.K = np.array(msg.K).reshape(3,3)
        self.D = np.array(msg.D)

    def ar_callback(self, msg):
        self.markers = msg

    def image_callback(self, msg):
        #print(self.rvecs)
        img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        h, self.w, d = img.shape

        if not self.markers == None:
            if not (self.markers.markers==[]):
                self.see_ar = True
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

                (self.cx,self.cy) = tuple(imgpts[3].ravel())

                cv2.circle(img,(self.cx,self.cy),20,(0,0,255),-1)

        cv2.imshow('img',img)
        cv2.waitKey(1)

if __name__ == "__main__":
        print("running code")
	rospy.init_node('lineDraw')
	lineDraw = lineDraw()
	rospy.spin()
