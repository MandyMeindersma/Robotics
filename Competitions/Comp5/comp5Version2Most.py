#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
import face_recognition
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy


class Facerec:
    def __init__(self):
        self.sound = SoundClient()
        #time.sleep(1)
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        # our states: LearnFaces, GoHome, LookingForFace, MatchToHome, Guide, FindHuman
        self.state = "LearnFaces"
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
        Image, self.image_callback, queue_size = 1, buff_size = 2**24)
        self.image = None
        pic_of_mandy = face_recognition.load_image_file("Mandy.jpeg")
        pic_of_michele = face_recognition.load_image_file("Michele.jpg")
        self.mandy_encoding = face_recognition.face_encodings(pic_of_mandy)[0]
        self.michele_encoding = face_recognition.face_encodings(pic_of_michele)[0]
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()


    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("window",self.image)
        cv2.waitKey(1)

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


    def take_home(self, name):
        print("taking",name,"home")
        self.state = "MatchToHome"
        goal = goal_pose(self.homes[name])
        self.state = "Guide"
        #self.client.send_goal(goal)
        self.client.wait_for_result()
        self.sound.play(SoundRequest.NEEDS_UNPLUGGING)
        time.sleep(1)
        self.state = "GoHome"
        print("going back to robot home")
        goal = goal_pose(self.homes["robot"])
        #client.send_goal(goal)
        client.wait_for_result()
        print("arrived back home")
    

    def run(self):
        self.state = "LookingForFace"
        print("starting detection")
        cv2.imwrite("unknown.jpg", self.image)
        loaded_image = face_recognition.load_image_file("unknown.jpg")
        try:
            unknown_face_encoding = face_recognition.face_encodings(loaded_image)[0]
            if face_recognition.compare_faces([self.mandy_encoding], unknown_face_encoding)[0]:
                print("I see Mandy!")
                self.sound.say("I found mandy")
                time.sleep(2)
                self.take_home("Mandy")
            elif face_recognition.compare_faces([self.michele_encoding], unknown_face_encoding)[0]:
                print("I see Michele!")
                self.sound.say("I found michelle")
                time.sleep(2)
                self.take_home("Michele")
            else:
                print("I see an unknown face")
        except IndexError:
	    print("No face seen")

rospy.init_node('facerec')
facerec = Facerec()
while facerec.image == None:
    time.sleep(0.25)
for i in range(50):
    facerec.run()
rospy.spin()
