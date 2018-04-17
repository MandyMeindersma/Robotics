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
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class Facerec:
    def __init__(self):
        self.sound = SoundClient()
        # press b to cancel the program
        self.b_pressed = False
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        # our states: LearnFaces, GoHome, LookingForFace, MatchToHome, Guide, FindHuman
        self.state = "LearnFaces"

        # Subscribers 
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback, queue_size = 1, buff_size = 2**24)
        self.web_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.webcam_callback)
        rospy.Subscriber('joy', Joy, self.sense_joystick)


        self.image = None
        self.webcam = None

        # training facial recognition
        pic_of_mandy = face_recognition.load_image_file("Mandy.jpeg")
        pic_of_michele = face_recognition.load_image_file("Michele.jpg")
        self.mandy_encoding = face_recognition.face_encodings(pic_of_mandy)[0]
        self.michele_encoding = face_recognition.face_encodings(pic_of_michele)[0]
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        # dictionary of names that match to home coordinates
        self.homes = {"Mandy": [(34.1367874146, 20.6174278259, 0.0), (0.0, 0.0, 0.604348280606, 0.796720249353)],
                      "Michele": [(39.3439826965, 17.4054641724, 0.0), (0.0, 0.0, -0.82186885021, 0.569676744352)],
                      "Robot": [(36.6986694336, 19.0020484924, 0.0), (0.0, 0.0, -0.967686757782, 0.252155386247)]}

    def webcam_callback(self, msg):
        # encode the webcam image into a jpg for the face
        # recognition program to analyze it
        self.webcam = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite("unknown_webcam.jpg", self.webcam)
        webcam_image = face_recognition.load_image_file("unknown_webcam.jpg")
        try:
            unknown_face_encoding = face_recognition.face_encodings(webcam_image)[0]
            if face_recognition.compare_faces([self.mandy_encoding], unknown_face_encoding)[0]:
                print("Mandy is following")
            elif face_recognition.compare_faces([self.michele_encoding], unknown_face_encoding)[0]:
                print("Michele is following")
        except:
            pass

    def sense_joystick(self, data):
        if data.buttons[1] == 1:
            self.b_pressed = True
            print("button pushed to cancel")
            self.client.cancel_goal()

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
        print("taking %s home" % name)
        self.state = "MatchToHome"
        
        # getting the persons home from the dictionary
        goal = self.goal_pose(self.homes[name])
        self.state = "Guide"
        
        # going to the persons home
        self.client.send_goal(goal)
        self.client.wait_for_result()
        # if we haven't cancelled the program, make the noise
        if self.b_pressed != True:
            self.sound.playWave("/home/malbach/catkin_ws/src/my_awesome_code/src/party.wav")
            time.sleep(1)
            self.sound.say("Thank you for travelling ARIA robotics")
        
        # getting robot's home from dictionary
        self.state = "GoHome"
        print("going back to ARIA's home")
        goal = self.goal_pose(self.homes["Robot"])
        
        # going to robots home 
        self.client.send_goal(goal)
        self.client.wait_for_result()
        # if we haven't cancelled the program, make the noise
        if self.b_pressed != True:
            self.sound.say("ARIA is home")
            time.sleep(0.5)
        print("arrived to ARIA's home")  

    def run(self):
        self.state = "LookingForFace"
        # Starting detection
        # encode the image into a jpg for the face recognition program to analyze it
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
# waiting for the camera to load up
while facerec.image == None:
    time.sleep(0.25)
# starting program and will run until b is pressed on the joystick
while facerec.b_pressed != True:
    facerec.run()
rospy.spin()
