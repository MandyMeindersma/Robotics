#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
import face_recognition
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
import time

class Facerec:
    def __init__(self):
        self.sound = SoundClient()
        #time.sleep(1)
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
        Image, self.image_callback, queue_size = 1, buff_size = 2**24)
        self.image = None
        pic_of_mandy = face_recognition.load_image_file("Mandy.jpeg")
        pic_of_michele = face_recognition.load_image_file("Michele.jpg")
        self.mandy_encoding = face_recognition.face_encodings(pic_of_mandy)[0]
        self.michele_encoding = face_recognition.face_encodings(pic_of_michele)[0]

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("window",self.image)
        cv2.waitKey(1)

    def face(self):
        print("starting detection")
        cv2.imwrite("unknown.jpg", self.image)
        loaded_image = face_recognition.load_image_file("unknown.jpg")
        try:
            unknown_face_encoding = face_recognition.face_encodings(loaded_image)[0]
            if face_recognition.compare_faces([self.mandy_encoding], unknown_face_encoding)[0]:
                print("I see Mandy!")
                self.sound.say("I found mandy")
                time.sleep(2)
            if face_recognition.compare_faces([self.michele_encoding], unknown_face_encoding)[0]:
                print("I see Michele!")
                self.sound.say("I found michelle")
                time.sleep(2)
        except IndexError:
	    print("No face seen")

rospy.init_node('facerec')
facerec = Facerec()
time.sleep(5)
if facerec.image != None:
    for i in range(20):
        #print(i)
        facerec.face()
rospy.spin()
