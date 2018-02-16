#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def sense_joystick(data):
    global x_pressed
    global b_pressed
    print(data.buttons)
    if data.buttons[2] == 1:
        x_pressed = True
    if data.buttons[1] == 1:
        b_pressed = True


x_pressed = False
b_pressed = False

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    cv2.namedWindow("window2", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()
  def image_callback(self, msg):
    global old_err
    if b_pressed:
      while True:
          time.sleep(2)
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    image = cv2.GaussianBlur(image, (5,5), 0)
    h, w, d = image.shape
    image_left = image[:,:w/2]
    image_right = image[:,w/2:]
    # we found rgb values worked better than hsv
    lower_white = numpy.array([ 230, 230, 230])
    upper_white = numpy.array([ 255, 255, 255])
    mask_left = cv2.inRange(image_left, lower_white, upper_white)
    mask_right = cv2.inRange(image_right, lower_white, upper_white)
    h, w, d = image.shape
    search_top_left = 3*h/4
    search_bot_left = h
    search_top_right = 2*h/3
    search_bot_right = h
    mask_left[0:search_top_left, 0:w] = 0
    mask_left[search_bot_left:h, 0:w] = 0
    mask_right[0:search_top_right, 0:w] = 0
    mask_right[search_bot_right:h, 0:w] = 0
    M_left = cv2.moments(mask_left)
    M_right = cv2.moments(mask_right)
    cxleft = False
    cxright = False
    if M_left['m00'] > 0:
      cxleft = int(M_left['m10']/M_left['m00'])
      cyleft = int(M_left['m01']/M_left['m00'])
      cv2.circle(image_left, (cxleft, cyleft), 20, (0,0,255), -1)
    if M_right['m00'] > 0:
      cxright = int(M_right['m10']/M_right['m00'])
      cyright = int(M_right['m01']/M_right['m00'])
      cv2.circle(image_right, (cxright, cyright), 20, (0,0,255), -1)
    # BEGIN CONTROL
    if cxleft and cxright:
      if cxleft > w/4:
        err_right_x = w - cxright
        err_right_y = cyright
        self.twist.linear.x = 0.7
        if err_right_y > 2*h/3: #hard turn
          self.twist.angular.z = float(err_right_x)/100 + err_right_y/800
        else: #straight
          self.twist.angular.z = 0
        old_err = False
      #elif cxright < 5*w/8:
       # err_left_x = cxleft
        #err_left_y = cyleft
        #self.twist.linear.x = 0.7
        #self.twist.angular.z = -float(err_left_x)/600 - err_left_y/1000
        #old_err = False        
      else: #soft turn
        err = (cxleft+cxright)-w/4
        if not old_err:
          old_err=err
        self.twist.linear.x = 0.7
        self.twist.angular.z = -float(err)/1000+(float(err)-float(old_err))/400
        old_err = err
    elif cxleft:
      err_left_x = cxleft
      err_left_y = cyleft
      self.twist.linear.x = 0.7
      self.twist.angular.z = -float(err_left_x)/600 - err_left_y/1000
      old_err = False
    elif cxright:
      err_right_x = w - cxright
      err_right_y = cyright
      self.twist.linear.x = 0.7
      self.twist.angular.z = float(err_right_x)/700 + err_right_y/1000
      old_err = False
    else:
      self.twist.linear.x = 0.6
      self.twist.angular.z = 0
      old_err = False
    if x_pressed:
      self.cmd_vel_pub.publish(self.twist)
    # END CONTROL
    cv2.imshow("window", image_left)
    cv2.imshow("window2",image_right)
    cv2.waitKey(3)

old_err = False
rospy.init_node('follower')
rospy.Subscriber('joy', Joy, sense_joystick)
print("waiting for x")
follower = Follower()
rospy.spin()
# END ALL
