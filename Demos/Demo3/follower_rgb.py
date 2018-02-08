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
    else:
        x_pressed = False
    if data.buttons[1] == 1:
        b_pressed = True
    else:
        b_pressed = False


x_pressed = False
b_pressed = False

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
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
    # we found rgb values worked better than hsv
    lower_white = numpy.array([ 200, 200, 200])
    upper_white = numpy.array([ 255, 255, 255])
    mask = cv2.inRange(image, lower_white, upper_white)
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      if not old_err:
          old_err=err
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err)/100-(float(err)-float(old_err))/400
      self.cmd_vel_pub.publish(self.twist)
      old_err = err
      # END CONTROL
    cv2.imshow("window", image)
    cv2.waitKey(3)

old_err = False
rospy.init_node('follower')
rospy.Subscriber('joy', Joy, sense_joystick)
print("waiting for x")
while not x_pressed:
  time.sleep(0.1)
follower = Follower()
rospy.spin()
# END ALL
