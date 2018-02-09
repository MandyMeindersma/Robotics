#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from kobuki_msgs.msg import BumperEvent 
import smach
import smach_ros
import time
import math
import numpy as np

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = msg.ranges[len(msg.ranges)/2]

def sense_bump(data):
    global bump
    if (data.state == BumperEvent.PRESSED):
        bump=True
        print("bump!")
    else:
        bump=False

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

g_range_ahead = 1
bump = False
x_pressed = False
b_pressed = False

class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['more_forward','hit'],
                                   input_keys=['forward_vel_in'],
                                   output_keys=['forward_vel_out']) 

    def ramped_vel(self, v_prev, v_target, ramp_rate):
        print(v_prev)
        step = ramp_rate * 0.1
        sign = 1.0 if (v_target > v_prev) else -1.0
        error = math.fabs(v_target - v_prev)
        if error < step: # we can get there in this time so we are done
            return v_target
        else:
            return v_prev + sign*step



    def execute(self, userdata):
        rospy.loginfo('Executing state Forward')
        time.sleep(0.1)
        twist = Twist()
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        if b_pressed:
            while True:
                #print("stopped")
                time.sleep(2)
                if x_pressed:
                    break
        if bump or g_range_ahead < 0.8:
            global time_started_spinning
            time_started_spinning = rospy.Time.now()
            output = 'hit'
            ramp_rate = 1
            target = 0
        else:
            output = 'more_forward'
            target = 0.5
            ramp_rate = 0.5
        new_vel = self.ramped_vel(userdata.forward_vel_in, target, ramp_rate)
        twist.linear.x = new_vel
        userdata.forward_vel_out = new_vel
        cmd_vel_pub.publish(twist)
        return output

# define state Bar
class Spinning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['spun','more_spinning'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Spinning')
        time.sleep(0.1)
	for i in range(10000):
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 2
        cmd_vel_pub.publish(twist)
        time_now = rospy.Time.now()
        time_spinning = (time_now-time_started_spinning).to_sec()
        if (time_spinning < 1 or bump) or g_range_ahead < 1.5:
            return 'more_spinning'
        else:
            return 'spun'

def main():
    rospy.init_node('wander')
    
    rospy.Subscriber('scan', LaserScan, scan_callback)
    rospy.Subscriber('mobile_base/events/bumper', BumperEvent, sense_bump)
    rospy.Subscriber('joy', Joy, sense_joystick)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    global cmd_vel_pub
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['start'])
    sm.userdata.vel = 0
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Forward', Forward(), 
                               transitions={'more_forward':'Forward', 
                                            'hit':'Spinning'},
                               remapping={'forward_vel_in':'vel', 
                                          'forward_vel_out':'vel'})
        smach.StateMachine.add('Spinning', Spinning(), 
                               transitions={'spun':'Forward',
                                            'more_spinning':'Spinning'})

 
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/start')
    sis.start()
    
    # Pause until x is pressed
    print("waiting for x")
    while not x_pressed:
        time.sleep(0.1)

    # Execute the state machine
    outcome = sm.execute()
   
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
