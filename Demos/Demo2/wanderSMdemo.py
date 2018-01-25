#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import smach
import smach_ros
import time

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = msg.ranges[len(msg.ranges)/2]

g_range_ahead = 1

class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['more_forward','hit']) 

    def execute(self, userdata):
        rospy.loginfo('Executing state Forward')
        time.sleep(0.1)
        twist = Twist()
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        twist.linear.x = 0.2
        cmd_vel_pub.publish(twist)
        if g_range_ahead < 0.8:
            return 'hit'
        else:
            return 'more_forward'

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
            twist.angular.z = 1
        cmd_vel_pub.publish(twist)
        if g_range_ahead < 2:
            return 'more_spinning'
        else:
            return 'spun'

def main():
    rospy.init_node('wander')
    
    rospy.Subscriber('scan', LaserScan, scan_callback)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    global cmd_vel_pub
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['start'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Forward', Forward(), 
                               transitions={'more_forward':'Forward', 
                                            'hit':'Spinning'})
        smach.StateMachine.add('Spinning', Spinning(), 
                               transitions={'spun':'Forward',
                                            'more_spinning':'Spinning'})

 
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/start')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()
   
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
