#!/usr/bin/env python

import rospy
import actionlib
import rospy
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy

# Competition runs added 5th point
goals = [
    [(34.0622825623, 21.9256591797, 0.0), (0.0, 0.0, -0.843529172371, 0.537083359786)],
    [(37.005405426, 15.1303691864, 0.0), (0.0, 0.0, -0.30610268063, 0.951998502578)],
    [(40.28515625, 16.7959804535, 0.0), (0.0, 0.0, 0.716519445132, 0.697567118454)],
    [(39.6048049927, 17.7464504242, 0.0), (0.0, 0.0, 0.965418572492, 0.260704775346)],
    [(37.3469619751, 23.1265106201, 0.0), (0.0, 0.0, 0.999872376429, 0.0159759461281)]]

def joy_callback(msg):
    global running
    global client
    if msg.buttons[1]==1:
        print 'Exit...'
        client.cancel_goal()
        os._exit(0)

def goal_pose(pose):
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

if __name__ == '__main__':
    rospy.init_node('patrol')
    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)

    # global localization
    rospy.wait_for_service('global_localization')
    global_localization = rospy.ServiceProxy('global_localization', Empty)
    global_localization()

    # do some random movements
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    tw = Twist()
    print('turning...')
    index = 0
    while index < 40:
        tw.angular.z = 0.8
        tw.linear.x = 0.8
        cmd_vel_pub.publish(tw)
        index += 1

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    for i in range(2):
        for pose in goals:
            print("new goal")
            # does a lap
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
    goal = goal_pose(goals[0])
    client.send_goal(goal)
    client.wait_for_result()
