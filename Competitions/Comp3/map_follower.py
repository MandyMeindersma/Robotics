#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy

# Competition runs added 5th point
goals = [
    [(34.0622825623, 21.9256591797, 0.0), (0.0, 0.0, -0.843529172371, 0.537083359786)],
    [(37.005405426, 15.1303691864, 0.0), (0.0, 0.0, -0.30610268063, 0.951998502578)],
    [(40.28515625, 16.7959804535, 0.0), (0.0, 0.0, 0.716519445132, 0.697567118454)],
    [(39.6048049927, 17.7464504242, 0.0), (0.0, 0.0, 0.965418572492, 0.260704775346)],
    [(37.3469619751, 23.1265106201, 0.0), (0.0, 0.0, 0.999872376429, 0.0159759461281)]]


# Competition run wide
#goals = [
 #   [(33.8187675476, 21.9315605164, 0.0), (0.0, 0.0, -0.822799818349, 0.568331293283)],
  #  [(37.1337242126, 14.5787649155, 0.0), (0.0, 0.0, -0.222414685179, 0.974952156681)],
   # [(39.9550018311, 16.9167613983, 0.0), (0.0, 0.0, 0.798297620636, 0.602263155843)],
    #[(36.9681434631, 23.4497394562, 0.0), (0.0, 0.0, 0.868666418431, 0.49539747021)]]

# corners, robot starting from north west corner
#goals = [
 #   [(34.5, 20.0, 0.0), (0.0, 0.0, -0.8, 0.6)],
  #  [(35.2, 15.35, 0.0), (0.0, 0.0, -0.15, 0.98)],
   # [(37.5, 15.75, 0.0), (0.0, 0.0, 0.6, 0.8)],
    #[(35.5, 21.2, 0.0), (0.0, 0.0, 0.95, 0.2)]]
    

# safe points with robot starting from north west corner
#goals = [
 #   [(33.9, 22.2, 0.0), (0.0, 0.0, -0.98, 0.2)],
  #  [(37.5, 15.0, 0.0), (0.0, 0.0, -.55, 0.8)],
   # [(39.0, 17.0, 0.0), (0.0, 0.0, 0.2, 0.98)],
    #[(36.7, 23.0, 0.0), (0.0, 0.0, 0.8, 0.55)]]

# initial testing, too tight
#goals = [
#    [(38.6375770569, 18.1390075684, 0.0), (0.0, 0.0, 0.84311165689, 0.537738536852)],
#    [(36.6167945862, 22.2664661407, 0.0), (0.0, 0.0, -0.971353508665, 0.237639140725)],
#    [(34.4763793945, 21.2282066345, 0.0), (0.0, 0.0, -0.501865079396, 0.864945918589)],
#    [(36.810836792, 16.9542865753, 0.0), (0.0, 0.0, 0.183434540733, 0.98303192688)]
#]

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
