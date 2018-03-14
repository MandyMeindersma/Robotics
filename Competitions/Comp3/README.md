# to run:
* roscore
* roslaunch turtlebot_bringup minimal.launch
* rosparam set joy-node/dev "/dev/input/js0"
* rosrun joy joy_node (to hit the joystick buttons to kill the program)
* rosrun map_server map_server ./Comp3Map.yaml
* roslaunch turtlebot_navigation amcl_demo.launch map_file:=./Comp3Map.yaml
* roslaunch turtlebot_rvix_launchers view_navigation.launch
    * this is where we can initialize points
    * this is also where we can get point info
* rosrun rqt_reconfigure rqt_reconfigure (to change speed)
* rosparam set /move_base/min_vel_x 0.7 or /move_base/TragectoryPlannerROS/min_vel_x 0.7
* ./MapFollower.py

