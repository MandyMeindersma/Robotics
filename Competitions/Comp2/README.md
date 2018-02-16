We had three different attempts to complete the competition.

We ran 'run_robot_run_split_slow.py' three times through the actual competition. The other attempts were good strategies but we didn't have enough time to fix their flaws before the competition.

To run 'run_robot_run_split_slow.py':
* roscore
* roslaunch turtlebot_bringup minimal.launch
* roslaunch turtlebot_bringup 3dsensor.launch
* rosparam set joy-node/dev "/dev/input/js0"
* rosrun joy joy_node (so we can use x to start and b to stop)
* ./run_robot_run_split_slow.py 