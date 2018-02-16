run part 1:
* roscore
* source the devel setup files
* roslaunch followbot course.launch
* ./follower_yellow.py (this has to have the yellow hsv values)

run part 2:
* roscore
* roslaunch turtlebot_bringup minimal.launch
* roslaunch turtlebot_bringup 3dsensor.launch
* rosparam set joy-node/dev "/dev/input/js0"
* rosrun joy joy_node (so we can use x to start and b to stop)
* ./follower_rgb.py (this has to have the white rgb values)
