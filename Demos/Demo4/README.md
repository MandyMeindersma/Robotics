# part 1:
We had to run the code in the text book and change parameters to make a map. <br/>
To run:
* roscore
* roslaunch turtlebot_stage turtlebot_in_stage.launch (starts the stage)
* roslaunch turtlebot_teleop keyboard_teleop.launch (to move robot)
* rosbag record -O data.bag /scan /tf (to start bag, Ctrl + C when done)
* rosparam set use_sim_time true
* rosrun gmapping slam_gmapping
* kill the stage
* rosrun gmapping slam_gmapping (start gmapping)
* rosbag play --clock data.bag (start the bag)
* roslaunch turtlebot_rviz_launchers view_navigation (to see the map the robot is making)
* let the bag play through and **DO NOT** kill gmapping
* if it is a good map: rosrun map_server map_saver <br/>

Parameters changed: (straight from the text)
* rosparam set /slam_gmapping/angularUpdate 0.1
* rosparam set /slam_gmapping/linearUpdate 0.1
* rosparam set /slam_gmapping/lskip 10
* rosparam set /slam_gmapping/xmax 10
* rosparam set /slam_gmapping/xmin -10
* rosparam set /slam_gmapping/ymax 10
* rosparam set /slam_gmapping/ymin -10 <br/>

I found that this lab only working if you went really slow, spun a lot and almost ran into walls a lot


# Part 2:
* We needed to make sure to add a compressed image when we ran rviz navigate
    * add rgb/image_raw/image/compressed
    * show the map, robot model, image