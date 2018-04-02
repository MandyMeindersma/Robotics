# to run:
* roscore 
* roslaunch pkg_name egg.launch
* rosrun map_server map_server Comp3Map.yaml
* roslaunch turtlebot_navigation amcl_demo.launch map_file:=./Comp3Map.yaml
* ./comp4.py

# sounds
download:
* sudo apt-get install ros-inidigo-sound-play

play a sound:
* rosrun sound_play soundplay_node.py 
* rosrun sound_play play.py ~/winter18/Robotics/Competitions/Comp4/meow.ogg
