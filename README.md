# Cmput 412 - Experimental Mobile Robotics

Check out our site: https://sites.google.com/ualberta.ca/experimental-mobile-robotics/home

Making a package: <br/>
setup:
* mkdir -p ~/catkin_ws/src
* cd ~/catkin_ws/src
* catkin_init_workspace
* cd ~/catkin_ws
* catkin_make
* source devel/setup.bash <br/>
new package:
* cd ~/catkin_ws/src
* catkin_create_pkg my_awesome_code rospy
* DO NOT FORGET TO CHANGE EXECUTE PERMISSIONS chmod u+x topic_publisher.py


never use this link: (it permanently changes your bashrc file)
http://wiki.ros.org/Robots/TurtleBot/Network%20Setup
