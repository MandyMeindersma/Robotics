# to run Comp 5:
* roscore
* roslaunch turtlebot_bringup minimal.launch
* rosparam set joy-node/dev "/dev/input/js0"
* rosrun joy joy_node (to hit the joystick buttons to kill the program)
* rosrun map_server map_server Comp3Map.yaml
* roslaunch turtlebot_navigation amcl_demo.launch map_file:=./Comp3Map.yaml
* roslaunch turtlebot_rviz_launchers view_navigation.launch
    * this is where we can initialize points
    * this is also where we can get point info
* ./Comp5


# to instal python face recognition:
* first, if you have ubuntu 14.04 you will need to go through this tutorial: https://github.com/ageitgey/face_recognition/issues/120
    * instead of `wget https://bootstrap.pypa.io/get-pip.py` do `sudo wget https://bootstrap.pypa.io/get-pip.py`
    * instead of `sudo python get-pip.p` do `sudo python get-pip.py`
* then install with this link: https://pypi.python.org/pypi/face_recognition
    * instead of `pip3 install face_recognition` do `sudo pip2 install face_recognition`
* run our comp5FaceRec.py code to test that it was installed correctly or run `face_recognition ./test ./test2`. You should get a "test is not a directory" error (This command is supposed to compare a directory of known faces and unknown faces to try to recognize them)
* YOU WILL LOOK LIKE A BLUE BERRY, UNCLEAR WHY...



# to instal ROS face recognition:
* follow this tutorial: http://wiki.ros.org/face_recognition
    * instead of `rosrun face_recognition Fserver` do ` rosrun face_recognition Fserver camera/image_raw:=camera/rgb/image_raw`
    * instead of `rosrun face_recognition Fclient` do ` rosrun face_recognition Fclient camera/image_raw:=camera/rgb/image_raw`