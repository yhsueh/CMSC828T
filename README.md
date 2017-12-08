# Vision based Quad control AR-drone

## Requirement
-ROS indigo Ubuntu 14.04.5 LTS
-catkin

## Setup
* clone from github
* Type source ~/catkin_ws/devel/setup.bash in every terminal.
or change ~/.bashrc file
* cd ~/catkin_ws
* catkin_make

## Run and build
* Create four terminal windows
* In the first window: run 
```
roscore
```
* Second:
```
rosrun CMSC828T pr2_indiv.launch
```
* Third: 
cd ~/catkin_ws/src/CMSC828T/src (This is needed to read the image from the correct path)
```
rosrun CMSC828T talker
```
* Forth: 
```
rosrun CMSC828T listener
```

## Pipeline

### Take off 

* Create node/terminal:  publish ardrone/takeoff topics ( std_msgs/Empty )

* Check/Wait ardrone/navdata.state => 4 Hovering

### Search for the window tags

* Rotate about Z axis "SLOWLY" for 360 degrees since there is no PID controller. To rotate the quadrotor, publish the message to cmd_vel topic 
* Check the Z-axis angle from the topic ardrone/navdata. If not all four tags are found and the quadrotor has rotated for 360 degrees, then move the quadrotor upward vertically for certain distance say 10 cm.
* After three attempts, repeat the second step toward left.
* After three attempts, repeat the second step toward right.
* If all the tags are found proceed to next phase.

### Adjust quadrotor pose

* From all the corners, extract the camera pose from homography. (For the time being, we will assume the dimension of the window, the size of the window will be given on the demonstration day, so the program must have the ability to read the argument from the users)
* Rotate and translate the quadrotor to the center of the windows.
* Read the camera again and verify that the quadrotor is at the center of the window. Keep translate until the quadrotor is right at the center of the window.

### Fly through the window

* Advance through the window for 50cm.

### Search for the vertical tag.

* Repeat the procedures in "search four tags" 

### Advance to the front of the tag

* Repeat the procedure in "Adjust quadorotor pose

### Hover 

* Finally, stop at the vertical tag and when the size of the tag gets large enough.



