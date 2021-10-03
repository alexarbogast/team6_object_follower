# team6_turtlebot

Repository for ME 7785 labs using the turtlebot3

## Packages
* **team6_object_follower**

This package rotates the robot to follow an object.
The object is tracked using contour tracking, and the HSV values can be tuned using the parameter assistant node.
```
rosrun team6_object_follower param_assistant.py
```
The turtlebot startup routines and object traking can be started through a launch file as follows:
```
roslaunch team6_object_follower turtlebot3_track_object.launch
```
