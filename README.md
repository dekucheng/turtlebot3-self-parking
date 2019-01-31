# winter_project

## Basic setup for turtlebot3:
1. Setup network configuration for remote PC
2. Setup network configuration for turtlebot3 PC
3. Linking both remote PC and turtlebot3 into the same network
4. SSH into turtlebot3


## How to run turtlebot3:
1. Since during this project, turtlebot3 and remote PC shares one ROS master, so the first thing is to run
>**$ roscore**

  in remote PC terminal.
2. SSH into turtlebot3 PC (raspberry Pi 3), using command
>**$ roslaunch turtlebot3_bringup turtlebot3_robot.launch**

  to bring up the turtlebot. After doing this, check rostopic to see if the topics on turtlebot are available.

3. Now turtlebot has already been set up and is the ROS is ready to run nodes for the turtlebot. (So far see examples in manipulation.py)
