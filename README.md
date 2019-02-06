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

## CV part:

See initial CV line detection node in src/.ipynb file.

After going through several lane detection examples, I found that the main strategy of them are similar. Here is the summary of general method to detect the lane on road(or parking lines):
1. Cropping the raw image into a region of interest.
2. Convert the raw image into a Grayscale image to simplify edge detection.
3. Use Canny Edge Detection to find the edges.
4. Generating Lines from Edge Pixels.
5. Using Hough Transfroms to detect lines.
6. Mark the detected lines as an overlap.
7. Creating the lines we want by grouping the lines with certain property (e.g. the slope)
8. Plot the lines we derived in the image.

### Things to do:
1. There are basically two ways to detect the lines from turtlebot3, one is to detect by image from a snapshot of the camera, another one is to detect by video in real time.(The second one is much cooler than the first one and I am ready to try that.)
2. Since the basic line detection just set a filter to collect the lines whose slopes are in a certain range, it may not be robust in video detection because the slopes are changing. So next I am going to develop this method to detect the changing lines.
