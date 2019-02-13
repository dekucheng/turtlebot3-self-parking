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

## Set up turtlebot3 camera:
Run the commands in turtlebot3  
>**$ roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch**

>**$ roslaunch raspicam_node camerav2_1280x960.launch**

Then in Remote PC, run:
>**$ rqt_image_view**


## CV_test in turtlebot3 camera (week 6):
1. solve the low fps problem by using publish a compressed image and small queue_size.
2. calibration finished, rectified seems well (needs test in real-world)
3. go through the tutorial that how opencv communicate with ROS.
4. CompressedImage msg type seems cannot be transformed by cv_bridge, mannually extract the data from compressedImage to fit into opencv for further operation.

### Things to do:
1. Check if the calibration works well for bird's eye view camera.
2. find proper algorithm to send back the location of detected line.
3. find out urdf relations between camera frame and base frame to navigation.
