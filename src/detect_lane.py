#!/usr/bin/env python
from __future__ import division
import rospy
import cv2
import roslib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import copy
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
import numpy as np
import os



np.set_printoptions(precision=5, suppress=True)  # suppress scientific float notation

class detect_laneoffset_center():

    def __init__(self):
        self.lane_center_pub = rospy.Publisher("lane_center",Float64, queue_size = 1)
        self.Bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_projected_compensated",Image,self.gtimage,queue_size = 1)
        self.mid_lane_pub = rospy.Publisher("/mid_lane_Image",Image, queue_size = 1)




    def gtimage(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''

        self.lane_image = self.Bridge.imgmsg_to_cv2(ros_data,"bgr8")
        #cv2.imshow('img',self.lane_image)
        self.height = self.lane_image.shape[0]
        self.width = self.lane_image.shape[1]
        #path='/home/zhicheng/turtlebot3ws/src/turtlebot3_selfparking/shared_files'
        #cv2.imwrite(os.path.join(path,'lane.jpg'),self.lane_image)

        # use  hshv color to filter out the black lines
        hsv = cv2.cvtColor(self.lane_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([0,0,50])
        upper_yellow = np.array([180,255,150])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        res = cv2.bitwise_and(self.lane_image,self.lane_image,mask=mask)
        cannyed_image = cv2.Canny(res, 30, 85)
        #cv2.imshow('img',res)
        #cv2.waitKey(1000)
        #cv2.destroyAllWindows()
        #print(gray_image.shape)

        # calculate the image histogram after being filted
        histogram = np.sum(cannyed_image[res.shape[0]//2:,:], axis=0)
        #histogram = np.sum(histogram[:,:], axis=1)

        midpoint = np.int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # derive the offset value, negative means offset by right, positive means offset by left.
        # publish to rqt to visulize if parameters are set correctself
        #mid_lane_img = copy.copy(self.lane_image)
        #print(int((leftx_base+rightx_base)/2))
        cannyed_image = cv2.line(cannyed_image, (int((leftx_base+rightx_base)/2), self.height), (int((leftx_base+rightx_base)/2),0), (255,0,0), 2)
        cannyed_image = cv2.cvtColor(cannyed_image, cv2.COLOR_GRAY2BGR)
        cannyed_image = self.Bridge.cv2_to_imgmsg(cannyed_image,"bgr8")

        center = (leftx_base+rightx_base)/2

        rospy.loginfo("midpoint of lanes is: %d"%(center))
        self.mid_lane_pub.publish(cannyed_image)
        self.lane_center_pub.publish(center)



def main():

    rospy.init_node('detect_lane_offset')

    lane_ct = detect_laneoffset_center()
    rospy.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
