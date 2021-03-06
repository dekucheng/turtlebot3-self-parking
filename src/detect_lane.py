#!/usr/bin/env python
from __future__ import division
import rospy
import cv2
import roslib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import copy
from sensor_msgs.msg import Image,CompressedImage
from std_srvs.srv import Trigger
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
        rospy.Service('detect_lane/turn_off', Trigger, self.svc_turn_off)


    def svc_turn_off(self, data):
        self.image_sub.unregister()
        return (True,'detect_lane node has been turned off!')

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
        lower_color = np.array([0,0,50])
        upper_color = np.array([20,255,255])

        lower_color2 = np.array([0,0,0])
        upper_color2 = np.array([180,255,40])

        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask2 = cv2.inRange(hsv, lower_color2, upper_color2)
        mask = mask + mask2


        # fill the white trigle space with black triangles on left and right side of bottom
        triangle1 = np.array([[0, 599], [0, 240], [200, 599]], np.int32)
        triangle2 = np.array([[999, 599], [999, 240], [799, 599]], np.int32)
        black = (0, 0, 0)
        white = (255, 255, 255)
        mask = cv2.fillPoly(mask, [triangle1, triangle2], black)

        res = cv2.bitwise_and(self.lane_image,self.lane_image,mask=mask)
        cannyed_image = cv2.Canny(res, 30, 85)
        cannyed_image = cv2.fillPoly(cannyed_image, [triangle1, triangle2], black)


        # calculate the image histogram after being filted
        histogram = np.sum(mask[res.shape[0]//2:,:], axis=0)


        midpoint = np.int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint


        cannyed_image = cv2.line(cannyed_image, (int((leftx_base+rightx_base)/2), self.height), (int((leftx_base+rightx_base)/2),0), (255,0,0), 2)
        mask = cv2.line(mask, (int((leftx_base+rightx_base)/2), self.height), (int((leftx_base+rightx_base)/2),0), (255,0,0), 2)

        cannyed_image = cv2.cvtColor(cannyed_image, cv2.COLOR_GRAY2BGR)
        cannyed_image = self.Bridge.cv2_to_imgmsg(cannyed_image,"bgr8")

        # below modified for report
        res = self.Bridge.cv2_to_imgmsg(res, "bgr8")
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask = self.Bridge.cv2_to_imgmsg(mask, "bgr8")
        center = (leftx_base+rightx_base)/2

        rospy.loginfo("midpoint of lanes is: %d"%(center))
        self.mid_lane_pub.publish(mask)
        self.lane_center_pub.publish(center)




def main():

    rospy.init_node('detect_lane')

    lane_ct = detect_laneoffset_center()
    rospy.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
