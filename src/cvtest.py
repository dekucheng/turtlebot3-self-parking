#!/usr/bin/env python
from __future__ import print_function, division
import sys, time
import rospy
import cv2
import roslib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy.ndimage import filters
from std_msgs.msg import String
import copy
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from turtlebot3_selfparking.srv import*
from std_srvs.srv import Trigger
import numpy as np
import os
from scipy.cluster.hierarchy import dendrogram, linkage
# some setting for this notebook to actually show the graphs inline
# you probably won't need this
#matplotlib inline
np.set_printoptions(precision=5, suppress=True)  # suppress scientific float notation


VERBOSE = True

def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
# If there are no lines to draw, exit.
    if lines is None:
        return
    # Make a copy of the original image.
    img = np.copy(img)

    # Create a blank image that matches the original in size.
    line_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=np.uint8,
    )

    # Loop over all lines and draw them on the blank image.
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    # Merge the image with the lines onto the original.
    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

    return img

class image_receiver():

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_1",CompressedImage,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_rect_color/compressed",CompressedImage,self.gtimage)
        self.call_init_image = rospy.ServiceProxy('cv_test/provide_rectified_image', Trigger)
        rospy.Service('cv_test/provide_rectified_image',Trigger,self.svc_set_init_image)
        self.image_np = CompressedImage()




    def gtimage(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE :
            print('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 reac means the name of subscribing topic reac_compress ####

        self.image_reac = ros_data.data


        '''
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        #### Feature detectors using CV2 ####
        # "","Grid","Pyramid" +
        # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        '''
        '''
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        '''
    # CompressedIamge to cv2 image and init init_image
    def compress2img(self,data):
        np_arr = np.fromstring (data,np.uint8)
        self.init_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.height = self.init_image.shape[0]
        self.width = self.init_image.shape[1]
        self.region_of_vertices = [
            (0, self.height),
            #(0, 350),
            (200,300),
            (270,250),
            (550,300),
            (self.width,self.height),
            #(width,280),
            #(width, 170),
            #(width, height),
        ]
        #self.subscriber.unregister()

    def region_of_interest(self, img, vertices):

        mask = np.zeros_like(img)
        match_mask_color = 255 # This line altered for grayscale.
        cv2.fillPoly(mask, vertices, match_mask_color)
        print(321)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def generate_cropped_lines(self, img):



        # using hsv to filter out the color we want
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        lower_color = np.array([35,50,150])
        upper_color = np.array([40,250,250])
        # mask the hsv image in color range
        mask = cv2.inRange(hsv, lower_color, upper_color)

        plt.figure()
        plt.imshow(mask)
        plt.show()

        res = cv2.bitwise_and(img,img,mask=mask)
        cannyed_image = cv2.Canny(res, 20, 35)
        cropped_image = self.region_of_interest(
            cannyed_image,
            np.array([self.region_of_vertices], np.int32)
        )

        plt.imshow(cropped_image)
        plt.show()

        lines = cv2.HoughLinesP(
            cropped_image,
            rho=6,
            theta=np.pi / 100,
            threshold=70,
            lines=np.array([]),
            minLineLength=40,
            maxLineGap=8
        )
        print('hough lines detected, lines number:',lines.shape[0])
        self.hough_lines = lines

        line_image = draw_lines(park5, self.hough_lines) # <---- Add this call.

        plt.figure()
        plt.imshow(line_image)


    def generate_hough_params(self):
        slope_list = list()
        dev_list = list()
        for line in self.hough_lines:
            if (line[0][2]-line[0][0]) != 0:  # eliminate the lines with y1 = y2
                slope = (line[0][3]-line[0][1])/(line[0][2]-line[0][0])
                dev = line[0][1]-slope*line[0][0]
                slope_list.append(slope)
                dev_list.append(dev)

        # combine slope and deviation in one list
        lines_array = np.column_stack((slope_list, dev_list))
        # filter can also be developed, like use (rou,theta) as data
        lines_array_filted = copy.copy(lines_array)

        for i in range(lines_array_filted.shape[0]):
            if np.absolute(lines_array_filted[i][0]) > 3.5:
                lines_array_filted[i][0] = lines_array_filted[i][0]/lines_array_filted[i][0] * 3.5
                lines_array_filted[i][1] = lines_array_filted[i][1]/lines_array_filted[i][1] * 1200
        Z = linkage(lines_array_filted, 'ward')
        max_d = image.shape[0]*0.6
        clusters = fcluster(Z, max_d, criterion='distance')
        cluster_number = np.unique(clusters).shape[0]


    def svc_set_init_image(self, data):
        '''
        self.compress2img(data)
        self.generate_cropped_lines(self.init_image)



        plt.figure()
        plt.scatter(slope_list,dev_list)
        plt.show()
        lines.shape
        '''
        self.compress2img(self.image_reac)
        rospy.sleep(1)
        #cv2.imshow('cv_img', self.init_image)

        # save the image to path
        #path = '/home/zhicheng/turtlebot3ws/src/turtlebot3_selfparking/src'
        #cv2.imwrite(os.path.join(path,'parking_test_02.jpg'),self.init_image)
        #cv2.waitKey(3000)
        #cv2.destroyAllWindows()
        '''
        self.testimg=cv2.imread('parking_test_05.jpg')

        print(self.testimg)
        cv2.imshow('img',self.testimg)
        cv2.waitKey(3000)
        cv2.destroyAllWindows()
        '''




        return (True,"Has recerived the image")

def main():



    rospy.init_node('cv_test')
    global park5
    path='/home/zhicheng/turtlebot3ws/src/turtlebot3_selfparking/src/parking_test_08.jpg'
    park5=mpimg.imread(path)
    print(park5.shape)
    ic = image_receiver()
    rospy.sleep(3)
    ic.call_init_image()
    ic.generate_cropped_lines(park5)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
