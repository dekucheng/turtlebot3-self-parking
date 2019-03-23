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
from scipy.cluster.hierarchy import fcluster
from matplotlib.patches import Circle
from geometry_msgs.msg import Point
# some setting for this notebook to actually show the graphs inline
# you probably won't need this
#matplotlib inline
np.set_printoptions(precision=5, suppress=True)  # suppress scientific float notation


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

def get_new_lines(k_ave, b_ave, width):
    new_lines = np.zeros((k_ave.shape[0],1,4))
    for i in range(b_ave.shape[0]):

        new_y2 = k_ave[i][0]*width + b_ave[i][0]
        new_y2 = int(new_y2)
        b_ave[i][0] = int(b_ave[i][0])
        new_y1 = b_ave[i][0]
        temp_lines = np.zeros((1,4))
        temp_lines[0] = [0, new_y1, width, new_y2]
        new_lines[i] = temp_lines

    new_lines = new_lines.astype(int)
    return new_lines


# change the represent format of each line
def lines_array_represent(lines):
    lines_invk = copy.copy(lines)
    #lines_invk[:,0] = -lines_invk[:,0]
    lines_2pts = np.zeros((lines.shape[0],4))

    for i in range(lines_invk.shape[0]):
        k = lines_invk[i][0]
        b = lines_invk[i][1]
        if 0 <= b <= 480:
            lines_2pts[i][0] = 0
            lines_2pts[i][1] = b
            check = 640 * k + b
            if 0 <= check <= 480:
                lines_2pts[i][2] = 640
                lines_2pts[i][3] = check
            elif check > 480:
                lines_2pts[i][2] = (480-b)/k
                lines_2pts[i][3] = 480
            else:
                lines_2pts[i][2] = -b/k
                lines_2pts[i][3] = 0

        elif lines[i][1] > 480:
            lines_2pts[i][0] = (480-b)/k
            lines_2pts[i][1] = 480
            check = 640 * k + b
            if 0 <= check <= 480:
                lines_2pts[i][2] = 640
                lines_2pts[i][3] = check
            else:
                lines_2pts[i][2] = -b/k
                lines_2pts[i][3] = 0

        else:
            lines_2pts[i][0] = -b/k
            lines_2pts[i][1] = 0
            check = 640 * k + b
            if 0 <= check <= 480:
                lines_2pts[i][2] = 640
                lines_2pts[i][3] = check
            else:
                lines_2pts[i][2] = (480-b)/k
                lines_2pts[i][3] = 480

    return lines_2pts

# This function used for take average of slope and intersection (with y axis) in one line cluster
def lines_para_ave(clst_num, clsts, lines_array):
    lineslope_array_cluster = np.zeros((clst_num, 1))
    slope_ave = np.zeros((clst_num, 1))
    linedev_array_cluster = np.zeros((clst_num, 1))
    dev_ave = np.zeros((clst_num, 1))


    for i in range (clst_num):
        count = 0
        for j in range (clsts.shape[0]):
            if clsts[j] == (i+1):
                count += 1
                lineslope_array_cluster[i][0] += lines_array[j][0]
                linedev_array_cluster[i][0] += lines_array[j][1]
        slope_ave[i][0] = lineslope_array_cluster[i][0]/count
        dev_ave[i][0] = linedev_array_cluster[i][0]/count
    return slope_ave, dev_ave

# CompressedIamge to cv2 image and init init_image
def compress2img(data):
    np_arr = np.fromstring (data,np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return img

class image_receiver():

    def __init__(self):
        #self.image_pub = rospy.Publisher("image_topic_1",CompressedImage,queue_size=10)
        self.bridge = CvBridge()
        self.pub_image_crop_lines = rospy.Publisher('cv_test/crop_lines', Image, queue_size=1)
        self.pub_image_cannyed_lines = rospy.Publisher('cv_test/cannyed_lines', Image, queue_size=1)
        self.pub_image_mid_point_img = rospy.Publisher('cv_test/mid_point_img', Image, queue_size=1)
        self.pub_mid_point = rospy.Publisher('cv_test/mid_point', Point, queue_size=1)
        # for report
        #self.image_sub = rospy.Subscriber("/camera/image_rect_color/compressed",CompressedImage,self.gtimage)
        #rospy.Service('cv_test/get_mid_location',GetPointLocation,self.svc_parking_point_pipeline)

        #self.image_sub = rospy.Subscriber("/camera/image_rect_color/compressed",CompressedImage,self.gtimage)
        #rospy.Service('cv_test/get_mid_location',GetPointLocation,self.svc_parking_point_pipeline)
        rospy.Service('cv_test/turn_off', Trigger, self.svc_turn_off)
        rospy.Service('cv_test/turn_on', Trigger, self.svc_turn_on)
        self.image_np = CompressedImage()

        self.CLST_NUM = False
        self.TURN_OFF = False

    def svc_turn_on(self, data):

        self.image_sub = rospy.Subscriber("/camera/image_rect_color/compressed",CompressedImage,self.gtimage)
        rospy.Service('cv_test/get_mid_location',GetPointLocation,self.svc_parking_point_pipeline)
        return(True,'cvtest topic turned on!')
    def svc_turn_off(self,data):

        self.image_sub.unregister()

        return(True,'cvtest topic turned off!')
    def region_of_interest(self, img, vertices):

        mask = np.zeros_like(img)
        match_mask_color = 255 # This line altered for grayscale.
        cv2.fillPoly(mask, vertices, match_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image



    def gtimage(self, ros_data):

        '''Callback function of subscribed topic.
        Here images get converted and features detected'''

        #### direct conversion to CV2 reac means the name of subscribing topic reac_compress ####
        self.image_reac = ros_data.data

        self.init_image = compress2img(self.image_reac)
        cv2.imwrite('/home/zhicheng/turtlebot3ws/src/turtlebot3_selfparking/src/IMAGE.jpg',self.init_image)

        #cv2.imshow('img',self.init_image)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        #print(self.init_image.shape)
        #\self.init_image = park5
        self.height = self.init_image.shape[0]
        self.width = self.init_image.shape[1]
        self.region_of_vertices = [
            (60, self.height*0.8),
            #(0, 350),
            (60, self.height*0.3),
            (580,self.height*0.3),
            (580,self.height*0.8),
        ]
        pts = np.array([[60, self.height*0.8], [60, self.height*0.3], [580,self.height*0.3], [580, self.height*0.8]], np.int32)
        pts = pts.reshape((-1,1,2))

        # using hsv to filter out the color we want
        hsv = cv2.cvtColor(self.init_image, cv2.COLOR_BGR2HSV)
        # lower mask (0-10)
        lower_red = np.array([0,50,10])
        upper_red = np.array([28,250,250])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)
        # upper mask (170-180)
        lower_red = np.array([160,50,10])
        upper_red = np.array([190,250,250])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask = mask0 + mask1

        res = cv2.bitwise_and(self.init_image,self.init_image,mask=mask)
        cannyed_image = cv2.Canny(res, 40, 85)
        cropped_image = self.region_of_interest(
            cannyed_image,
            np.array([self.region_of_vertices], np.int32)
        )
        print('cropped image get')
        show_cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_GRAY2BGR)

        self.pub_image_cannyed_lines.publish(self.bridge.cv2_to_imgmsg(show_cropped_image, "bgr8"))

        lines = cv2.HoughLinesP(
            cropped_image,
            rho=4,
            theta=np.pi / 80,
            threshold=40,
            lines=np.array([]),
            minLineLength=40,
            maxLineGap=10
        )
        if lines is not  None:
            HOUGHLINES = True
            print('hough lines detected, lines number:',lines.shape[0])
            self.hough_lines = lines
            #line_image = draw_lines(self.init_image, self.hough_lines) # <---- Add this call.
            #self.pub_image_crop_lines.publish(self.bridge.cv2_to_imgmsg(line_image, "bgr8"))
        else:
            HOUGHLINES = False
            print('no hough lines detected!')
            #self.pub_image_crop_lines.publish(self.bridge.cv2_to_imgmsg(self.init_image, "bgr8"))

        if HOUGHLINES == True:
            slope_list = list()
            dev_list = list()
            for line in self.hough_lines:
                if (line[0][2]-line[0][0]) != 0:  # eliminate the lines with y1 = y2
                    slope = (line[0][3]-line[0][1])/(line[0][2]-line[0][0])
                    dev = line[0][1]-slope*line[0][0]
                    slope_list.append(slope)
                    dev_list.append(dev)

            lines_array = np.column_stack((slope_list, dev_list))
            #lines_array_represent = copy.copy(lines_array)

            lines_array_2pts = lines_array_represent(lines_array)

            self.clst_number = 0
            if self.hough_lines.shape[0] > 2:
                Z = linkage(lines_array_2pts, 'ward')
                max_d = self.init_image.shape[0]*0.45
                clusters = fcluster(Z, max_d, criterion='distance')
                self.clst_number = np.unique(clusters).shape[0]
                slope_ave, dev_ave = lines_para_ave(self.clst_number,clusters, lines_array)

                self.new_lines = get_new_lines(slope_ave, dev_ave, self.width)
                self.init_image = cv2.polylines(self.init_image, [pts], True, (0,0,255))
                self.new_lines_img = draw_lines(self.init_image, self.new_lines, color=[255, 0, 0], thickness=3)

                self.pub_image_crop_lines.publish(self.bridge.cv2_to_imgmsg(self.new_lines_img, "bgr8"))
                print('%d lines have been detected' %self.clst_number)
            else:
                print('Invalid lines numbers, fail to use hierarchycal K-means')

            if self.clst_number == 3:
                print('TRUE')
                self.CLST_NUM = True
            else:
                self.CLST_NUM = False

       # CompressedIamge to cv2 image and init init_image

    def svc_parking_point_pipeline(self, data):
        print(self.CLST_NUM)
        if self.CLST_NUM == True:
            print('parking lanes number is valid, begin to check intersection points!')
            intersect_pts = list()
            for i in range(self.new_lines.shape[0]):
                for j in range(i+1, self.new_lines.shape[0]):

                    k1 = (self.new_lines[i][0][3] - self.new_lines[i][0][1])/(self.new_lines[i][0][2]-self.new_lines[i][0][0])
                    k2 = (self.new_lines[j][0][3] - self.new_lines[j][0][1])/(self.new_lines[j][0][2]-self.new_lines[j][0][0])
                    x = - (self.new_lines[j][0][1]-self.new_lines[i][0][1])/(k2-k1)
                    y = k1*x + self.new_lines[i][0][1]
                    temp = [x, y]

                    intersect_pts.append(temp)
            intersect_pts = np.array(intersect_pts)
            intersect_pts = intersect_pts.astype(int)
            # drop the pts that not in detect region
            index= 0
            for i in range(intersect_pts.shape[0]):
                x = intersect_pts[index][0]
                y = intersect_pts[index][1]
                if (x<=60 or x>=580 or y<=144 or y>=384):
                    intersect_pts = np.delete(intersect_pts, index, 0)
                    index -= 1
                index += 1

            # get the mid point of the pts
            if intersect_pts.shape[0] == 2:
                print('Intersection points are valid, begin to calculate parking point!')
                mid_x = np.sum(intersect_pts[:,0])/intersect_pts.shape[0]
                mid_y = np.sum(intersect_pts[:,1])/intersect_pts.shape[0]
                ref_y = np.max(intersect_pts[:,1])
                ref_index = np.argmax(intersect_pts[:,1])
                ref_x = intersect_pts[ref_index,0]
                # get the midpoint location and publish it
                self.mid_point = Point()
                self.mid_point.x = mid_x
                self.mid_point.y = mid_y
                self.mid_point.z = 0

            # draw the mid point

                cv2.circle(self.new_lines_img, (np.int(mid_x),np.int(mid_y)), 5, (0,0,255), -1)
                self.pub_image_mid_point_img.publish(self.bridge.cv2_to_imgmsg(self.new_lines_img, "bgr8"))
            else:
                print('No valid parking intersection points detected!')
                mid_x = 0
                mid_y = 0
                ref_x = 0
                ref_y = 0

            return mid_x, mid_y, ref_x, ref_y
        else:
            print('No valid mid point detected!')
            return 0,0,0,0


def main():

    rospy.init_node('cv_test')
    global  HOUGHLINES
    HOUGHLINES = False

    #path='/home/zhicheng/turtlebot3ws/src/turtlebot3_selfparking/src/parking_test_08.jpg'
    #park5=cv2.imread(path)
    #print(park5.shape)
    ic = image_receiver()
    rospy.sleep(3)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
