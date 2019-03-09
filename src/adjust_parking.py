#!/usr/bin/env python
from __future__ import print_function, division
import sys, time
import rospy
import cv2
import roslib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy.ndimage import filters
from std_msgs.msg import String,Float64
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
        if 0 <= b <= 240:
            lines_2pts[i][0] = 0
            lines_2pts[i][1] = b
            check = 1000 * k + b
            if 0 <= check <= 240:
                lines_2pts[i][2] = 1000
                lines_2pts[i][3] = check
            elif check > 240:
                lines_2pts[i][2] = (240-b)/k
                lines_2pts[i][3] = 240
            else:
                lines_2pts[i][2] = -b/k
                lines_2pts[i][3] = 0

        elif lines[i][1] > 240:
            lines_2pts[i][0] = (240-b)/k
            lines_2pts[i][1] = 240
            check = 1000 * k + b
            if 0 <= check <= 240:
                lines_2pts[i][2] = 1000
                lines_2pts[i][3] = check
            else:
                lines_2pts[i][2] = -b/k
                lines_2pts[i][3] = 0

        else:
            lines_2pts[i][0] = -b/k
            lines_2pts[i][1] = 0
            check = 1000 * k + b
            if 0 <= check <= 240:
                lines_2pts[i][2] = 1000
                lines_2pts[i][3] = check
            else:
                lines_2pts[i][2] = (240-b)/k
                lines_2pts[i][3] = 240

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

class adjust_parking():

    def __init__(self):
        #self.image_pub = rospy.Publisher("image_topic_1",CompressedImage,queue_size=10)
        self.bridge = CvBridge()
        self.pub_image_crop_lines = rospy.Publisher('adjust_parking/crop_lines', Image, queue_size= 1)
        self.pub_image_cannyed_lines = rospy.Publisher('adjust_parking/cannyed_lines', Image, queue_size= 1)
        self.pub_image_mid_point_img = rospy.Publisher('adjust_parking/mid_point_img', Image, queue_size= 1)
        self.pub_adjust_angle = rospy.Publisher('adjust_parking/adjust_angle', Float64, queue_size = 1)
        self.image_sub = rospy.Subscriber("/camera/image_projected_compensated", Image,self.gtimage)
        #self.call_get_mid_location = rospy.ServiceProxy('adjust_parking/get_mid_location', GetPointLocation)

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
        #self.init_image = self.bridge.imgmsg_to_cv2(self.image_reac)
        self.init_image = self.bridge.imgmsg_to_cv2(ros_data,"bgr8")
        #cv2.imwrite('/home/zhicheng/turtlebot3ws/src/turtlebot3_selfparking/src/IMAGE.jpg',self.init_image)

        self.height = self.init_image.shape[0]
        self.width = self.init_image.shape[1]
        self.region_of_vertices = [
            (0, self.height*0.4),
            #(0, 350),
            (self.width, self.height*0.4),
            (self.width,0),
            (0,0),
        ]
        pts = np.array([[0, self.height*0.4], [self.width, self.height*0.4], [self.width,0], [0,0]], np.int32)
        pts = pts.reshape((-1,1,2))

        # using hsv to filter out the color we want
        hsv = cv2.cvtColor(self.init_image, cv2.COLOR_BGR2HSV)
        # lower mask (0-10)
        lower_red = np.array([0,50,50])
        upper_red = np.array([25,250,250])
        mask0 = cv2.inRange(hsv, lower_red, upper_red)
        # upper mask (170-180)
        lower_red = np.array([160,50,50])
        upper_red = np.array([185,250,250])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask = mask0 + mask1

        res = cv2.bitwise_and(self.init_image,self.init_image,mask=mask)
        cannyed_image = cv2.Canny(res, 30, 85)
        cropped_image = self.region_of_interest(
            cannyed_image,
            np.array([self.region_of_vertices], np.int32)
        )
        show_cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_GRAY2BGR)

        self.pub_image_cannyed_lines.publish(self.bridge.cv2_to_imgmsg(show_cropped_image, "bgr8"))

        lines = cv2.HoughLinesP(
            cropped_image,
            rho=4,
            theta=np.pi / 80,
            threshold=35,
            lines=np.array([]),
            minLineLength=80,
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
                else:
                    slope = 240
                    dev = line[0][1]-slope*line[0][0]
                    slope_list.append(slope)
                    dev_list.append(dev)

            lines_array = np.column_stack((slope_list, dev_list))
            #lines_array_represent = copy.copy(lines_array)

            lines_array_2pts = lines_array_represent(lines_array)
            self.clst_number = 0
            if self.hough_lines.shape[0] > 2:
                Z = linkage(lines_array_2pts, 'ward')
                max_d = self.init_image.shape[0]*0.5
                clusters = fcluster(Z, max_d, criterion='distance')
                self.clst_number = np.unique(clusters).shape[0]
                slope_ave, dev_ave = lines_para_ave(self.clst_number,clusters, lines_array)

                self.new_lines = get_new_lines(slope_ave, dev_ave, self.width)
                self.init_image = cv2.polylines(self.init_image, [pts], True, (0,0,255))
                self.new_lines_img = draw_lines(self.init_image, self.new_lines, color=[255, 0, 0], thickness=3)

                self.pub_image_crop_lines.publish(self.bridge.cv2_to_imgmsg(self.new_lines_img, "bgr8"))
                print('%d clusters lines have been detected' %self.clst_number)
            else:
                print('Invalid lines numbers, fail to use Hierarchycal K-means')

            if self.clst_number >= 1:
                line_random = slope_ave[0][0]
                theta = np.arctan(line_random)
                if theta > 0:
                    self.pub_adjust_angle.publish(theta)
                else:
                    theta = np.pi + theta
                    self.pub_adjust_angle.publish(theta)

def main():

    rospy.init_node('adjust_parking')
    global park5, HOUGHLINES, CLST_NUM
    HOUGHLINES = False
    CLST_NUM = False
    path='/home/zhicheng/turtlebot3ws/src/turtlebot3_selfparking/src/lane.jpg'
    park5=cv2.imread(path)
    print(park5.shape)

    ic = adjust_parking()
    rospy.sleep(3)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
