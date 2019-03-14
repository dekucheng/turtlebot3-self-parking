#!/usr/bin/env python

import rospy
import numpy as np
import math
import os
import cv2
from enum import Enum
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

class DetectSign():
    def __init__(self):
        self.fnPreproc()

        self.sub_image_type = "compressed"
        self.pub_image_type = "compressed"


            # subscribes compressed image
        self.sub_image_original = rospy.Subscriber('/camera/image_rect_color/compressed', CompressedImage, self.cbFindTrafficSign, queue_size = 1)


        self.pub_traffic_sign = rospy.Publisher('/detect/traffic_sign', UInt8, queue_size=1)

        #if self.pub_image_type == "compressed":
            # publishes traffic sign image in compressed type
        self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output/compressed', CompressedImage, queue_size = 1)
        #elif self.pub_image_type == "raw":
            # publishes traffic sign image in raw type
        #self.pub_image_traffic_sign = rospy.Publisher('/detect/image_output', Image, queue_size = 1)

        self.cvBridge = CvBridge()

        self.TrafficSign = Enum('TrafficSign', 'divide stop parking tunnel')

        self.call_cv_test_node = rospy.ServiceProxy('cv_test/turn_on', Trigger)

        self.call_detect_lane_node = rospy.ServiceProxy('detect_lane/turn_off', Trigger)

        self.call_lane_follow_node = rospy.ServiceProxy('lane_follow/turn_off', Trigger)

        self.call_adjust_parking_node = rospy.ServiceProxy('adjust_parking/turn_on', Trigger)

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)


        self.counter = 1

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create()
        #self.sift = cv2.ORB_create()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('turtlebot3_selfparking/src', 'turtlebot3_selfparking/')
        dir_path += 'file/detect_sign/'

        self.img2 = cv2.imread(dir_path + 'Parking_03.jpg',0)         # trainImage1
        self.img3 = cv2.imread(dir_path + 'Parking_04.jpg',0)      # trainImage2
        self.img4 = cv2.imread(dir_path + 'Parking_05.jpg',0)       # trainImage3

        self.kp2, self.des2 = self.sift.detectAndCompute(self.img2,None)
        self.kp3, self.des3 = self.sift.detectAndCompute(self.img3,None)
        self.kp4, self.des4 = self.sift.detectAndCompute(self.img4,None)

        FLANN_INDEX_KDTREE = 0


        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def region_of_interest(self, img, vertices):

        mask = np.zeros_like(img)
        match_mask_color = 255 # This line altered for grayscale.
        cv2.fillPoly(mask, vertices, match_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
            err = sum / num_all
            return err

    def cbFindTrafficSign(self, image_msg):
        # drop the frame to 1/15 (5fps) because of the processing speed. This is up to your computer's operating power.

        self.region_of_vertices = [
            (120, 480),
            #(0, 350),
            (320, 480),
            (320, 0),
            (120, 0),
        ]
        if self.counter % 2 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        print('GET!')
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        #cv_image_input = self.region_of_interest(
        #    cv_image_input,
        #    np.array([self.region_of_vertices], np.int32)
        #)
        cv_image_input = cv_image_input[320:480, 100:540]
        MIN_MATCH_COUNT = 9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches2 = self.flann.knnMatch(des1,self.des2,k=2)
        matches3 = self.flann.knnMatch(des1,self.des3,k=2)
        matches4 = self.flann.knnMatch(des1,self.des4,k=2)
        #matches2 = bf.match(des1,self.des2)
        #matches2 = sorted(matches, key = lambda x:x.distance)
        #matches3 = self.flann.knnMatch(np.asarray(des1,np.float32),np.asarray(self.des3,np.float32),k=2)
        #matches4 = self.flann.knnMatch(np.asarray(des1,np.float32),np.asarray(self.des4,np.float32),k=2)
        image_out_num = 1

        good2 = []
        for m,n in matches2:
            if m.distance < 0.7*n.distance:
                good2.append(m)

        if len(good2)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good2 ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp2[m.trainIdx].pt for m in good2 ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask2 = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.stop.value

                self.pub_traffic_sign.publish(msg_sign)

                rospy.loginfo("TrafficSign 2")

                image_out_num = 2

        else:
            matchesMask2 = None

        good3 = []
        for m,n in matches3:
            if m.distance < 0.7*n.distance:
                good3.append(m)

        if len(good3)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good3 ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp3[m.trainIdx].pt for m in good3 ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask3 = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.parking.value

                self.pub_traffic_sign.publish(msg_sign)

                rospy.loginfo("TrafficSign 3")

                image_out_num = 3

        else:
            matchesMask3 = None

        good4 = []
        for m,n in matches4:
            if m.distance < 0.7*n.distance:
                good4.append(m)
        if len(good4)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good4 ]).reshape(-1,1,2)
            dst_pts = np.float32([ self.kp4[m.trainIdx].pt for m in good4 ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask4 = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.tunnel.value

                self.pub_traffic_sign.publish(msg_sign)

                rospy.loginfo("TrafficSign 4")

                image_out_num = 4

        else:
            matchesMask4 = None

        if image_out_num == 1:
            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(cv_image_input, "bgr8"))

        elif image_out_num == 2:
            draw_params2 = dict(matchColor = (0,0,255), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matchesMask2, # draw only inliers
                            flags = 2)

            final2 = cv2.drawMatches(cv_image_input,kp1,self.img2,self.kp2,good2,None,**draw_params2)

            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final2, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final2, "bgr8"))

        elif image_out_num == 3:
            draw_params3 = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matchesMask3, # draw only inliers
                            flags = 2)

            final3 = cv2.drawMatches(cv_image_input,kp1,self.img3,self.kp3,good3,None,**draw_params3)

            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final3, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final3, "bgr8"))

        elif image_out_num == 4:
            draw_params4 = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matchesMask4, # draw only inliers
                            flags = 2)

            final4 = cv2.drawMatches(cv_image_input,kp1,self.img4,self.kp4,good4,None,**draw_params4)

            if self.pub_image_type == "compressed":
                # publishes traffic sign image in compressed type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final4, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes traffic sign image in raw type
                self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_imgmsg(final4, "bgr8"))

        # parking sign been detected, close detect_lane lane follow detect sign nodes and start cv_test node
            if image_out_num != 1:

                rospy.sleep(1)


                self.call_cv_test_node()
                self.call_lane_follow_node()                                    # shut down lane follow node
                self.call_detect_lane_node()                                    # shut down detect lane node
                self.sub_image_original.unregister()                            # shut down detect_sign node
                self.call_adjust_parking_node()                                 # turn on adjust_parking_node
                print('Parking sign has been detected, detect_sign node turned off')

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_sign')
    node = DetectSign()
    node.main()
