#!/usr/bin/env python
from __future__ import print_function

import sys, time
import rospy
import cv2
import roslib
from scipy.ndimage import filters
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from turtlebot3_selfparking.srv import*
from std_srvs.srv import Trigger
import numpy as np

VERBOSE = True

class image_receiver():

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_1",CompressedImage,queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_rect_color/compressed",CompressedImage,self.callback)
        self.call_init_image = rospy.ServiceProxy('cv_test/provide_rectified_image', Trigger)
        rospy.Service('cv_test/provide_rectified_image',Trigger,self.svc_set_init_image)
        self.image_np = CompressedImage()

    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE :
            print('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        self.image_np = ros_data.data

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


        #self.subscriber.unregister()
    def svc_set_init_image(self, data):

        np_arr = np.fromstring(self.image_np, np.uint8)
        init_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        print(init_image.shape)
        rospy.sleep(1)
        cv2.imshow('cv_img', init_image)

        cv2.imwrite('parking_test_02.jpg',init_image)

        cv2.waitKey(0)


        #if rospy.ROSInterruptException:
        #    cv2.destroyAllWindows()
        #    print("cv_shutting_down")

        return (True,"Has recerived the image")

def main():
  rospy.init_node('cv_test')

  ic = image_receiver()
  rospy.sleep(1)
  ic.call_init_image()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
