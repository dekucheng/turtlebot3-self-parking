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
import numpy as np

VERBOSE = True

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_1",CompressedImage,queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image/compressed",CompressedImage,self.callback)

    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE :
            print('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        #### Feature detectors using CV2 ####
        # "","Grid","Pyramid" +
        # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        '''
        cv2.imshow('cv_img', image_np)
        print(image_np.shape)
        cv2.waitKey(2)
        '''
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

        #self.subscriber.unregister()

def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
