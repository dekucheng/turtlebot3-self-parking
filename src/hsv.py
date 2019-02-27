#!/usr/bin/env python
import cv2
import numpy as np

image = cv2.imread('parking.jpg')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_red = np.array([22,85,200])
upper_red = np.array([50,255,220])

mask = cv2.inRange(hsv, lower_red, upper_red)
res = cv2.bitwise_and(image,image,mask=mask)


cv2.imshow('image',image)
cv2.imshow('mask',mask)
cv2.imshow('res',res)
cv2.waitKey(0)


cv2.destroyAllWindows()
