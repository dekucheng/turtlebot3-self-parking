#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from turtlebot3_selfparking.srv import*

class Navigate():
    def __init__(self):
        self.theta = 0.34907
        self.mid_x = 0
        self.mid_y = 0
        self.ref_x = 0
        self.ref_y = 0
        self.last_angleError = 0
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.cam_mat = np.array([[240.578818, 0.000000, 314.581041, 0],
                                 [0.000000, 238.581041, 218.404197, 0],
                                 [0,           0,         1,        0]])
        self.pose_mat = np.array([[1, 0,              0,              0   ],
                                  [0, np.cos(self.theta), np.sin(self.theta), 0.12],
                                  [0, -np.sin(self.theta),np.cos(self.theta), 0   ],
                                  [0, 0,              0,              1   ]])


        # stop the turtlebot when shut down the window
        rospy.on_shutdown(self.fnShutDown)
        #rospy.Service('navigation/localize_mid_point',GetPointLocation,self.svc_get_point_location)
        rospy.wait_for_service('cv_test/get_mid_location')
        self.call_get_mid_location = rospy.ServiceProxy('cv_test/get_mid_location', GetPointLocation)


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
    def get_parking_rotateangle(self, z1, z2, x1, x2):
        dz = z1-z2
        dx = x1-x2
        theta = np.arctan(dz/dx)
        return theta

    def fake_go_parking(self, xw, zw, theta):

        self.rotate_angle = np.arctan(xw/zw)
        self.straight = np.sqrt(xw**2+zw**2)
        self.parking_rotateangle = -self.rotate_angle - theta


        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        dt = np.abs(self.rotate_angle/0.5)
        twist.angular.z = (np.abs(self.rotate_angle)/self.rotate_angle) * 0.5
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(dt)
        self.fnShutDown()
        rospy.sleep(1)

        dt = 1
        twist.angular.z = 0
        twist.linear.x = 0.1
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(dt)
        self.fnShutDown()
        rospy.sleep(1)

    def go_parking(self, xw, zw, theta):

        self.rotate_angle = np.arctan(xw/zw)
        self.straight = np.sqrt(xw**2+zw**2)
        self.parking_rotateangle = -self.rotate_angle - theta


        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        dt = np.abs(self.rotate_angle/0.5)
        twist.angular.z = (np.abs(self.rotate_angle)/self.rotate_angle) * 0.5
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(dt)
        self.fnShutDown()
        rospy.sleep(1)

        dt = self.straight/0.1
        twist.angular.z = 0
        twist.linear.x = 0.1
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(dt)
        self.fnShutDown()
        rospy.sleep(2)

        self.get_adjust_angle = rospy.Subscriber('adjust_parking/adjust_angle', Float64, self.adjust_parking)

        print('Finished!')
    def go_to_spot(self):

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        twist.linear.x = 0.15
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(0.8)
        self.fnShutDown()
        rospy.sleep(1)

        twist.angular.z = np.pi/3
        twist.linear.x = 0
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(3)
        self.fnShutDown()

    def adjust_parking(self, data):
        print('ADJUST!!')
        current_angle = data.data
        print(current_angle)
        ref_angle = 1.5708
        error = ref_angle - current_angle

        Kp = 0.9
        Kd = 0.6

        angular_z = Kp * error + Kd * (error - self.last_angleError)
        self.last_angleError = error
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = max(angular_z, -0.4) if angular_z < 0 else min(angular_z, 0.4)
        print(twist.angular.z)
        self.pub_cmd_vel.publish(twist)
        print(error)
        if np.abs(error) < 0.03:
            self.get_adjust_angle.unregister()
            print('Get to nice position!')
            self.go_to_spot()
    def get_world_location(self,x,y):
        u = 640 - x
        v = y
        print('pixel value is ',y)
        kf = self.cam_mat[0][0]
        lf = self.cam_mat[1][1]
        u0 = self.cam_mat[0][2]
        v0 = self.cam_mat[1][2]
        cos = np.cos(self.theta)
        sin = np.sin(self.theta)
        #yc = 0.12

        #zc = (lf*yc)/(v-v0)
        zc = 0.12*lf/(v0*cos-lf*sin-v*cos)
        print(v0*cos-lf*sin-v*cos)
        yc = (-zc*sin-0.12)/cos
        xc = (u*zc-u0*zc)/kf
        pca = np.array([xc,yc,zc,1]).T
        print('pca=',pca)

        #Lc = np.sqrt(yc**2 + zc**2)
        #afa = np.arcsin(np.abs(yc)/np.abs(Lc))
        #lc = np.abs(yc)/np.sin(self.theta+afa)
        #ycr = lc/Lc * yc
        #xcr = lc/Lc * xc
        #zcr = lc/Lc * zc
        #pcr = np.array([xcr,ycr,zcr,1]).T
        psc = np.dot(self.pose_mat,pca)
        # rotate axis to ros frame
        xw = -psc[0]
        yw = psc[1]    # ==0 by default
        zw = -psc[2]
        print('psc=',psc,zw,xw)

        return xw,zw
    def move_forward(self):

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(1.3)
        self.fnShutDown()



def main():

    rospy.init_node('navigation')

    navi = Navigate()
    rospy.loginfo('navigation test begins!')
    rospy.sleep(1)
    #navi.mid_x, navi.mid_y =


    mid_xy = navi.call_get_mid_location()
    while mid_xy.x == 0 and mid_xy.y == 0:
        mid_xy = navi.call_get_mid_location()
        rospy.sleep(1)
        #navi.move_forward()
        rospy.loginfo('No valid location get, ready for next calling')
    navi.mid_x = int(mid_xy.x)
    navi.mid_y = int(mid_xy.y)
    navi.ref_x = int(mid_xy.refx)
    navi.ref_y = int(mid_xy.refy)
        #navi.mid_x = 306
        #navi.mid_y = 230
    park_x, park_z = navi.get_world_location(navi.mid_x, navi.mid_y)
    '''
    while (park_z < 0) & (not rospy.is_shutdown()):

        mid_xy = navi.call_get_mid_location()
        navi.mid_x = int(mid_xy.x)
        navi.mid_y = int(mid_xy.y)
        navi.ref_x = int(mid_xy.refx)
        navi.ref_y = int(mid_xy.refy)

        park_ref_x,park_ref_z = navi.get_world_location(navi.ref_x, navi.ref_y)
        parkingangle = navi.get_parking_rotateangle(park_x, park_z, park_ref_x, park_ref_z)
        navi.fake_go_parking(park_x, park_z, parkingangle)
        park_x, park_z = navi.get_world_location(navi.mid_x, navi.mid_y)
        print(park_z)
        print('The parking location is not valid, try to go further to get valid location!')

    mid_xy = navi.call_get_mid_location()
    navi.mid_x = int(mid_xy.x)
    navi.mid_y = int(mid_xy.y)
    navi.ref_x = int(mid_xy.refx)
    navi.ref_y = int(mid_xy.refy)
    '''
    park_ref_x,park_ref_z = navi.get_world_location(navi.ref_x, navi.ref_y)
    parkingangle = navi.get_parking_rotateangle(park_x, park_z, park_ref_x, park_ref_z)
    navi.go_parking(park_x, park_z, parkingangle)

    rospy.sleep(1)
    print('The coordinate of parking point in image frame is (%d, %d)' %(navi.mid_x,navi.mid_y))

    #print(navi.mid_x, navi.mid_y)
    #mani.parking_test()
    while not rospy.is_shutdown():
        rospy.spin()
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
