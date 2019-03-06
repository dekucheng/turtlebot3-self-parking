#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

class Manipulation():
    def __init__(self):

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.sub_info_marker = rospy.Subscriber('ar_pose_marker',AlvarMarkers,self.head_to_artag, queue_size = 1)

        rospy.on_shutdown(self.fnShutDown)


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

    def head_to_artag(self,data):
        rospy.loginfo("Subscribing the ar_pose")


        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        rospy.sleep(1)
        while(data.markers[0].pose.pose.position.y<-0.2 or data.markers[0].pose.pose.position.y>0.2):
            twist.angular.z = data.markers[0].pose.pose.position.y
            print(twist)
            self.pub_cmd_vel.publish(twist)
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        rospy.loginfo("angle correct")

        while(data.markers[0].pose.pose.position.x>0.1):
            twist.linear.x = 0.08
            print("start to move in x")
            self.pub_cmd_vel.publish(twist)

        twist.linear.x = 0
        self.pub_cmd_vel.publish(twist)

        rospy.loginfo("Have moved to artag!")


    '''
    def parking_test(self):

        rospy.sleep(1)
        twist = Twist()

        twist.linear.x = 0.2
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        rospy.sleep(2)

        twist.linear.x = 0
        twist.angular.z = np.pi/6
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(3)

        twist.linear.x = 0.1
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(2)

        twist.linear.x = 0
        twist.angular.z = np.pi/6
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(6)

        twist.linear.x = 0.1
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(2)

        twist.linear.x = 0
        twist.angular.z = np.pi/6
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(3)

        twist.linear.x = 0.2
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)
        rospy.sleep(3)
    '''

    def head_to_parkingslot(self):
        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist

        rospy.loginfo("Parking_Straight")

        Kp = 0.4
        Kd = 0.05

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()
        twist.linear.x = 0.07
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos





def main():

    rospy.init_node('manipulation_test')
    mani = Manipulation()
    #mani.parking_test()
    rospy.loginfo('manipulation test begins!')

    while not rospy.is_shutdown():
        rospy.spin()
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
