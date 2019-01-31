#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Manipulation():
    def __init__(self):

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

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






def main():

    rospy.init_node('manipulation_test')
    mani = Manipulation()
    mani.parking_test()
    rospy.loginfo('manipulation test begins!')

    while not rospy.is_shutdown():
        rospy.spin()
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
