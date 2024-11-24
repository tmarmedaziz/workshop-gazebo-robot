#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import random

class NoisyOdom():

    def __init__(self):
        rospy.init_node("NoisyDataEncoder", anonymous=True)
        self.odom_sub = rospy.Subscriber("odom_data", Odometry, self.add_noise)
        self.noisy_odom_pub = rospy.Publisher("noisy_odom_data", Odometry, queue_size=10)

    def add_noise(self, msg):
        noisy_msg = msg

        noisy_msg.pose.pose.position.x += random.gauss(0, 0.01)
        noisy_msg.pose.pose.position.y += random.gauss(0, 0.01)

        noisy_msg.twist.twist.linear.x = random.gauss(0, 0.01)
        noisy_msg.twist.twist.angular.x = random.gauss(0, 0.01)

        self.noisy_odom_pub.publish(noisy_msg)


if __name__ == "__main__":
    try:
        noisy_odom = NoisyOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
