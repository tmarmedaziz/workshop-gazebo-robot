#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
import math

class OdomPublisher():

    def __init__(self):
        rospy.init_node("EncoderData", anonymous=True)
        self.odom_pub = rospy.Publisher("odom_data", Odometry, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.2
        self.vtheta = 0.1
        self.rate = rospy.Rate(30)
    
    def publish_odom(self):
        current_time = rospy.Time.now()
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"

        delta_t = 0.1

        self.x +=  delta_t * self.vx * math.cos(self.theta)
        self.y +=  delta_t * self.vx * math.sin(self.theta)

        self.theta = self.vtheta * delta_t

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.x = self.y

        quat = quaternion_from_euler(0,0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.angular.x = self.vtheta

        self.odom_pub.publish(odom_msg)


if __name__ == "__main__":
    try:
        odom = OdomPublisher()
        while not rospy.is_shutdown():
            odom.publish_odom()
            odom.rate.sleep()
    except rospy.ROSInterruptException:
        pass
        



