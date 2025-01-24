#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def odom_callback(odom_msg):
    transform = TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"

    transform.transform.translation.x = odom_msg.pose.pose.position.x
    transform.transform.translation.y = odom_msg.pose.pose.position.y
    transform.transform.translation.z = odom_msg.pose.pose.position.z

    transform.transform.rotation = odom_msg.pose.pose.orientation

    br.sendTransform(transform)

def main():
    rospy.init_node('tf_dynamic_node')

    global br
    br = tf2_ros.TransformBroadcaster()

    rospy.Subscriber('/odom', Odometry, odom_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
