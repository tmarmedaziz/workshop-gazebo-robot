#!/usr/bin/env python3

from math import sin, cos, pi

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import Quaternion

def odom_callback(odom_msg):
    transform = TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "rear_left_wheel"

    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0     

    quaternion_ = Quaternion()
    quaternion_.x = 0.0
    quaternion_.y = sin(pi / 4)
    quaternion_.z = 0.0
    quaternion_.w = cos(pi / 4)

    transform.transform.rotation = quaternion_

    br.sendTransform(transform)

def main():
    rospy.init_node('tf_dynamic_node_wheels_frames')

    global br
    br = tf2_ros.TransformBroadcaster()

    rospy.Subscriber('/odom', Odometry, odom_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass