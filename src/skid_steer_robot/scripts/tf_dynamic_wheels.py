#!/usr/bin/env python3
"""
Needs to be fetched from the motors encoder in the real robot!
"""

from math import sin, cos, pi

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import Quaternion

import tf

L = 0.34 # Track width (distance between the left and right wheels) in meters
R = 0.12 # Wheel radius in meters

total_theta_left = 0.0
total_theta_right = 0.0
last_time = None

def broadcast_wheel_transform(parent_frame, child_frame, x, y, z, angle):
    transform = TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame

    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z

    rotation = tf.transformations.quaternion_from_euler(0.0, angle, 0.0)
    quaternion_ = Quaternion()
    quaternion_.x = rotation[0]
    quaternion_.y = rotation[1]
    quaternion_.z = rotation[2]
    quaternion_.w = rotation[3]

    transform.transform.rotation = quaternion_

    br.sendTransform(transform)


def odom_callback(odom_msg):
    global total_theta_left, total_theta_right, last_time

    v = odom_msg.twist.twist.linear.x
    omega_z = odom_msg.twist.twist.angular.z

    current_time = rospy.get_time()

    if last_time is None:
        last_time = current_time
        return

    dt = current_time - last_time

    v_left = v - (L / 2.0) * omega_z
    v_right = v + (L / 2.0) * omega_z
    
    delta_theta_left = (v_left * dt) / R
    delta_theta_right = (v_right * dt) / R
    
    total_theta_left += delta_theta_left
    total_theta_right += delta_theta_right

    rospy.loginfo("Total Rotation (Left Wheel): {:.2f} radians".format(total_theta_left))
    rospy.loginfo("Total Rotation (Right Wheel): {:.2f} radians".format(total_theta_right))
   
    last_time = current_time

    broadcast_wheel_transform("base_link", "rear_left_wheel", -0.16, 0.16, 0.05, total_theta_left)
    broadcast_wheel_transform("base_link", "rear_right_wheel", -0.16, -0.16, 0.05, total_theta_right)
    broadcast_wheel_transform("base_link", "front_left_wheel", 0.16, 0.16, 0.05, total_theta_left)
    broadcast_wheel_transform("base_link", "front_right_wheel", 0.16, -0.16, 0.05, total_theta_right)


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