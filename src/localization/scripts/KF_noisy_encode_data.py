#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import numpy as np

# python3 -m pip install filterpy
from filterpy.kalman import KalmanFilter

import tf
from geometry_msgs.msg import TransformStamped

class KalmanFilterNode():

    def __init__(self):
        rospy.init_node("KalmanFilterNode", anonymous=True)
        self.odom_sub = rospy.Subscriber("noisy_odom_data", Odometry, self.filter_odom)
        self.filtered_odom_pub = rospy.Publisher("filtered_odom", Odometry, queue_size=10)

        # Kalman filter initialization
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.array([0., 0., 0., 0.])  # initial state (x, y, vx, vy)
        self.kf.F = np.array([[1., 0., 1., 0.],
                              [0., 1., 0., 1.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])  # state transition matrix: defines how the state evolves from one time step to the next.
        self.kf.H = np.array([[1., 0., 0., 0.],
                              [0., 1., 0., 0.]])  # measurement matrix: maps the true state of the system to the measured values
        self.kf.P *= 1000.  # covariance matrix uncertainty of the state estimate. It is updated at each step
        self.kf.R = np.array([[0.1, 0.],
                              [0., 0.1]])  # measurement noise  the uncertainty or noise in the measurements. It reflects the confidence in the measurements.
        self.kf.Q = np.eye(4) * 0.01  # process noise ncertainty in the process model. It accounts for errors due to approximations in the system model.
        self.tf_broadcaster = tf.TransformBroadcaster() 

    def filter_odom(self, odom_msg):
        z = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y
        ]) # measurements received from the system (the noisy odometry data)
        self.kf.predict()
        self.kf.update(z)
        filtered_msg = odom_msg
        filtered_msg.pose.pose.position.x = self.kf.x[0]
        filtered_msg.pose.pose.position.y = self.kf.x[1]
        filtered_msg.twist.twist.linear.x = self.kf.x[2]

        # filtered_msg.twist.twist.linear.y = self.kf.x[3]

        # filtered_msg.pose.covariance = [
        #     0.01, 0, 0, 0, 0, 0,
        #     0, 0.01, 0, 0, 0, 0,
        #     0, 0, 0.01, 0, 0, 0,
        #     0, 0, 0, 0.01, 0, 0,
        #     0, 0, 0, 0, 0.01, 0,
        #     0, 0, 0, 0, 0, 0.01
        # ]
        # filtered_msg.twist.covariance = [
        #     0.01, 0, 0, 0, 0, 0,
        #     0, 0.01, 0, 0, 0, 0,
        #     0, 0, 0.01, 0, 0, 0,
        #     0, 0, 0, 0.01, 0, 0,
        #     0, 0, 0, 0, 0.01, 0,
        #     0, 0, 0, 0, 0, 0.01
        # ]

        self.filtered_odom_pub.publish(filtered_msg)

        t = rospy.Time.now()

        translation = (0.2, 0.0, 0.0)  

        rotation = (0.0, 0.0, 0.0, 1.0)  

        self.tf_broadcaster.sendTransform(
            translation,       
            rotation,          
            t,                 
            "odom",        # Child frame
            "base_link"     # Parent frame   
        )

if __name__ == '__main__':
    try:
        kf_node = KalmanFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

