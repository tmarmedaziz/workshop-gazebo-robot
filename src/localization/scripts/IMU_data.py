#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import random
import math

class IMUData():

    def __init__(self):
        rospy.init_node("IMUPublisher", anonymous=True)
        self.imu_publisher = rospy.Publisher("imu_data", Imu, queue_size=10)
        self.rate = rospy.Rate(10)

    def publlish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.angular_velocity.x = 0.1 + random.gauss(0, 0.01)
        imu_msg.linear_acceleration.x = 0.2 + random.gauss(0, 0.01)

        self.imu_publisher.publish(imu_msg)

if __name__ == "__main__":
    try:
        IMU_data = IMUData()
        while not rospy.is_shutdown():
            IMU_data.publlish_imu()
            IMU_data.rate.sleep()
    except rospy.ROSInterruptException:
        pass
