#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg import Header
import math
import random

class FakeGPSSimulator(Node):
    def __init__(self):
        super().__init__('fake_gps_imu_publisher')

        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # --- GPS ---
        gps_msg = NavSatFix()
        gps_msg.header.stamp = now
        gps_msg.header.frame_id = "gps_link"
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        gps_msg.latitude = 34.0622881 + random.uniform(-0.00001, 0.00001)
        gps_msg.longitude = -117.8171448 + random.uniform(-0.00001, 0.00001)
        gps_msg.altitude = 382.0 + random.uniform(-0.1, 0.1)
        gps_msg.position_covariance = [1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 1.0]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_pub.publish(gps_msg)

        # --- IMU ---
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu_link"

        # Simulate fixed orientation (facing east)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance = [0.01, 0.0, 0.0,
                                          0.0, 0.01, 0.0,
                                          0.0, 0.0, 0.01]
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeGPSSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
