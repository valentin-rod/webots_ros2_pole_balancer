#!/usr/bin/env python3

"""
Subscribes to /distance1 and /distance2 (Float64)
Publishes to /cmd_vel (Float64)

When pole tilts towards Distance1, wheels must have - velocity
When pole tilts towards Distance2, wheels must have + velocity
"""

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64

import os

#Target distance for both sensors; Set Point
TARGET_DISTANCE = 1000

# Limits
MAX_VEL = 10.0

class BalanceNode(Node):

    def __init__(self):
        super().__init__("balance_node")
        self.__distance1_subscriber = self.create_subscription(Float64, 'distance1', self.__sync_distance1, 10)
        self.__distance2_subscriber = self.create_subscription(Float64, 'distance2', self.__sync_distance2, 10)
        self.__cmd_vel_publisher = self.create_publisher(Float64, 'cmd_vel', 1)

        # Create controller
        self.__controller = PID()

        # Cache variables initializes to None to track when data arrives
        self.__distance1 = None
        self.__distance2 = None

        # Flags to ensure self.__process_data runs once per run
        self.__has_d1 = False
        self.__has_d2 = False

    def __sync_distance1(self, msg):
        self.__distance1 = msg.data
        self.__has_d1 = True
        self.__try_process_data()

    def __sync_distance2(self, msg):
        self.__distance2 = msg.data
        self.__has_d2 = True
        self.__try_process_data()

    def __try_process_data(self):
        if self.__has_d1 and self.__has_d2:
            self.__has_d1 = False
            self.__has_d2 = False
            self.__process_data()

    def __process_data(self):
        if self.__distance1 is not None and self.__distance2 is not None:
            # When pole tilts to distance2, velocity must be +
            # When pole tilts to distance1, velocity must be -
            if self.__distance1 < 1000.0:
                degrees = 180 - math.degrees(self.__distance1/1000.0)
            elif self.__distance2 < 10000.0:
                degrees = math.degrees(self.__distance2/1000.0)
            else:
                degrees = 90
                
            vel_msg = Float64()
            vel_msg.data = self.__controller(set_point=90, measured_value=degrees)
            vel_msg.data = max(-10.0, min(10.0, vel_msg.data))
            self.__cmd_vel_publisher.publish(vel_msg)

# Copyright © 2026 Alberto Diaz y Guillermo Iglesias. UPM. https://github.com/blazaid/Robotica/blob/main/Notebooks/3.1.%20Controladores%20PID%20y%20borroso%20para%20el%20mantenimiento%20de%20la%20ventilaci%C3%B3n%20en%20un%20refugio.ipynb
class PID:
    def __init__(self, kp=0.9, ki=0.001, kd=0.002):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.sum_error = 0

    def __call__(self, set_point, measured_value):
        error = set_point - measured_value
        self.sum_error += error
        d_error = error - self.prev_error
        output = self.kp * error + self.ki * self.sum_error + self.kd * d_error
        self.prev_error = error
        return output

def main(args=None):
    rclpy.init(args=args)
    node = BalanceNode()
    try:
        rclpy.spin(node)
    finally: 
        stop_msg = Float64()
        stop_msg.data = 0.0
        node.create_publisher(Float64, '/cmd_vel', 1).publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()