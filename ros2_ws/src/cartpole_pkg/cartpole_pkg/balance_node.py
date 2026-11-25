#!/usr/bin/env python3

"""
Subscribes to /distance1 and /distance2 (Float64)
Publishes to /cmd_vel (Float64)

When pole tilts towards Distance1, wheels must have - velocity
When pole tilts towards Distance2, wheels must have + velocity
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

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

        # Cache variables initializes to None to track when data arrives
        self.__distance1 = None
        self.__distance2 = None

        # PID state variables
        self.__kp = 0.03
        self.__ki = 0.005
        self.__kd = 0.001
        self.__prev_error = None
        self.__sum_error = 0

    def __sync_distance1(self, msg):
        self.__distance1 = msg.data
        self.__process_data()

    def __sync_distance2(self, msg):
        self.__distance2 = msg.data
        self.__process_data()

    def __process_data(self):
        if self.__distance1 is not None and self.__distance2 is not None:
            # Synchronous logic here, must publish to /cmd_vel
            # Error = distance1 - distance2, so when pole tilts to distance2, velocity will be +
            vel_msg = Float64()
            error = self.__distance1 - self.__distance2
            if self.__prev_error is None:
                d_error = 0.0
            else:
                d_error = error - self.__prev_error

            self.__sum_error += error
            vel_msg.data = self.__kp * error + self.__ki * self.__sum_error + self.__kd * d_error

            if vel_msg.data > MAX_VEL:
                vel_msg.data = MAX_VEL
            elif vel_msg.data < -MAX_VEL:
                vel_msg.data = -MAX_VEL
                
            self.__prev_error = error
            self.__cmd_vel_publisher.publish(vel_msg)

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