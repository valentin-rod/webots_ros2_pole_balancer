#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image


class CartpoleDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        rclpy.init(args=None)
        self.__node = rclpy.create_node('cartpole_driver')

        # Get motors
        self.__motor_fl = self.__robot.getDevice('wheel1')
        self.__motor_fr = self.__robot.getDevice('wheel2')
        self.__motor_rl = self.__robot.getDevice('wheel3')
        self.__motor_rr = self.__robot.getDevice('wheel4')
        self.__motors = [self.__motor_fl, self.__motor_fr, self.__motor_rl, self.__motor_rr]

        # Initialize motors
        for motor in self.__motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0)

        # Get sensors
        self.__distance1 = self.__robot.getDevice('Distance1')
        self.__distance2 = self.__robot.getDevice('Distance2')
        self.__camera = self.__robot.getDevice('Camera')

        # Enable sensors
        timestep = int(self.__robot.getBasicTimeStep())

        self.__distance1.enable(timestep)
        self.__distance2.enable(timestep)
        self.__camera.enable(timestep)

        # Create sensor publishers
        self.__distance1_publisher = self.__node.create_publisher(Float64, 'distance1', 10)
        self.__distance2_publisher = self.__node.create_publisher(Float64, 'distance2', 10)
        self.__camera_publisher = self.__node.create_publisher(Image, 'camera/image_raw', 10)
        
        self.__node.create_subscription(Float64, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, msg):
        speed = msg.data
        for motor in self.__motors:
            motor.setVelocity(speed)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec = 0)

        distance1_msg = Float64()
        distance1_msg.data = float(self.__distance1.getValue())
        self.__distance1_publisher.publish(distance1_msg)

        distance2_msg = Float64()
        distance2_msg.data = float(self.__distance2.getValue())
        self.__distance2_publisher.publish(distance2_msg)
