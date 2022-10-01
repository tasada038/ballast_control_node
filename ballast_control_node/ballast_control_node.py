# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
# https://github.com/adafruit/Adafruit_Python_PCA9685

# ROS2 python library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

# Adafruit library
import board
import busio
import adafruit_pca9685

# Standard library
import time
import pandas as pd
import numpy as np


class BallastNode(Node):

    def __init__(self):
        super().__init__('ballast_control_node')
        self.sub = self.create_subscription(
            Int32MultiArray,
            'ballast/data_raw',
            self.ballast_node_callback,
            10)
        self.sub


        self.motor_dict = {
            "left_motor": 3, 
            "left_valve": 7, 
            "right_motor": 11, 
            "right_valve": 15
        }

        self.df = pd.DataFrame(
            data = {'id' : list(self.motor_dict.values())},
            index = list(self.motor_dict.keys())
        )

        self.left_ballast_max = 8
        self.left_ballast_rate = 0
        self.right_ballast_max = 8
        self.right_ballast_rate = 0
        self.ballast = ControlMotors()
        print("{}".format(self.df.T)+"\n")

    def ballast_node_callback(self, msg):
        self.ballast_id = msg.data[0]
        self.ballast_duty = msg.data[1]
        self.ballast_time = msg.data[2]
        
        if self.ballast_id == self.motor_dict["left_motor"]:
            self.left_ballast_rate += self.ballast_time
            self.get_logger().info("id: {0}, duty: {1}, time: {2}".format(self.ballast_id, self.ballast_duty, self.ballast_time))
            if self.left_ballast_rate <= self.left_ballast_max:
                print("left motor ON, left ballast rate: {}".format(self.left_ballast_rate)+"\n")
                self.ballast.pwm_control(self.ballast_id, self.ballast_duty, self.ballast_time)
            else:
                self.left_ballast_rate -= self.ballast_time
                print("The air ratio exceeds the maximum value, left ballast rate: {}".format(self.left_ballast_rate)+"\n")

        elif self.ballast_id == self.motor_dict["left_valve"]:
            self.left_ballast_rate -= self.ballast_time
            self.get_logger().info("id: {0}, duty: {1}, time: {2}".format(self.ballast_id, self.ballast_duty, self.ballast_time))
            if self.left_ballast_rate < 0:
                self.left_ballast_rate = 0
            self.ballast.pwm_control(self.ballast_id, self.ballast_duty, self.ballast_time)
            print("left valve ON, left ballast rate: {}".format(self.left_ballast_rate)+"\n")

        elif self.ballast_id == self.motor_dict["right_motor"]:
            self.right_ballast_rate += self.ballast_time
            self.get_logger().info("id: {0}, duty: {1}, time: {2}".format(self.ballast_id, self.ballast_duty, self.ballast_time))
            if self.right_ballast_rate <= self.right_ballast_max:
                print("right motor ON, right ballast rate: {}".format(self.right_ballast_rate)+"\n")
                self.ballast.pwm_control(self.ballast_id, self.ballast_duty, self.ballast_time)
            else:
                self.left_ballast_rate -= self.ballast_time
                print("The air ratio exceeds the maximum value, left ballast rate: {}".format(self.left_ballast_rate)+"\n")
 
        elif self.ballast_id == self.motor_dict["right_valve"]:
            self.right_ballast_rate -= self.ballast_time
            self.get_logger().info("id: {0}, duty: {1}, time: {2}".format(self.ballast_id, self.ballast_duty, self.ballast_time))
            if self.right_ballast_rate < 0:
                self.right_ballast_rate = 0
            self.ballast.pwm_control(self.ballast_id, self.ballast_duty, self.ballast_time)
            print("right valve ON, right ballast rate: {}".format(self.right_ballast_rate)+"\n")

        else:
            print("The id specified in the data frame is not registered")


class ControlMotors(object):
    def __init__(self):

        print("Preparing servo kit")  # debag

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.hat = adafruit_pca9685.PCA9685(self.i2c,address=0x40)
        self.hat.frequency = 100

        print("Servo kit ready!")  # debag

    def pwm_control(self, channels, duty, timer):
        # Default adafruit duty value
        # 0xffff = 65535 is Max
        # pump value is 30000 to 0xffff
        # valve value is 65000 to 0xffff

        self.pwm = self.hat.channels[channels]
        self.pwm.duty_cycle = duty * int(65535/100)
        time.sleep(timer)

        # pwm control Stop
        self.pwm.duty_cycle = 0
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    try:
        ballast_node = BallastNode()
        rclpy.spin(ballast_node)
    finally:
        ballast_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()