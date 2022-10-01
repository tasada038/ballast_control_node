# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
# https://github.com/adafruit/Adafruit_Python_PCA9685

# ROS2 python library
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

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
        self.sub_joy = self.create_subscription(
            Joy,
            'joy',
            self.ballast_node_callback,
            10,
        )
        self.sub_joy  # prevent unused variable warning

        # Create publisher(s)  
        self.pub_string = self.create_publisher(Float32, '/ballast/btm_rate', 10)
        self.ballast_rate = Float32()
        self.ballast_rate.data = 0.0

        self.motor_dict = {
            "right_motor": 3, 
            "right_valve": 7
        }

        self.df = pd.DataFrame(
            data = {'id' : list(self.motor_dict.values()),
                    'button': ["A", "B"],
                    'ballast': ["bottom", "bottom"],
            },
            index = list(self.motor_dict.keys())
        )

        self.ballast_id, self.ballast_duty, self.ballast_time = 0, 0, 0.0
        self.left_ballast_max, self.right_ballast_max = 6.0, 6.0
        self.left_ballast_rate, self.right_ballast_rate = 0.0, 0.0
        self.ballast = ControlMotors()
        self.get_logger().info("{}".format(self.df.T)+"\n")

    def ballast_node_callback(self, joy_msg):
        self.A_button = joy_msg.buttons[1]
        self.B_button = joy_msg.buttons[2]
        self.RB_button = joy_msg.buttons[5]
        self.RT_button = joy_msg.buttons[7]

        if self.RT_button == 1 and self.RB_button == 1:
            self.ballast_control(self.A_button, self.B_button, 0.5)
        elif self.RT_button == 1 and self.RB_button == 0:
            self.ballast_control(self.A_button, self.B_button, 1.0)
        elif self.RT_button == 0 and self.RB_button == 1:
            self.ballast_control(self.A_button, self.B_button, 2.0)
        elif self.RT_button == 0 and self.RB_button == 0:
            self.ballast_control(self.A_button, self.B_button, 4.0)

    def ballast_control(self, a, b, ballast_time):

        if a == 1:
            self.ballast_id = self.motor_dict["right_motor"]
            self.ballast_duty, self.ballast_time = 100, ballast_time
            self.right_ballast_rate += self.ballast_time
            if self.right_ballast_rate >= 0.0 and self.right_ballast_rate <= self.right_ballast_max:
                self.get_logger().info("button A, Bottom Rmotor ON, R ballast rate: {}".format(self.right_ballast_rate)+"\n")
                # publish ballast rate data
                self.ballast_rate.data = self.right_ballast_rate
                self.pub_string.publish(self.ballast_rate)
                # control pump                
                self.ballast.pwm_control(self.ballast_id, self.ballast_duty, self.ballast_time)
                self.get_logger().info("Complete Bottom Rmotor On")
            else:
                self.right_ballast_rate -= self.ballast_time
                self.get_logger().info("The air ratio exceeds the maximum value, left ballast rate: {}".format(self.left_ballast_rate)+"\n")
    
        elif b == 1:
            self.ballast_id = self.motor_dict["right_valve"]
            self.ballast_duty, self.ballast_time = 100, ballast_time
            self.right_ballast_rate -= self.ballast_time
            if self.right_ballast_rate < 0.0:
                self.right_ballast_rate = 0.0
            self.get_logger().info("button B, Bottom Rvalve ON, R ballast rate: {}".format(self.right_ballast_rate)+"\n")
            # publish ballast rate data
            self.ballast_rate.data = self.right_ballast_rate
            self.pub_string.publish(self.ballast_rate)
            # control pump
            self.ballast.pwm_control(self.ballast_id, self.ballast_duty, self.ballast_time)
            self.get_logger().info("Complete Bottom Rvalve On")

        else:
            pass


class ControlMotors(object):
    def __init__(self):

        print("Preparing servo kit :ballast_node")  # debag

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.hat = adafruit_pca9685.PCA9685(self.i2c,address=0x40)
        self.hat.frequency = 100

        print("Servo kit ready! :ballast_node")  # debag

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
        time.sleep(0.1)


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