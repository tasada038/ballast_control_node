# https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
# https://github.com/adafruit/Adafruit_Python_PCA9685

# ROS2 python library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class BallastPub(Node):

    def __init__(self):
        super().__init__('ballast_control_node')
        self.pub = self.create_publisher(
            Int32MultiArray,
            'ballast/data_raw',
            10)
        timer_period = 1.0  #  seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.data_array = [0]*3

    def timer_callback(self):

        try:
            ballast_id = input("id (ml=3, vl=7, mr=11, vr=15): ")
            ballast_duty = input("duty (0〜100): ")
            ballast_time = input("time (0〜10): ")

            msg = Int32MultiArray(data=self.data_array)
            msg.data[0] = int(ballast_id)
            msg.data[1] = int(ballast_duty)
            msg.data[2] = int(ballast_time)

            self.get_logger().info("Publish:: id: {0}, duty: {1}, time: {2}".format(msg.data[0], msg.data[1], msg.data[2])+"\n")
            self.pub.publish(msg)

        except ValueError:
            self.get_logger().info("ValueError: invalid literal for int()")

def main(args=None):
    rclpy.init(args=args)
    try:
        ballast_pub = BallastPub()
        rclpy.spin(ballast_pub)
    finally:
        ballast_pub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
