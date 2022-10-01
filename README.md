# ballast_ros2_node
A ROS2 package that controls the ballast with a joystick 

## Requirements
- Adafruit Servo Bonnet for Raspberry Pi
  - [Web shop](https://www.switch-science.com/products/3773?_pos=1&_sid=fa1ed2bd2&_ss=r)
- Air pump
- Linux OS
  - Ubuntu 20.04
- ROS
  - Foxy Fizroy

## Installation
```
sudo apt install ros-foxy-joy-linux
sudo apt install ros-foxy-joy-linux-dbgsym
sudo pip3 install adafruit-pca9685
```

## Usage
Open two shells.
In the first shell, publish Joy msg
```
ros2 run joy_linux joy_linux_node
```

In the second shell, run the ballast node:
```
ros2 run ballast_control_node joy_to_topballast
```

## Published Topics

std_msgs.msg Float32: /ballast/top_rate

## License
This repository is licensed under the MIT license, see LICENSE.
