
# Project Manas: STM32 Communication ROS Package

Ros package designed to facilitate communication between a ROS system and an STM32 microcontroller board. This package enables seamless data exchange and control commands between ROS nodes and the STM32 board, enabling integration of the microcontroller into your ROS-based robotic system or project.


## Features

- Subscribe to /cmd_vel Topic: Send RPM commands to the STM32 board based on velocity commands received from the /cmd_vel topic.
- Subscribe to /joy Topic: Send RPM commands to the STM32 board based on joystick inputs from the /joy topic.
- Receive Encoder Values: Receive encoder values from the STM32 board, plot them for PID tuning, and adjust PID parameters (kp, ki, kd) directly from the package.


## Credits
This package utilizes the `compy` library, which is prebuilt and included within this package. Special thanks to Sachin for developing compy, which provides essential functionality for this project.

If you’re interested in implementing this in ROS2, there is a branch named `ros2` available. You can find it [here](https://github.com/Karthikeya819/manas_expo_nova/tree/ros2).
