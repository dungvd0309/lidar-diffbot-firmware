# lidar-diffbot-firmware

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Firmware](https://img.shields.io/badge/Firmware-repo-blue)](https://github.com/dungvd0309/lidar-diffbot-firmware)
[![ROS2_packages](https://img.shields.io/badge/ROS2_packages-repo-blue)](https://github.com/dungvd0309/lidar-diffbot-ros2)

ESP32 firmware for the low-level control of an autonomous differential-drive robot. Handles DC motor control via PWM, encoder feedback, PID closed-loop speed control, IMU reading, battery estimation, and UART communication with a Raspberry Pi 4 running ROS 2.

> **This is a two-repo project.** This repo contains the ESP32 firmware and hardware/wiring details. The ROS 2 software stack for high-level control (SLAM, Nav2, EKF, autonomous exploration) is in [lidar-diffbot-ros2](https://github.com/dungvd0309/lidar-diffbot-ros2).

<div align="center">
  <img width="400" height="400" alt="robot_pic" src="https://github.com/user-attachments/assets/11d7f795-ffd3-4c5b-8679-49c54d89d182"/>

  *Demo video on [YouTube](https://youtu.be/oauRnWwvWOY?t=132)*
</div>

## Table of Contents
- [1. Key features](#1-key-features)
- [2. System Architecture](#2-system-architecture)
- [3. Parts list](#3-parts-list)
- [4. Wiring diagram](#4-wiring-diagram)

## 1. Key features
- Reads wheel encoders via external interrupts and computes wheel speed.
- Closed-loop PID speed control for both DC motors, driven via PWM through an L298N H-bridge.
- Reads orientation data BNO055 IMU over I2C.
- Estimates battery voltage over ADC with EMA filtering.
- Streams encoder, IMU, and battery data to the Raspberry Pi 4 over UART, and receives velocity commands in return.

## 2. System Architecture
<div align="center">
  <img width="1753" height="897" alt="system-architecture" src="https://github.com/user-attachments/assets/765602cf-2603-46f9-8406-0e83ca6a2ade" />
  <img width="951" height="353" alt="data_flow" src="https://github.com/user-attachments/assets/7f0a3fa2-ef12-4424-b410-1948c214ab5d" />
</div>

## 3. Parts list
- ESP32 DevKit
- Raspberry Pi 4 
- YDLIDAR X3 LiDAR
- BNO055 IMU
- 2x JGA25 DC motors with encoders
- 2x 65mm blue wheels
- 12V battery pack (3S)
- DC charger port
- Latching push button
- 3.3 kΩ resistor
- 10 kΩ resistor
- 0.1 µF capacitor
- Custom 3D printed robot frame

## 4. Wiring diagram
<div align="center">
  <img width="1430" height="1180" alt="wiring_diagram" src="https://github.com/user-attachments/assets/ea72f541-895a-4c52-950e-067906d1606e" />
  <img width="693" height="471" alt="pcb" src="https://github.com/user-attachments/assets/4cb2fd00-816b-4ad8-8173-a4b666763ffd" />
  <img width="1213" height="910" alt="robot_internal" src="https://github.com/user-attachments/assets/8c836d1f-11f3-4daf-bd23-9768665c3417" />
</div>

## Acknowledgements
- [PID_Timed](https://github.com/kaiaai/arduino_pid_timed)
- [micro-ROS-arduino](https://github.com/micro-ROS/micro_ros_arduino)
