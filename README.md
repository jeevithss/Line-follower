# ESP32-Based PID Line Follower Robot

## Project Overview

This project implements an autonomous line follower robot using an ESP32 microcontroller and an 8-channel IR sensor array. The robot detects a line on the surface and continuously adjusts its motor speeds using a PID (Proportional–Integral–Derivative) control algorithm to follow the path.

A TB6612FNG dual motor driver controls the two DC motors through PWM signals, enabling differential steering and smooth directional correction.

The robot also features Bluetooth-based control, allowing the user to start or stop the robot, monitor sensor readings, and adjust PID parameters and motor speeds in real time.

## Key Features

- **8-Channel Line Detection:** Uses an array of eight IR sensors to detect the line position.
- **PID-Based Control:** Calculates steering corrections based on the detected line position.
- **Differential Motor Control:** Independently controls left and right motor speeds using PWM.
- **Bluetooth Communication:** Supports wireless commands for robot control and parameter tuning.
- **Right-Priority Junction Handling:** Detects right-side branches and prioritizes right turns at junctions.
- **Line-Loss Recovery:** Uses the last detected line direction to recover when the robot loses the track.

## Components Used

| S.No. | Component | Quantity | Purpose |
|---|---|---:|---|
| 1 | ESP32 Development Board | 1 | Main controller and Bluetooth communication |
| 2 | 8-Channel IR Sensor Array | 1 | Line detection and position sensing |
| 3 | TB6612FNG Motor Driver | 1 | Controls the direction and speed of DC motors |
| 4 | DC Geared Motors | 2 | Provides movement and steering |
| 5 | Robot Chassis | 1 | Mechanical frame for mounting components |
| 6 | Wheels | 2 | Provides traction and movement |
| 7 | Battery / Power Supply | 1 | Supplies power to the robot |
| 8 | Jumper Wires | As required | Electrical connections |

## Hardware Connections

### IR Sensor Array

| Sensor | ESP32 GPIO |
|---|---:|
| IR1 | GPIO 36 |
| IR2 | GPIO 39 |
| IR3 | GPIO 34 |
| IR4 | GPIO 35 |
| IR5 | GPIO 32 |
| IR6 | GPIO 33 |
| IR7 | GPIO 25 |
| IR8 | GPIO 26 |

### TB6612FNG Motor Driver

| Motor Driver Pin | ESP32 GPIO |
|---|---:|
| AIN1 | GPIO 12 |
| AIN2 | GPIO 27 |
| PWMA | GPIO 14 |
| BIN1 | GPIO 23 |
| BIN2 | GPIO 22 |
| PWMB | GPIO 13 |
| STBY | GPIO 4 |

*Note: Motor power and logic power connections must follow the TB6612FNG datasheet and the voltage requirements of the selected motors and ESP32 board.*

## Working Principle

The eight IR sensors continuously detect the line position. The sensor readings are processed using predefined weights to calculate the position error relative to the center of the robot.

The PID controller uses this error to calculate a correction value. Based on the correction, the ESP32 adjusts the PWM signals supplied to the left and right motors through the TB6612FNG motor driver.

This differential speed control allows the robot to correct its direction and follow the line.

When the robot encounters a junction, the right-priority logic checks the right-side and center sensor readings before initiating a right turn. If the line is lost, the recovery algorithm uses the previously detected direction to search for the line again.

## PID Control

The robot uses three PID parameters:

- **Proportional (Kp):** Determines the correction based on the current line-position error.
- **Integral (Ki):** Accumulates previous errors to compensate for persistent deviations.
- **Derivative (Kd):** Responds to changes in error to reduce oscillations.

The PID output is used to adjust the motor speeds dynamically.

## Bluetooth Control

The ESP32 uses Bluetooth Classic through the `BluetoothSerial` library. The robot can be controlled and configured wirelessly using a Bluetooth terminal application.

| Command | Function |
|---|---|
| START | Starts line following |
| STOP | Stops the motors |
| STATUS | Displays the current robot configuration |
| SENSORS | Displays all eight sensor readings |
| KP:value | Adjusts the proportional gain |
| KI:value | Adjusts the integral gain |
| KD:value | Adjusts the derivative gain |
| BASE:value | Changes the base motor speed |
| TURN:value | Changes the turning speed |
| RECOVERY:value | Changes the line recovery speed |

Example: `KP:30` sets the proportional gain to 30.

The Bluetooth device name is **LineBot**.
