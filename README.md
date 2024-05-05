# Smart Walking Cane Embedded Software

## Overview

This README provides an overview of the embedded software developed for the Smart Walking Cane capstone project. The software was designed to run on an STM32L412C8T6 microcontroller. The system's functionality is divided into two main parts: obstacle detection and LTE/GPS functionality.

<div style="text-align:center;">
    <img src="readme_res\smart_walking_cane.png" alt="Smart Walking Cane" width="50%">
</div>

## Obstacle Detection System

The obstacle detection system utilizes ultrasonic sensors (HC-SR04) to detect obstacles in the cane's vicinity and provides feedback to the user through vibration motors integrated into the cane's handle. The system consists of:

### Frontal Detection Ultrasonic Sensor

- Detects obstacles closer to the ground.
- Vibration feedback intensity varies based on the distance of the obstacle:
  - If obstacle distance is within 1 meter: strong vibration.
  - If obstacle distance is between 1 and 2 meters: weaker vibration.
  - If obstacle distance is further than 2 meters: no vibration.

### Head Level Detection Ultrasonic Sensor

- Detects obstacles at head level.
- Vibration feedback is provided in pulses to differentiate from vibrations triggered by the frontal sensor.

## LTE/GPS Functionality

The LTE/GPS module is responsible for the following:
- Obtains GPS location data
- Handles communication with a python server:
  - Send GPS coordinates to the server
  - Receive requests to ping the cane

## Components

- STM32L412C8T6
- 2 x HC-SR04 Ultrasonic Sensors
- LTE IoT 9 Click Module by MICROE
- Vibration Motors
- Buzzer Module

## References
- The HC-SR04 library used for this project is adapted from [HCSR04 library by Khaled-Magdy-DeepBlue](https://github.com/Khaled-Magdy-DeepBlue/STM32_Course_DeepBlue/tree/master/ECUAL/HCSR04).
- The code for the LTE IoT Click Module was adapted from the [LTE IoT 9 Click Repository](https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/lteiot9) provided within the MikroSDK 2.0 Click Repository.