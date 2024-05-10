# Smart Walking Cane Embedded Software

## Overview

This README provides an overview of the embedded software developed for the Smart Walking Cane Capstone Project. The software was designed to run on an STM32L412C8T6 microcontroller.

<p align="center">
    <img src="readme_res\smart_walking_cane.png" alt="Smart Walking Cane" width="50%">
</p>

## Obstacle Detection System

The obstacle detection system utilizes ultrasonic sensors (HC-SR04) to detect obstacles in the cane's vicinity and provides feedback to the user through vibration motors integrated into the cane's handle. The system consists of:

### Frontal Detection Ultrasonic Sensor

- Detects obstacles closer to the ground.
- Vibration feedback intensity varies based on the distance of the obstacle:
  - If the obstacle's distance is within 1 meter: strong vibration.
  - If the obstacle's distance is between 1 and 2 meters: weaker vibration.
  - If the obstacle's distance is further than 2 meters: no vibration.

### Head Level Detection Ultrasonic Sensor

- Detects obstacles at head level.
- Vibration feedback is provided in pulses.

## LTE/GPS Functionality

The LTE/GPS module is responsible for the following:
- Obtains GPS location data
- Handles communication with a Python server:
  - Send GPS coordinates to the server
  - Receive requests to ping the cane

## Components

- STM32L412C8T6
- 2 x HC-SR04 Ultrasonic Sensors
- LTE IoT 9 Click Module by MICROE
- Vibration Motors
- Buzzer Module

## Companion App

A companion Android app was developed for visually impaired users and caretakers. The app enhances the functionality of the Smart Walking Cane [Link to repository](https://github.com/harri012/Hermes_App).

## References
- The HC-SR04 library used for this project is adapted from [HCSR04 library by Khaled-Magdy-DeepBlue](https://github.com/Khaled-Magdy-DeepBlue/STM32_Course_DeepBlue/tree/master/ECUAL/HCSR04).
- The code for the LTE IoT Click Module was adapted from the [LTE IoT 9 Click Repository](https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/lteiot9) provided within the MikroSDK 2.0 Click Repository.
