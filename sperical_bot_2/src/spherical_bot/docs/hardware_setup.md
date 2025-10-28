# Spherical Bot - Hardware Setup Guide

## Overview

This document describes the hardware setup for the Spherical Bot - a self-balancing robot with Raspberry Pi Zero 2W, IMU, TOF camera, and cloud connectivity.

## Bill of Materials

### Core Components
- **Main Controller**: Raspberry Pi Zero 2W
- **IMU**: ICM-20948 9-DOF Motion Sensor
- **TOF Camera**: Arducam Time-of-Flight Camera
- **Motor Driver**: TB6612FNG Dual Motor Driver
- **Motors**: N20 Gear Motors with Encoders (1000 CPR)
- **Battery**: 3S LiPo 2200mAh
- **Voltage Regulator**: Buck Converter 5V/3A

### Additional Components
- **Wheels**: 70mm Diameter
- **Chassis**: Custom 3D Printed
- **Cooling**: 30mm Fan + Heatsink
- **Status LED**: WS2812B RGB LED
- **Emergency Stop Button**: Tactile Switch

## Wiring Diagram

### GPIO Pin Assignment

| Component | GPIO Pin | Physical Pin | Function |
|-----------|----------|--------------|----------|
| Motor A IN1 | GPIO17 | 11 | Left Motor Direction |
| Motor A IN2 | GPIO27 | 13 | Left Motor Direction |
| Motor A PWM | GPIO13 | 33 | Left Motor Speed |
| Motor B IN1 | GPIO22 | 15 | Right Motor Direction |
| Motor B IN2 | GPIO23 | 16 | Right Motor Direction |
| Motor B PWM | GPIO18 | 12 | Right Motor Speed |
| Fan Control | GPIO19 | 35 | Cooling Fan |
| Status LED | GPIO25 | 22 | System Status |
| Emergency Stop | GPIO4 | 7 | Emergency Button |
| Left Encoder A | GPIO5 | 29 | Left Encoder Channel A |
| Left Encoder B | GPIO6 | 31 | Left Encoder Channel B |
| Right Encoder A | GPIO12 | 32 | Right Encoder Channel A |
| Right Encoder B | GPIO16 | 36 | Right Encoder Channel B |

### I2C Devices
- **IMU (ICM-20948)**: Address 0x68
- **Power Monitor**: Address 0x40 (if used)

### Power Distribution