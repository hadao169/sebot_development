# Servo & Sensor Integration with Arduino and ROS 2

## Overview

This project demonstrates controlling a **servo motor** with an **ultrasonic distance sensor** using **Arduino** and integrating it with **ROS 2** for obstacle detection and avoidance. The servo continuously sweeps between 0° and 180°, while ROS 2 nodes read sensor data and send movement commands.

---

## Arduino Setup

### 1. Libraries

```cpp
#include <Servo.h>
