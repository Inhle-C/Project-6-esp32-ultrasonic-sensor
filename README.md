# Project 6: Ultrasonic Range Finder with ESP32

**Part 6 of my IoT Lab Series using ESP32 and Raspberry Pi 4**

## Overview

This project implements an ultrasonic range finder using the HC-SR04 sensor, enhanced with temperature compensation for accurate distance measurement.  
By adjusting the speed of sound based on real-time temperature readings, the ESP32 improves measurement precision for distances between 10 and 20 cm.

The goal is to demonstrate accurate sensor integration and environmental compensation in embedded systems.

## Objectives

- ✅ Read distance using the HC-SR04 ultrasonic sensor
- ✅ Read current temperature from the onboard temperature sensor
- ✅ Adjust the speed of sound based on temperature (assumed between 0°C and 50°C)
- ✅ Display distance (cm) and temperature (°C) every second via idf.py monitor
- ✅ Ensure less than 2 cm error in measured distances (10–20 cm range)
- ✅ Optimize measurements using multiple pulses or higher-precision counters

## Project Structure

esp32-lab6-ultrasonic-sensor/  
├── report.pdf # Lab report (required)  
├── lab6_1/  
│ ├── sdkconfig  
│ ├── hello_word.bin //this is not in the git as the file is too big  
│ ├── CMakeLists.txt  
│ ├── README.md  
│ └── main/  
│ ├── CMakeLists.txt  
│ ├── main.c  
│ └── ultrasonic.h  

## Setup Instructions

### 🌡️ Lab 6.1: Distance Measurement with Temperature Compensation

1. Connect the HC-SR04 ultrasonic sensor to the ESP32
2. Read distance data from the sensor
3. Read temperature data from the onboard sensor
4. Calculate the speed of sound based on temperature:
- Use appropriate formulas from the provided documentation
5. Print to terminal every second:
Distance: X.X cm at YY°C

6. Calibrate to ensure accuracy:
- Test with distances between 10 cm and 20 cm
- Maintain less than 2 cm error from PCB to surface

7. Optional:
- Generate multiple pulses for better accuracy
- Use high-precision counters from hal/cpu_hal.h

## Notes

- Exclude build/ directories when zipping the project.
- Submit the required files and directories: lab6_1/*
- Document issues or learnings in report.pdf and subfolder README.md.
- All external code must follow APACHE or BSD-like licenses.
- Reference any helpful resources properly in report.pdf (No StackOverflow, Reddit).

## What I Learned

- Integrating ultrasonic distance measurement on embedded systems
- Adjusting sensor readings based on environmental factors (temperature)
- Improving measurement accuracy with calibration and advanced counters
- Real-time data output with ESP32
- Working with timing precision for better results

## Future Improvements

- Extend range beyond 20 cm with higher sensitivity
- Add filtering or averaging for more stable readings
- Display readings on an external screen (e.g., LCD)
- Store distance data for later analysis
- Trigger alarms based on distance thresholds

## License
This project is for educational purposes.

Previous Project: [ESP32 Morse Code LED Communication](https://github.com/Inhle-C/Project-5-esp32-morse-led)  
(Part 5 of the series)

Next Project: [ESP32 Rust Setup](https://github.com/Inhle-C/Project-7-esp32-iot-setup-rust) 🔗  
(To be uploaded as Part 7 of the series)
