# Line-Following Robot for 2024 Robochallenge

This project is a line-following robot, developed using an ESP32 microcontroller, designed to participate in the 2024 Robochallenge in Bucharest. The robot uses PID (Proportional-Integral-Derivative) control to adjust its motors based on feedback from a line sensor array. This implementation includes a web interface for tuning PID values and other parameters in real-time.

## Features

- **PID Control**: The robot uses PID to adjust motor speeds for smooth and precise line-following.
- **Web Interface**: Allows real-time tuning of key PID parameters and sensor weights via a web page hosted on the ESP32.
- **Sensor Array**: Utilizes a 13-sensor array for accurate line detection.
- **Motor Control**: Powered by a DRV8835 Motor Driver and custom motor pins.

## Components

### Hardware
1. **[ESP32](https://www.optimusdigital.ro/en/esp32-boards/12933-plusivo-esp32-and-ble-compatible-wireless-development-board.html?search_query=esp32&results=38)** - Microcontroller used to process sensor data and control the motors.
2. **[Pololu DRV8835 Motor Driver](https://www.pololu.com/product/2135)** - Dual motor driver for controlling two motors independently.
3. **[Pololu QTRX-MD-13RC Reflectance Sensor Array](https://www.pololu.com/product/4353)** - High-precision sensor array for line detection.
4. **[Pololu DC Motors](https://www.pololu.com/product/999)** - Pololu 10:1 Micro Metal Gearmotor HP 6V.
5. **Chassis and Mounts** - 3D-printed parts (see 3D Model section) for housing the sensors, motors, and ESP32.
6. **[Battery](https://hpi-racing.ro/li-po-2s-74v/acumulator-lipo-gens-ace-g-tech-soaring-450mah-74v-30c-2s1p-cu-jst-syp.html)** - Lipo Gens Ace Acumulator - G-Tech Soaring - 450mAh - 7.4V - 30C - 2S1P with JST-SYP
7. **Miscellaneous** - Jumper wires, connectors, screws, PCB and other hardware for assembly.

### Software Libraries
- **DRV8835MotorShield** - For motor control.
- **WiFi** and **WebServer** - For web interface and real-time parameter adjustments.

## Circuit Diagram

Connect the components as follows:
- **DRV8835 Motor Driver**: Connect M1DIR, M1PWM, M2DIR, M2PWM pins to custom GPIO pins 19, 18, 16, and 17 respectively.
- **QTRX-MD-13RC Sensor Array**: Connect each of the 13 sensors to GPIO pins as defined in the code.
- **ESP32 Power and Ground**: Ensure a stable power source is connected to the ESP32 and motors.

## 3D Model

The chassis and sensor mounts are designed to be 3D-printed for robustness and optimal sensor positioning. The STL files for 3D printing can be found in the `3d-models` folder of this repository.

## Setup Instructions

1. **Connect Hardware**: Assemble the components according to the circuit diagram and ensure all connections are secure.
2. **Upload Code**: Flash the code to the ESP32 using Arduino IDE.
3. **Connect to WiFi**: Modify the `ssid` and `password` variables with your WiFi credentials to enable the web interface.
4. **Access the Web Interface**: Once connected, open the IP address of the ESP32 in a browser to access the PID tuning interface.

## Tuning the PID Controller

Navigate to the web interface to adjust:
- **Base Speed**: Sets the default motor speed.
- **Kp, Ki, Kd**: PID control values to balance proportional, integral, and derivative responses.
- **Sensor Weights**: Modifiable weights for each sensor position to enhance control accuracy.

## Code Overview

### Line Sensor

The `LineSensor` class initializes and reads from the sensor array to calculate positional error for PID correction.

### PID Control

The `PID` function calculates the output based on current error, adjusting motor speeds accordingly.

### Web Interface

The `WebInterface` class hosts a page for real-time adjustments to PID values and sensor weights.

## Part List

| Part                           | Quantity | Description |
| ------------------------------ | -------- | ----------- |
| ESP32                          | 1        | Microcontroller |
| DRV8835 Motor Driver           | 1        | Dual motor driver for controlling two motors |
| QTRX-MD-13RC Reflectance Sensor Array | 1        | 13-sensor array for line detection |
| DC Motors                      | 2        | Motors with sufficient torque and speed |
| Battery                        | 1        | Battery pack for powering ESP32 and motors |
| 3D Printed Chassis & Mounts    | Various  | Holds sensors, ESP32, and motors in place |
| Jumper Wires, Connectors, Screws | Various | Assembly hardware |

## 3D Model Files (TBD)

STL files for the 3D-printed chassis and sensor mounts are provided in the `3d-models` folder (TBD).

---

Happy building and best of luck!
