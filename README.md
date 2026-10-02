# Autonomous-Robot-Design

Team 4’s ELE2025 embedded systems project: a two-wheel robot designed for remote control and autonomous line following while pulling a lightweight trailer.

## Project overview

The robot has two operating modes:

- **Manual control:** An ESP32 reads controller inputs and sends them to the Arduino, which controls the motors.
- **Line following:** Three TCRT5000 infrared reflection sensors detect the track line and guide the robot.

The project also includes a servo-operated mechanism, wheel encoder measurements, and a live telemetry dashboard hosted by the Arduino’s Wi-Fi connection.

## Hardware

- Arduino UNO R4 WiFi
- ESP32 controller interface
- Two 6 V, 160 RPM geared DC motors with integrated quadrature encoders
- MakerDrive PCB with two H-bridge circuits for driving the motors
- Three TCRT5000 infrared reflection sensors
- Servo
- Lightweight trailer and pickup mechanism

The motor specification describes a 120:1 gearbox and an encoder resolution of 16 pulses per motor revolution, giving up to 1,920 pulses per gearbox output-shaft revolution. The Arduino sketch currently uses a configured value of **170 counts per wheel revolution** for its speed and distance calculations.

## Software features

- Switching between manual control and line-following mode
- Differential motor control using the controller’s steering and trigger inputs
- Servo movement from controller bumper inputs
- Wheel encoder counting and estimated speed and distance calculations
- Live Wi-Fi dashboard showing operating mode, encoder counts, speed, distance, servo angle, and runtime
- JSON telemetry endpoint for dashboard data

## Project brief

The project brief required a robot that could demonstrate line following and remote-control operation, attempt to pick up and pull a lightweight trailer, and provide live telemetry while navigating the course.
