# Robot
Line following and collision avoidance robot

## Overview
The robot car uses five TCRT5000 sensors connected to ADC0-ADC4 inputs to detect a dark line on a light surface. ADC conversion is interrupt driven. Weighted average calculation is used to determine the "exact" location of the line. The sensors can be calibrated via UART for better accuracy. Measured values of the light and dark surface are stored in EEPROM. The motors are driven by a PID control. The K values are user configurable via UART as well.

In collision avoidance mode the robot car uses the HC-SR04 and the SG90 micro servo to "look around". The servo is hardware PWM driven via OC1A output using 16-bit Timer1. The HC-SR04 module is connected to INT0 and uses 8-bit Timer0 to measure RTT using two ISRs. Because of the limited "viewing" angle of the sonar, the servo sweeps between 54 and 126 degrees. When an object is detected within the turning distance, the robot performs a sweep from 0 to 180 degrees and chooses the angle with the maximum clear distance to turn to. When an object is detected within the collision distance, the robot reverses until no object is within collision distance anymore. The turning and collision distance are user configurable from the UART.

The DC motors are driven by software PWM implementation using PD4-PD7 outputs connected to the L293D motor driver and 8-bit Timer2 interrupt.

A mode button, connected to INT1, is used to switch between the different modes; line follow, collision avoid and idle. In idle mode, the robot can be driven "remotely" from the UART as well.

## Hardware
* ATmega8 @ 16 MHz
* HC-SR04 Ultrasonic Ranging Module
* L293D Motor Driver
* SG90 Servo
* TCRT5000 Reflective Optical Sensor

## Schematic

![](schematic/Robot.png)

## UART Control
The UART is configured for 9600 baud 8N1. The following keys are understood by the robot car:

Key | Function
---- | ----
A | Left
D | Right
W | Forward
S | Backward
Space | Stop
4 | Servo to right
5 | Servo to neutral
6 | Servo to left
G | Single sonar ping
O | Single ADC conversion
T | Toggle robot mode
C | Enter new collision distance
U | Enter new turning distance
M | Calibrate light surface
N | Calibrate dark line
P | Enter new Kp value
K | Enter new Kd value
I | Enter new Ki value
Enter | Show stored values

## Build
The robot is built using a ZK-4WD DIY car kit that uses geared DC motors. Two circuits boards are used, one for the optical sensors and one for all other hardware. The robot can be powered by 4 AA batteries and can be operated using the two buttons and the two indicator LEDs. Red means line follow mode, green means collision detect mode and both LEDs off means idle mode. In idle mode the robot car can be controlled and configured via UART.

![](media/robot.jpg)

## Firmware
The firmware has been developed in Atmel Studio 7 using GCC C and can be uploaded to the ATmega8 using the ISP connector and an ISP programmer such as [USBasp tool](http://www.fischl.de/usbasp/) using [avrdude](http://www.nongnu.org/avrdude/):

`avrdude -p m8 -c usbasp -U flash:w:Robot.hex:i -U eeprom:w:Robot.eep:i -U hfuse:w:0xC9:m -U lfuse:w:0xFF:m`

## Gallery



