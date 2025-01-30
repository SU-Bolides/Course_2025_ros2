---
layout: default
title: Main components
nav_order: 3
---

# Main components of the car

## 1. Chassis 

The cars are based on 1/10th scale Toyota GR86 Model based on the Tamiya  TT02 chassis. A very popular kit, loads of custom parts and upgrades are available. While the rules of the race forbid changing the power train and battery voltage, you can change the direction servo for a digital one, and upgrade some of the weaker parts to aluminium. While you shouldn't worry about this too much, as it is fixed, the current cars use 2S (~7.5V) 3000 mAh batteries, with a [Tamiya 540 Torque Tuned brushed motor](https://www.rcteam.com/en/products/tamiya-540-torque-25t-motor-54358) and an AX-12A digital servo controlled by a U2D2 controller. The motor is controlled by a [TBLE-04S](https://www.rcteam.com/en/products/tamiya-variateur-brushless-sensored-tble-04s-45069) Electronic Speed Controller (ESC). 



## 2. Electronics

The main brain of the car is a Raspberry Pi 5 8 GB, coupled with an [STM32 L432KC](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html) microcontroller for the low level control of the car. The STM32 collects data from the two [infrared sensors](https://www.gotronic.fr/art-capteur-de-mesure-sharp-gp2y0a21yk0f-11539.htm), the [ultrasound distance sensor](https://www.robot-electronics.co.uk/htm/srf10tech.htm), the [IMU](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) and the optical fork that measures the vehicle's drive shaft speed. It also sends the motor commands to the ESC, converting from a [-1, +1] command to a PWM signal. Finally, an [RPLiDAR A2M12](https://www.slamtec.com/en/Lidar/a2) and a [Raspberry Pi Command Module 2 (CM2)](https://www.raspberrypi.com/documentation/accessories/camera.html) complete the sensor suite. 

## 3. Connections and fudgeries

The LiDAR is plugged in via USB to the Raspberry Pi, and so is the U2D2. This creates too much current draw at LiDAR startup, so the LiDAR is additionally plugged in via a secondary USB A port jacked in to the main 5V bus. As I didn't have any spare USB A ports lying around, I took one from an old keyboard of mine, making the result very janky. It works, but should be handled carefully. 

The AX12-A is plugged in to the U2D2 to get commands, but is powered by a second cable, barely long enough to make the connection to the rear PCB. Once again, this works but should be handled with care, and probably replaced with a slightly longer cable. 

The Raspberry Pi has a tendency to turn off when the car accelerates too much. The current draw makes the voltage drop too low, so the Pi turns itself off as a safety measure, but this results in frequent crashes. Acceleration should be limited, but one could also look at slightly bumping up the voltage to the Pi to compensate for the subsequent drop. You will find all the PCB schematics on the [CoVAPSy GitHub](https://github.com/ajuton-ens/CourseVoituresAutonomesSaclay).

Because the car uses a docker container, the device adresses of the U2D2 and LiDAR don't retain their aliases. As such, there's a 50/50 chance that one will be ```/dev/ttyUSB0``` or ```/dev/ttyUSB1```. Finding out which one is which is trivial, but the ports need to be updated to reflect the truth. You will find more details about this in the [Operation section](https://su-bolides.github.io/Course_2025/operation/). 

