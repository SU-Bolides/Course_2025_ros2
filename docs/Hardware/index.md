---
layout: default
title: Hardware
nav_order: 3
---

# Hardware setup

## 1. Chassis 

The cars are based on 1/10th scale Toyota GR86 Model based on the Tamiya  TT02 chassis. A very popular kit, loads of custom parts and upgrades are available. While the rules of the race forbid changing the power train and battery voltage, you can change the direction servo for a digital one, and upgrade some of the weaker parts to aluminium. While you shouldn't worry about this too much, as it is fixed, the current cars use 2S (~7.5V) 3000 mAh batteries, with a [Tamiya 540 Torque Tuned brushed motor](https://www.rcteam.com/en/products/tamiya-540-torque-25t-motor-54358) and an AX-12A digital servo controlled by a U2D2 controller. The motor is controlled by a [TBLE-04S](https://www.rcteam.com/en/products/tamiya-variateur-brushless-sensored-tble-04s-45069) Electronic Speed Controller (ESC). 



## 2. Electronics

The main brain of the car is a Raspberry Pi 5 8 GB, coupled with an [STM32 L432KC](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html) microcontroller for the low level control of the car. The STM32 collects data from the two [infrared sensors](https://www.gotronic.fr/art-capteur-de-mesure-sharp-gp2y0a21yk0f-11539.htm), the [ultrasound distance sensor](https://www.robot-electronics.co.uk/htm/srf10tech.htm), the [IMU](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) and the optical fork that measures the vehicle's drive shaft speed. It also sends the motor commands to the ESC, converting from a [-1, +1] command to a PWM signal.  

