---
layout: default
title: Heuristics
nav_order: 2
---

# Software and heuristics

## ROS 1

The main framework linking all the components together is **ROS1 Noetic** (Compatible with Ubuntu 20.04), which you should be familiar with. Migration to ROS2 is planned but not yet executed. The whole workspace is found in the pkgs repo, but you will find some details about the implementation and choices made with regards to the car and the navigation in this section. 

## Odometry

The car is equipped with an IMU, a speed sensor, and a steering angle sensor (The Dynamixel servo). The speed sensor is absolute, not signed, so simply integrating speed over time wouldn't account for the car moving backwards. To account for this, the following assumption is made: 

> The car only moves under its own power.

As such, the commanded speed's sign is used to determine whether the car has been instructed to move forwards or backwards, and is then applied to the speed to sign it. Regarding orientation, using the IMU's angle alone leads to significant, which makes the LiDAR data lag behind the actual track, making the particle filter's job more difficult. As such, the steering angle + speed are used to estimate the linear and angular velocity of the car, following the Ackermann Bicycle kinematic model. This estimation is then fused with the IMU measured angular velocity, leading to a way better odometry which leads to cleaner maps and faster localisation. 

The IMU and speed sensor data is collected by the STM32 and transmitted to the Raspberry Pi over SPI, then parsed and published as a ROS Topic by the stm32_publisher node. This same node also handles the data being sent to the STM32. More information about this implementation in the Communication protocols section.

## SLAM

The SLAM implementation used on the car is a version of OpenKarto using the SPA solver. It is a graph based SLAM algorithm, making loop closure essential. The params of it can be adjusted, but are difficult to nail if not in-situ. 

## Navigation - Stanley

The navigation heuristic used in the 2024 Race closely follows the one used by Stanford University in the 2005 DARPA Grand Challenge. Reading the paper is essential for a good understanding of it. Offline, a reference trajectory is found by optimization, using an algorithm developed by the TUMFTM (Munich's Technical University's Automotive department). This trajectory is then followed by the car by using a Stanley Controller.

## Navigation - MPC

Implementing an MPC controller would make obstacle avoidance a lot easier, but it has proven to be more difficult than initally thought. First tests with a primitive RK4 controller have showcased the need for a more robust constraint formulation. The current implementation uses ACADOS to generate C code based on CasADi formulas, allowing for fast execution times. The model used is a kinematic model, because the current harware is limiting the max speed of the car below slip conditions, and the trajectory following heuristic is based on the Alex Liniger implementation for the AMZ Race Car of ETH Zurich. Another must read. 

## Obstacle detection - LiDAR

An approach similar than the one I used in my Autonomous TurtleBot project has been implemented effectively, using DB Scan and manually tuned thresholds to detect clusters corresponding to obstacle. This works well, and the fact that all the obstacles are assumed to be cars of roughly similar shapes and sizes makes it very robust. It is of my personal opinion that this is the most effective method given the context.

## Obstacle detection - Vision

Using YOLO to detect opponents has proven less effective, more computationnaly expensive, and less accurate than LiDAR based deteciton. Nonetheless, the promised arrival of a Qualcomm RB5 Devkit should give the approach a boost. 
