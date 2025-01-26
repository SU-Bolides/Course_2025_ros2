---
layout: default
title: Homepage
nav_exclude: true
---

# Sorbonne University Bolides Manual

This is the website where you will find all the answers and explanations related to the Sorbonne University TT02 1/10th race cars made for the CoVAPSy Races. 

This might not be fully done yet, but I am actively working on completing this before I finish my masters degree. If there's anything you need that isn't covered here, ask me directly by sending me an email me@nicolashammje.com.

Have fun working on this. 

## Where to start 

If you don't know anything about the car, start by reading through the Introduction. If you want to get the car running as soon as possible, read the [Quick Start Guide](https://sorbonneuniversitybolidecontributors.github.io/Course_2025/quickstart.html). 

The rest is covered in the other pages. You can also check out the [ressources folder](https://github.com/SU-Bolides/Course_2025/tree/main/ressources) in the main repo to check out the different documents related to the project. 

# Introduction

This is the website for the Sorbonne University student teams entering in the CoVAPSy races organised by ENS Saclay every year. The race is devised into two distinct parts: 

### Qualifying

The goal of this phase is to complete a lap of a circuit as fast as possible. The rankings of this phase determine the starting grids for the main races. The first attempt is made on a track devoid of obstacles, whereas the second phase has a static obstacle placed on the track. The track layout changes between the two attempts, and the final time used for the rankings is the sum of the two. 

### Main race

Usually a five lap, eight contestant race on a new circuit layout. Speed isn't everything, as the high number of contestants makes for a very chaotic race where everyone has a chance at victory.

## 2024 Race recap

We entered two cars in the 2024 race, both using two different navigation heuristics. The hardware is pretty much identical between the two cars, but one of them uses a Dynamixel instead of a standard Servo for steering control. The main difference lies in the heuristics used by the cars, one being based on a purely reactive system, whereas the second one makes a map of the circuit using a SLAM algorithm and then aims to follow a reference trajectory found by optimisation. 

The second car ended up 2nd/25 in the qualifying phases, but ultimately ended up 4th because the poor obstacle avoidance hindered its ability to avoid the other cars during the races. It ended up winning Best technological innovation prize for its use of SLAM. 

The first car ended up 4th/25 in the qualifying but ultimately ended up 2nd in a fun swaperoo with the other car. What it lacked in speed it made up for in sheer robustness, always finding ways around the other cars and ultimately only getting caught out once by a particularly bad crash. 

## 2025 Goals

To make the SLAM car better, the goal is to implement an MPC controller to formulate dynamically correct trajectories around the racetrack and its obstacles. Obstacle detection is done via both vision and LiDAR, but the Vision part has been delayed due to waiting for a specific part from a partner (A Qualcomm RB5 devkit). The LiDAR approach uses a clustering algorithm to pick out obstacles from the circuit walls.
