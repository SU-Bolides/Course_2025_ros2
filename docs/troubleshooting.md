---
layout: default
title: Troubleshooting
nav_order: 5
---

# Troubleshooting the car

Got an issue with the car? Here is a list of common issues and their fixes.

## Communication problems

### The STM32 is not talking to the Raspberry Pi

Symptoms: 
- Missing rostopics related to the STM32 such as ```/raw_fork_data``` or ```/raw_imu_data```.

To do: 
* Check the STM32 LED number 4. Its state switches at every loop iteration on the STM32. If it is visibily blinking, that means that the SPI communication is not working. In nominal conditions, it should be turning on and off again so quickly that it appears to be steadily lit.
* Reset the STM32 by pressing the lowest button of the three yellow buttons to the right of the screen. The screen should be on and displaying Sorbonne Bolide 1
* Have you launched ```stm32_publisher.py```? It is required to parse SPI data. Check namespaces. Can you see other topics such as ```/rosout```? 
* Check if there's an issue with the hardware configuration of the Raspberry Pi if you made any changes to it. SPI needs to be enabled.

### The car is not moving when commanded 

Symptoms:
- The car is not moving forward, but the steering works.

To do: 
* Have you turned the ESC on? There is a switch situated near the motor. It should be switched to on, in which position it should beep shortly before the STM32 signal is detected.
* If the battery charge is too low, the car might not move for low throttle inputs. Try turning it up a notch.

### The car is not turning when commanded

Symptoms:
- The car is not steering, but moves forward.

To do: 
* Have you checked that the U2D2 is on the correct port ? Try setting the port to 1 if at 0, or 0 if at 1 in ```ackermann_controller.py:92```.
* Check if the U2D2 is lit up blue/green. This means that data is transiting through it. Can you see the steering angle in the ```/car_state``` topic? Check that the servo LED is on.
* Use DynamixelWizard on your computer to check if everything is alright servo-wise.

### The Raspberry Pi doesn't connect to my hotspot

Symptoms:
- Turning the car then the Raspberry Pi on, the latter doesn't connect to the hotspot.

To do:
* Disconnect every other connected device. Stay on the personal hotspot page to make it discoverable. Reboot the Raspberry Pi, or your phone.
* Check available networks for BolideX_AP. If it is available, connect to it, and use ```sudo raspi-config``` then ```System / Wireless LAN``` to connect to your phone.

### ping: cannot resolve bolide.local: Unknown host

Symptoms:
- The Raspberry Pi connects to the network, but I can't ping it or connect to it.

To do:
* Check that the host computer is connected to the same network. If using a VM, try pinging it from there. Be sure to set the Network configuration to Bridged for the VM.
* Go to the router admin page to see connected devices' IP addresses. The DNS of the router might be broken, so you have to ping the IP address directly.

### I can see the topics on my computer but the car doesn't receive any commands

Symptoms:
- When trying to set pose in RVIZ or Teleop, the car doesn't respond, but I can see the topics of the car on the computer.

To do:
* ssh to the car and ```rostopic echo``` the topic that should receive the message. If the message is received, the problem isn't the comms from PC to Car.
* run ```hostname -I``` or equivalent on the host computer. Check that the IP address matches that in ```echo $ROS_HOSTNAME```. If it doesn't, change it in your bashrc ```nano ~/.bashrc```.

### My computer is connected and I can ssh but I can't see the topics of the car

Symptoms:
- On the SSH shell, I can see the ROS topics such as ```/rosout```, but I can't on my own computer.

To do:
* ssh to the car and ```hostname -I```. On your own computer, check that the IP matches that in ```echo $ROS_MASTER_URI```. On the car, check that it matches ```echo $ROS_MASTER_URI``` and ```echo $ROS_HOSTNAME```. Correct the errors in the respective bashrcs. **Don't forget to add the :11311 after the ROS_MASTER_URI IP**.


## General errors

### docker: Cannot attach to bolide_container: container is stopped

Symptoms: 
- When runnning ```docker attach bolide_container```, it errors out because the container is stopped.

To do:
* ```docker start bolide_container```

### unknown msg type: control_bolide/SpeedDirection (or similar)

Symptoms:
- When trying to launch something locally such as the Teleop node, this error appears.

To do:
* Download the course_202X_pkgs to your local workspace and catkin_make. Then, run ```source devel/setup.bash```.

### Couldn't find executable XXX. Found the following, but they're either not files or not executable

Symptoms:
- When trying to rosrun or roslaunch something, this pops up.

To do:
* ```chmod + x /path/to/file.py```

### Launching a node yields nothing, CTRL-C wakes it up but it still doesn't work.

Symptoms:
- When trying to launch or run something ROS related, it times out and appears to do nothing.

To do:
* Set the ROS_MASTER_URI to the correct IP address, making sure to add port 11311 to it (:11311).

### The STM32 data is nonsensical

Symptoms:
- The data published to the sensor topics by the stm32_publisher node is nonsensical.

To do:
* Reset the STM32 by pressing the lowest of the three yellow buttons situated to the right of the screen.

### The Raspberry Pi shuts down on its own

Symptoms: 
- Out of nowhere, the Raspberry Pi shuts down, yielding to a disconnect and a red power LED.

To do:
* Swap out the battery, the voltage is too low.


## Problem not listed? 

Check the [reports folder](https://github.com/SU-Bolides/Course_2025/tree/main/reports), you might find help there. 

Open an issue on GitHub detailing the problem, close it when completed, add it to this list. 

Shoot me an email me@nicolashammje.com, I might be able to help. 
