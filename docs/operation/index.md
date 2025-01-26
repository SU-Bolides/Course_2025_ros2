---
layout: default
title: Operation
nav_order: 1
---

# Operation of the car

Here you will find the detailed guide on how to interact and operate with the car. The quick start guide should be enough to get you started, but this should be read once you're ready to actually work on the car.

## Raspberry Pi network connections

The Raspberry Pi connects to networks specified in the ```/etc/wpa_supplicant/wpa_supplicant.conf``` file. The current three networks are my home network, the router, and my phone's hotspot. If you don't have access to the router, you can simply rename your phone's hotspot to ```money trees``` and set the password to ```Poulette``` and the car will connect to it, allowing you to change the config. You should also be able to connect via Ethernet to it directly, although I haven't personally tried this out. 

You can also turn on the Raspberry Pi Access Point (AP), although that isn't recommended because you lose all internet connectivity, which makes updating or installing packages a chore. The current preferred method of access is using your phone's hotspot. 

## Navigating the file system

The easiest method to interact with the Pi's file system is to use VS Code with the Remote-SSH and Docker extensions. Once you have installed the extensions and reloaded VS Code, press ```CMD + SHIFT + P``` or ```CTRL+SHIFT+P``` to open the Command Palette, type in ```Remote SSH: Connect to host```, fill in the details and log in to the car. Then, on the sidebar to the right of your screen, click on the Docker icon (🐳), navigate to bolide_container, and right click ```Attach to VS Code```. If the container isn't started yet, start it. The home directory of the docker image is ```/home/bolide1/```. 

## Launching the existing algorithms

The two main launch files are ```ready_for_nav.launch``` and ```ready_for_slam.launch```. They will launch the required process nodes to parse the various sensors' data, and launch additional files like the particle filter or the SLAM node depending on the chosen launch file. The actual controllers that make the decisions on where the cars move are to be launched separately. The best working one is the stanley_controller, which should be launched via the corresponding ```stanley_launch.launch``` file. 

## Making a map of the circuit

After connecting to the car and placing it in the track, launch the ```ready_for_slam.launch``` file. On your host computer, launch an RVIZ instance to monitor the progress, and launch a teleop node to control the car. Pilot the car around the circuit, making sure to be as smooth as possible to minimise odometry errors. RVIZ will show you the map being built live. When the result is good enough, run ```rosrun map_server map_saver -f TRACK_NAME``` to save the .pgm and .yaml files. You can now turn the nodes off and take the car away from the track.

## Finding the optimal trajectory

Using the ```extract_centerline.ipynb``` notebook, extract the centerline from the map. If the centerline isn't a line, but has some "branches", use an image editing program (GIMP is free on Ubuntu) to paint the curves smooth. Then, run the ```sanity_check.ipynb``` notebook to double check that the extracted centerline is in the right coordinate system. 

You will now have a .csv file with the centerline coordinates x,y and the trackwidths to the left and right of the point. Place that file in the raceline optimisation "inputs/tracks" folder, then run ```main_globaltraj_tt02.py```. After it has completed, you will find the trajectory in "outputs/MAP_NAME/". The folder names include their creation date. Run the ```visualize_raceline.ipynb``` notebook to triple check that the extracted trajectory is correct and feasible. Then, take the original MAP_NAME.yaml, MAP_NAME.pgm, as well as the raceline file, and place the first two in ```perception_bolide/maps/```, and the latter in ```control_bolide/racelines/```. 

## Following the trajectory

### VERY IMPORTANT 

ALWAYS TRIPLE CHECK THAT YOU HAVE THE CORRECT MAP AND CORRECT RACELINE LOADED.

The raceline is specified in ```stanley_launch.launch```: 

```json
<param name="waypoints_path" value="/home/bolide1/bolide_ws/src/course_2024_pkgs/control_bolide/racelines/RACELINE_NAME.csv"/>```

While the map file is specified in ```ready_for_nav.launch```: 

```json
<arg name="map" default="$(find perception_bolide)/maps/MAP_NAME.yaml"/>```
### VERY IMPORTANT
