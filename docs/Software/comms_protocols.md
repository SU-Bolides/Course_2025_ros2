---
layout: default
title: Communication protocols
excerpt: SPI yadee yadee
nav_order: 2
---

# Communication protocols

All the cars sensors must talk to each other efficiently and quickly to ensure accurate but fast data transmission. 


## STM32

The STM32 handles most of the data collection, and is the least trivial to debug / program. A logic analyser is recommended. 

### IMU

Communication with the BNO055 IMU is mada via I2C at a rate of about 50 Hz. Care is taken to not read too many registers, and only yaw and angular velocity are of any use. The IMU must respect a specific startup sequence, which is hardcoded in the STM32 code. It is recommended to reset everything before starting the race to avoid any potential issues related to sensor initialisation. Reseting the STM32 is done by clicking the bottomn button in the row of three yellow buttons situated to the right of the screen. Pressing it will lead to a long beep, at the end of which the car will be ready to go.

To know more about the BNO055 and how to access its registers, the [datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf) should be carefully read through. Some details are also given in the STM32 code, in the [bno055 part](https://github.com/SorbonneUniversityBolideContributors/Course_2025/blob/main/CoVAPSy_STM32/Core/Src/CoVAPSy_bno055.c). Be careful when changing things, it's easy to break things and hard to diagnose if you don't have access to proper equipment. Please check out the Troubleshooting section if you encounter any issues. 

### Speed sensor 

The speed is detected using an Optical fork, which sends interrupts every time the wheel interrupts the light beam. Based on the timing between these interrupts, the speed is measured based on the following bits of code:

```c
if((delta_us) >= 300) //si mesure cohérente (pas un glitch):
	{
		if((mesure_us > (mesure_precedente_us+100000)) || ((mesure_us-100000) > mesure_precedente_us)) //cas d'un nouveau départ (the second term is for overflow)
		{
			memset(tableau_intervalles_us, 0, sizeof(tableau_intervalles_us));
			indice=0;
		}
		else //cas où on tourne depuis plus d'un intervalle
		{
			tableau_intervalles_us[indice] = delta_us; //on sauvegarde la nouvelle mesure dans le tableau
			//On fait une moyenne sur 20 ms au plus ou 16 valeurs.
			somme_intervalles_us = 0;

			i= indice+1;

			do{
				if(!tableau_intervalles_us[i-1]) {
						break;
				}

				somme_intervalles_us += tableau_intervalles_us[i-1];

				i--;
				if (!i) {
					i = 16;
				}

				nb_intervalles++;

			} while ((somme_intervalles_us<20000) && (nb_intervalles < 16));

			indice = (indice+1)%16; // on incrémente l'indice avec retour à 0 pour indice = 16

			vitesse_mesuree_mm_s = 1000 * coefficient_distance_par_intervalle_um * nb_intervalles / somme_intervalles_us;

		}
		mesure_precedente_us = mesure_us;
	}
```

For some reason, the resulting speed still needs to be multiplied by 2. This may be a stupid oversight on my part, but I'm too lazy to actually go through finding out why it is that way. 

### ESC

The STM32 controls the throttle of the car by sending a PWM signal to the ESC. This needs to be calibrated following the ESC's calibration procedure detailled in its datasheet, but mostly consists of pressing a button, applying full throttle, pressing a button, then applying 0 throttle, then braking, etc. This is easy to program in ROS and should pose no problem. 

### Ultrasound

Don't bother, it's a hassle and barely worth it. The infrared sensors are faster, less annoying to deal with, and yield pretty much the same results. I took the ultrasound off the car entirely. 

### Infrared sensors

Standard analog read. Some constants to go from the reading to an actual measurement, but that's all handled by the stm32_publisher node.

### Display

Don't bother, it's slow and the car is supposed to be moving anyway. You can go through the documentation to try and figure out how to change it, but I seriously wouldn't use it for anything serious. 

## Raspberry Pi

### LiDAR 

The LiDAR is connected via USB through a UART-USB converter. This converter also handles the additional power delivery of the secondary USB port. Controlling the LiDAR is done through the RPLidar ROS package linked in the workspace. Just launch ```rplidar_a2m12.launch``` and you'll have your data delivered on a topic at 12Hz. 

### U2D2 (Dynamixel Servo)

The direction servo is controlled by a U2D2, whose role is to convert the USB instructions to Dynamixel-understandable TTL. The Dynamixel SDK is used to handle the communication, the baudrate is 115200 and the protocol version is 1.0. The Dynamixel comms are handled in the ```ackermann_controller.py``` node, and several examples can be found in the Robotis Documentation. It is recommended to install the DynamixelWizard to your computer to easily debug and control the servo without going through the SDK. Useful settings include setting min,max position limits, and setting the PID gains for position control. 
