---
layout: default
title: Communication protocols
excerpt: SPI yadee yadee
nav_order: 2
---

# Communication protocols

All the cars sensors must talk to each other efficiently and quickly to ensure accurate but fast data transmission. 

## STM32 <-> Raspberry PI
### On the STM32
The communication between the two uses SPI at 200 Hz, but a Circular Redundancy Check (CRC) is added to make sure only correct data is being processed. 

The L432KC has a dedicated CRC module, which can take 4 bytes at a time. The following code is the SPI implementation on the STM32, with the CRC:

```c
//Reset CRC;
CRC->CR = 1;


SPI_TxBuffer[0] = (uint8_t)((lectures_ADC[2] >> 8) & 0xFF);
SPI_TxBuffer[1] = (uint8_t)(lectures_ADC[2] & 0xFF); //battery_voltage (unit ?)
SPI_TxBuffer[2] = (uint8_t)((yaw >> 8) & 0xFF);
SPI_TxBuffer[3] = (uint8_t)(yaw & 0xFF);

CRC->DR = (uint32_t)((((uint16_t) lectures_ADC[2] << 16)) | ((uint16_t) yaw));


SPI_TxBuffer[4] = (uint8_t)((((uint16_t)(lectures_ADC[0])) >> 8) & 0xFF);
SPI_TxBuffer[5] = (uint8_t)(((uint16_t)(lectures_ADC[0])) & 0xFF);   // LEFT IR
SPI_TxBuffer[6] = (uint8_t)((((uint16_t)(lectures_ADC[1])) >> 8) & 0xFF);
SPI_TxBuffer[7] = (uint8_t)(((uint16_t)(lectures_ADC[1])) & 0xFF);  // RIGHT IR

CRC->DR = (uint32_t)(((uint16_t) lectures_ADC[0] << 16) | ((uint16_t) lectures_ADC[1])); //Put 4 next bytes in the CRC register


SPI_TxBuffer[8] = (uint8_t)((vitesse_mesuree_mm_s >> 8) & 0xFF); // Octet de poids fort de vitesse_mesuree_mm_s
SPI_TxBuffer[9] = (uint8_t)(vitesse_mesuree_mm_s & 0xFF);    // Octet de poids faible de vitesse_mesuree_mm_s
SPI_TxBuffer[10] = (uint8_t)((distance_US_cm >> 8) & 0xFF);
SPI_TxBuffer[11] = (uint8_t)((distance_US_cm &0xFF));

CRC->DR = (uint32_t)(((uint16_t) vitesse_mesuree_mm_s << 16) | ((uint16_t) distance_US_cm));


SPI_TxBuffer[12] = (uint8_t)((acc_y >> 8) & 0xFF);
SPI_TxBuffer[13] = (uint8_t)((acc_y &0xFF));
SPI_TxBuffer[14] = (uint8_t)((yaw_rate >> 8) & 0xFF);
SPI_TxBuffer[15] = (uint8_t)((yaw_rate &0xFF));

CRC->DR = (uint32_t)(((uint16_t) acc_y << 16) | ((uint16_t) yaw_rate));


uint32_t checksum = CRC->DR; //Read from register to get computer value

CRC->CR = 1; //Reset, get it ready for the RX

//Send the checksum over SPI.
SPI_TxBuffer[16] = (uint8_t)((checksum >> 24) & 0xFF);
SPI_TxBuffer[17] = (uint8_t)((checksum >> 16) & 0xFF);
SPI_TxBuffer[18] = (uint8_t)((checksum >> 8) & 0xFF);
SPI_TxBuffer[19] = (uint8_t)(checksum & 0xFF);

HAL_SPI_Receive(&hspi3, (uint8_t *)SPI_RxBuffer, 8,30);
HAL_SPI_Transmit(&hspi3, (uint8_t *)SPI_TxBuffer, 20,10);

while(HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);

CRC->DR = (uint32_t)((uint32_t)SPI_RxBuffer[0] << 24 | (uint32_t)SPI_RxBuffer[1] << 16 | (uint32_t)SPI_RxBuffer[2] << 8 | (uint32_t)SPI_RxBuffer[3]);

CRC->DR = (uint32_t)((uint32_t)SPI_RxBuffer[4] << 24 | (uint32_t)SPI_RxBuffer[5] << 16 | (uint32_t)SPI_RxBuffer[6] << 8 | (uint32_t)SPI_RxBuffer[7]);

checksum = CRC->DR;
if (checksum) {
  asm("nop");
} else {
  ESC_pulse_us = (uint16_t)(((uint16_t)SPI_RxBuffer[0] << 8) | (uint16_t)SPI_RxBuffer[1]);
}

```

At first, the CRC Reset bit is set to 1 to put everything back in a known config. The TX buffer is then filled up in 8 bit increments. Each data field is 16 bits (or 2 bytes), so each data field is split in two bytes, the first 8 bits being set then the next 8 shifted by a byte and loaded in. The CRC Data register is then set to the first 4 bytes. This process is repeated until SPI_TxBuffer is full and the whole data has been sequentially set to the CRC Data regiser. The computed value is then extracted from the Data Register by reading it, and its 4 bytes are loaded into the TxBuffer. 

The CRC Reset bit is set to prepare the CRC for checking the received data. The RxBuffer is filled with the received data (sent by the RPi) and the TxBuffer is sent to the RPi. The received data is then sent to the CRC Data Register, and if the transmission was uncorrupted, the Data Register should then read 0. If that is the case, the ESC_pulse field is set to the RxBuffer data fields, and the whole process starts again on the STM32.

### On the Raspberry Pi

The ```stm32_publisher.py``` node takes care of the Rx Tx via SPI. The spidev library is used to access the SPI data, and the following function implements an equivalent CRC as to the one used in the STM32:
```python
def crc32mpeg2(self,buf, crc=0xffffffff):
        for val in buf:
            crc ^= val << 24
            for _ in range(8):
                crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
        return crc
```

This returns 0 if the data is uncorrupted, and something else if it isn't. 


## STM32

The STM32 handles most of the data collection, and is the least trivial to debug / program. A logic analyser is recommended. 

### IMU

Communication with the BNO055 IMU is mada via I2C at a rate of about 50 Hz. Care is taken to not read too many registers, and only yaw and angular velocity are of any use. The IMU must respect a specific startup sequence, which is hardcoded in the STM32 code. It is recommended to reset everything before starting the race to avoid any potential issues related to sensor initialisation. Reseting the STM32 is done by clicking the bottomn button in the row of three yellow buttons situated to the right of the screen. Pressing it will lead to a long beep, at the end of which the car will be ready to go.

To know more about the BNO055 and how to access its registers, the [datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf) should be carefully read through. Some details are also given in the STM32 code, in the [bno055 part](https://github.com/SorbonneUniversityBolideContributors/Course_2025/blob/main/CoVAPSy_STM32/Core/Src/CoVAPSy_bno055.c). Be careful when changing things, it's easy to break things and hard to diagnose if you don't have access to proper equipment. Please check out the [Troubleshooting section](https://su-bolides.github.io/Course_2025/troubleshooting.html) if you encounter any issues. 

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
