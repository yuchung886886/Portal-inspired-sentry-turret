# Portal inspired sentry turret
![Title](./Title.jpg)  
This repository provides the firmware source code and the hardware schematic of the Portal inspired sentry turret project. 

# Features demostration
- This project consists of two objects, the Turret and the Controller, which communicate with each other wirelessly.  
- The turret equips two AEG airsofts teared down from the UHC mini AEG series airsoft and two laser heads for zeroing.
- The turret equips a OV5640 camera module to provide a first-person view to navigate the pan/tilt of the turret.
- The turret carry a speaker and audio amplifier module to play the classic Portal sound effects.  
- Three NEMA 17 stepper motors driven by A4988 modules are used to implement the pan/tilt rotation and telescopic arms extension/retraction of the turret.  
- All electronic components on turret are integrated and controlled by a ESP32-S3 MCU.
- The controller also carry a ESP32-S3 MCU to control a 320x240 touch panel module, receive turret's status and captured images, and transmit user's commands to the turret.  
- The turret is powered by a 12V power supply and an extra airsoft battery; the controller is powered by four 1.2V AAA batteries.

# Hardware components
The schematic of the turret and controller are in the hardware/ folder of this repository. All electronic units placed in the schematic are off-the-shelf modules that  can be found and purchased on the web. You can refer to the part numbers of these modules listed in the Chapter 1.

# Firmware development environment setup
Two firmware source code packages, the Controller and the Turret, are in the firmware/ folder. These codes are developed based on Espressif ESP32-S3 MCU. ESP-IDF development framework v5.0.1 is required to configure the settings, build the firmware source code, and flash the firmware binary of these source packages. Please refer to the Chapter 2 for the detail tutorial.

# Mechanical components
All the STL files of the 3D printed parts are sold on Cult3D. Besides the 3D printed parts, there are also some off-the-shelf mechanical components used in this project and you can refer to the part names listed in the Chapter 1 to search and purchase these parts on the web.

# Tutorial
Please refer to the rest of tutorials in the playlist for the assembly guide of this project.
