# ***MADHEX-v2***
> ## the Fully open source 6 legged hexapod robot 
<img width="1066" height="482" alt="image" src="https://github.com/user-attachments/assets/c4eeb37d-0344-47bd-aa49-c55a325e3e97" />

* Custom controller PCB
* Fully open-source
* great mechanics design with amortization
#### the 18DOF hexapod robot 

## the README include 
* Overwiew
* Features
* Electronics
* CAD
* BOM
* Assembly guide
* Gallery

# Overview 
MadHex v2 is an open-source hexapod robot designed for people who wanna build their spider friend and learn building — CAD, PCB , and design  are all custom-made. This repository contains everything needed to reproduce the robot: CAD source and manufacturing files, PCB design and production files, bill of materials, and a full assembly guide.

Currently the project is in procces of building 

# Features

* Fully custom mechanical design , all parts are 3d printable
* custom control PCB : schematics , 4 layer of layout , kicad project , and manufactoring files
* 2 Full BOM files for both PCB and Robot
* great design with beatiful decisions

 ### Electronics PCB 

Custom 4-layer control board 

**Key Features :**

* **MCU:** ESP32-S3-WROOM-1 — great wifi/bluetooth MCU 
* **IMU:** BNO055 (I2C) — 9-DOF orientation for balancing
* **Power input:** XT60 connector - for battery connection 
* **Power protection:** P-MOSFET reverse-polarity protection IRF4905 + TVS diode
* **Regulation:** dual TPS563201 buck converters — independent 3.3V and 5V 
* **Power monitoring:** INA226 current/voltage sensor (I2C) + analog voltage divider 
* **USB-C:**  USB with USBLC6-2P6 ESD protection
* **Status LED:**  WS2812 RGB
* **Servo interface:** 2× independent half-duplex UART buses , 3 legs per bus — driving serial bus servos over a single data line per bus instead of  PWM channels
* **Connectors:** 6 3-pin leg connectors (J2–J7), one per leg makes the wiring a lot easier 

**Files:**
* Schematic: 
* Gerbers + drill files : ['Electronics/manufactoring files'](Electronics/manufactoring files/)
* BOM: ['BOM/hexapod_pcb_bom.csv'](BOM/hexapod_pcb_bom.csv/)

**Board spec:**

| Spec | Value |
|---|---|
| Layers | 4 |
| MCU | ESP32-S3-WROOM-1 |
| Logic rail | 3.3V |
| Peripheral rail | 5V |
| Servo buses | 2× half-duplex UART (3 legs each) |
| Power input | XT60, [voltage range] |
| Current sensing | INA226, I2C |
