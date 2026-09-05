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

 # Electronics PCB 
<img width="718" height="768" alt="PCB TOP VIEW" src="https://github.com/user-attachments/assets/97f264cd-2c99-4b4e-a89b-87877c42cb54" />
<img width="720" height="766" alt="F cu layer" src="https://github.com/user-attachments/assets/94a67c43-43af-441d-8d24-0f5f2dc3f5fa" />
<img width="4096" height="2896" alt="image" src="https://github.com/user-attachments/assets/0999fd3b-5d95-4f35-9504-0da9727197dc" />


Custom 4-layer control board 

**Key Features :**

* **MCU:** ESP32-S3-WROOM-1 — great wifi/bluetooth MCU 
* **IMU:** BNO055  — 9-DOF orientation for balancing
* **Power input:** XT60 connector - for battery connection 
* **Power protection:** P-MOSFET reverse-polarity protection IRF4905 + TVS diode
* **Regulation:** dual TPS563201 buck converters — independent 3.3V and 5V 
* **Power monitoring:** INA226 current/voltage sensor (I2C) + analog voltage divider 
* **USB-C:**  USB with USBLC6-2P6 ESD protection
* **Status LED:**  WS2812 RGB
* **Servo interface:** 2× independent half-duplex UART buses , 3 legs per bus — driving serial bus servos over a single data line per bus instead of  PWM channels
* **Connectors:** 6 3-pin leg connectors (J2–J7), one per leg makes the wiring a lot easier 

**Files:**
* Schematic: [download the schematics](Electronics/PCB%20schematics.pdf)
* Gerbers + drill files : [Download the manufactoring files](Electronics/manufactoring%20files.zip)
* BOM: [download the PCB BOM](BOM/hexapod_pcb_bom.csv/)

**Board spec:**

| Spec | Value |
|---|---|
| Layers | 4 |
| MCU | ESP32-S3-WROOM-1 |
| Logic  | 3.3V |
| Peripheral  | 5V |
| Servo buses | 2× half-duplex UART (3 legs each) |
| Power input | XT60 |
| Current sensing | INA226 |

# CAD

Fully custom mechanical design, designed in autodesk Fusion every part is 3D-printable 

<img width="1284" height="606" alt="image" src="https://github.com/user-attachments/assets/17aa4c8e-530d-41e5-af02-332938fb3f3c" />

**values:**

| Spec | Value |
|---|---|
| size | 600 × 175 mm |
| DOF per leg | 3 (coxa – femur – tibia) |
| Total DOF | 18 |
| Coxa segment | 50 mm |
| Femur segment | 130 mm |
| Tibia segment | 150 mm |
| Actuators | LX-16A serial bus servo, 17 kg·cm torque (×18) |

Segment lengths were chosen using the servo torque and calculated to be 50x130x150mm 

**3D printing:**

| Spec | Value |
|---|---|
| Material | PETG |
| Total printed weight | ~878–950 g (all parts, with supports) |
| Parts | download [3d printing STL parts ](CAD/STL(3d%20printing%20parts)) |

PETG was chosen over PLA for better impact resistance sustained servo load — legs take repeated ground-impact stress during walking.

**Foot amortization (printed spring):**

Each foot includes a printed compliant spring element (PETG) acting as a passive shock absorber — it absorbs ground-impact impulse on footfall instead of transmitting it directly into the leg linkage/servo horn, reducing mechanical stress and jitter during walking.

**Files:**

| Format | Path | Use case |
|---|---|---|
| `.f3d` / `.f3z` | [autodesk fusion source file](CAD/source(f3d)) | Editable native Fusion source |
| `.step` | [full robot design universal file](CAD/step) | Full assembly + parts, universal format |
| `.stl` | [3d printing files](CAD/STL(3d%20printing%20parts)) | ready to slice |
| Renders | [photos of the CAD](CAD/renders) | Reference images |

Full parts list, printed vs. purchased components, and quantities: [see the main BOM](BOM/BOM%20HEXAPOD.csv)

# Bill of Materials

Two separate BOMs are provided — one for the full robot, one for the PCB only.

### Full Robot BOM

Full file: [see the full main BOM](BOM/BOM%20HEXAPOD.csv)

| Item | Qty | Description | Price |
|---|---|---|---|
| PCB | 1 | Custom PCB, UART servo controller — see [PCB BOM](#pcb-bom) below | $56.10 |
| LX-16A | 18 | Serial bus UART servo, 17 kg·cm | $252.00 |
| Battery | 1 | 6000mAh 2S, 7.4–8.4V | $27.70 |
| Auto Fuse | 1 | 30A safety fuse | $5.31 |
| XT60 | 1 | Pack of male/female connectors | $2.00 |
| PETG | 900g | Filament for 3D-printed parts (min. 900g) | — |
| M3 screws | pack | Structural assembly | — |
| M3×5 inserts | pack | Heat-set inserts, structural assembly | — |
| M2 screws | pack | Servo mounting | — |
| Wires | pack | Electronics wiring | — |
| PCB manufacturing | 1 | Bare PCB fab, green solder mask | $13.00 |
| **Total** | | *incl. VAT, delivery, components* | **$423** |

### PCB BOM

Full file: [see the full PCB BOM](BOM/hexapod%20pcb%20bom.csv)

| Ref Des | Qty | Description | Value | Footprint | Price (unit) |
|---|---|---|---|---|---|
| C1, C2, C23, C24 | 4 | Electrolytic cap | 1000µF 16V | CP_Elec 10×10mm | $2.23 |
| C3, C4, C9, C10 | 4 | Ceramic cap | 10µF 25V | 0805 | $1.21 |
| C5, C6, C11, C14, C16, C18–C22, C25 | 11 | Ceramic cap | 0.1µF 100V | 0805 | $0.91 |
| C7, C8, C12, C13, C15 | 5 | Ceramic cap | 22µF 25V | 0805 | $1.68 |
| C17 | 1 | Ceramic cap | 1µF 50V | 0805 | $1.08 |
| D1 | 1 | TVS diode | SMBJ10A 10V | SMB | $0.59 |
| D2 | 1 | RGB LED | WS2812B | PLCC-6 | $0.52 |
| J1 | 1 | Battery connector | XT60-M, PCB mount | AMASS XT60-M | $0.38 |
| J2–J7 | 6 | Servo bus connector (leg1–leg6) | 3-pin 2.5mm JST XH | JST_XH_B3B | $0.55 |
| J8 | 1 | USB-C receptacle | USB 2.0, 16-pin | HRO TYPE-C-31-M-12 | $0.86 |
| L1 | 1 | Power inductor | 3.3µH | SRN4018 | $0.59 |
| L2 | 1 | Power inductor | 2.2µH | SRN4018 | $1.76 |
| Q1 | 1 | P-MOSFET | IRF4905 | TO-263-2 | $1.10 |
| Q2, Q3 | 2 | Half-duplex transistor | 2N7002 | SOT-23 | $0.89 |
| R1 | 1 | Resistor | 100k | 0805 | $0.45 |
| R2 | 1 | Resistor | 54.9k | 0805 | $0.45 |
| R3, R4, R5, R6, R14, R15, R18–R20, R22–R24 | 12 | Resistor | 10k | 0805 | $0.45 |
| R7, R9 | 2 | Resistor | 33k | 0805 | $0.45 |
| R8 | 1 | Resistor | 51k | 0805 | $0.45 |
| R10, R12 | 2 | Resistor | 1k | 0805 | $0.45 |
| R11, R13, R25, R26 | 4 | Resistor | 4.7k | 0805 | $0.45 |
| R16, R17 | 2 | Resistor | 5.1k | 0805 | $0.45 |
| R21 | 1 | Resistor | 330R | 0805 | $0.45 |
| R27 | 1 | Shunt resistor | 2mΩ | 2512 | $2.43 |
| SW2, SW3 | 2 | Tactile button | TS-1088 | SMD | $0.64 |
| U1, U2 | 2 | Buck converter | TPS563201 | SOT-23-6 | $0.78 |
| U3 | 1 | MCU module | ESP32-S3-WROOM-1 | SMD module | $4.91 |
| U4 | 1 | ESD protection | USBLC6-2P6 | SOT-666 | $1.33 |
| U5 | 1 | 9-axis IMU | BNO055 | LGA-28 | $9.41 |
| U6 | 1 | Current/voltage monitor | INA226 | TSSOP-10 | $0.83 |
| **Total (per board)** | | *incl. LCSC + AliExpress delivery, 16% VAT, shipping* | | **$56.10** |
