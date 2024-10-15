
  

![poseidon_logo](https://github.com/klotzsch-lab/Maasi/blob/main/Images/render_tittle_logo.png?raw=true)

  

  

[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](https://github.com/dwyl/esta/issues)

[![DOI](https://zenodo.org/badge/385644588.svg)](https://zenodo.org/badge/latestdoi/385644588)


  

## maasi: Open source spin coater

## WARNING!!!!! This project is no longer actively maintained. 

## Proceeding to build or use this project is at your own risk and responsibility.
  

A fully 3D printed, easy-to-use and open-source spin coater.

  

  

**Build tutorial**: https://www.youtube.com/watch?v=bhJrJDhlixs

  

  

**HardwareX Paper**: In review

  

  

## The tl;dr

  

  

Here we present Maasi, an affordable Spin Coater that is easy to build and has all functional key features to be used in a wide range of applications up to 8000 RPM. Our design has a price of approximately 85 € and an assembly time of 2 hours. One of the principles that guided the design was to use only 3D printed parts and affordable Commercial Off-The-Shelf (COTS) components. In order to keep a low part count we use an electronic speed controller (ESC) with telemetry, this eliminates the need for a rotor sensor.

  
  

![using_maasi](https://github.com/klotzsch-lab/Maasi/blob/main/Images/use_animation.gif?raw=true)

  

## What is included?

  

* Computer Aided Design (CAD) files of the complete assembly.

  

* 3D printable .3MF files.

  

* Nextion Editor project for the Graphical user interface (GUI).

  

* Arduino firmware source files.

  

* Bill of materials for sourcing and purchasing materials.

  

* Detailed assembly instructions of hardware components.

  

  

## Getting Started

  

### 3D printing components and purchasing hardware

  

The 3D printed components can be fabricated on any desktop fused filament fabrication (FFF) 3D printer. They were designed using [Autodesk Fusion 360](http://autodesk.com/fusion360), a proprietary CAD software that offers free academic licenses.

  

  

- Fusion360 interactive view of [maasi spin coaterCAD](https://a360.co/3xFyf5n).

- Bill of materials with prices and vendor links on a [Spreadsheet](https://github.com/klotzsch-lab/Maasi/tree/main/doc/Maasi-BoM.ods).

- Print files (.3MF ) are available in the [/HARDWARE](https://github.com/klotzsch-lab/Maasi/tree/main/HARDWARE/) folder and print details are specified in the HardwareX publication.

  

### Build Video

Maasi v1: https://www.youtube.com/watch?v=bhJrJDhlixs&t=412s

Further build instructions are detaild in the HardwareX publication. The following is short version of those instrucitions.

### Setting up the ESP32

The ESP32 is the main MCU in Maasi. It coordinates the actions in the coating process and comunicates with the ESC and the Display MCUs.

Use the Arduino IDE to compile and flash the ESP32 (DevKitc V4)

 <strong>IMPORTANT: </strong> Users are experiencing problems (unresponsive touchscreen) with the Arduino ESP32 [v1.0.5](https://github.com/espressif/arduino-esp32/releases) and above. Please use the Board Manager to install v1.0.4 or previous until a fix is released.

**Make sure to install the following libraries**

  

-  [Nextion library](https://github.com/klotzsch-lab/Maasi/tree/main/SOFTWARE/Arduino/ITEADLIB_Arduino_Nextion). Included in this repository. To install, unzip the zip file to the libraries sub-folder of your sketchbook, if you get stuck here is a [how-to](https://www.arduino.cc/en/guide/libraries).

-  [PID_V1](https://github.com/br3ttb/Arduino-PID-Library)

-  [ElapsedMillis](https://www.arduino.cc/reference/en/libraries/elapsedmillis/)

  
  
  

### Setting up the Nextion Display

The display project was devoloped using Nextion Editor. We provide precompiled binaries if no customization is needed:

  

Copy into a micro SD card the file: [maasi_v1.tft](https://github.com/klotzsch-lab/Maasi/tree/main/SOFTWARE/Nextion/maasi_v1.tft)

Install the micro SD card in the Nextion display and power it up.

After the flashing process the SD card can be removed.

  

### Setting up the BLHeli_32 ESC

The ESC firmware is loaded using [BLHeliSuite32](https://github.com/bitdump/BLHeli/releases)

The required version for this project is the v32.7

An arduino UNO or nano is needed as flashing device between the PC and the ESC.

###  Putting together the Hardware
![circuit_diagram_maasi](https://github.com/klotzsch-lab/Maasi/blob/main/Images/circuit.png?raw=true)


## Safety Considerations

Please review the license before using/developing/distribution maasi.

  

The maasi spin coater system uses 3D printed PLA plastic and standard off the shelf components which do not pose a health hazard if handled correctly. Improper handling of plastics however can create hazards when working with organic solvents.

  

The motor can spin at very high RPM and a safety lid was added to reduce the risk of injure in case that the substrate was not properly fixed to the holder.

  

Before using maasi in a wetlab environment, please ensure that you have thoroughly tested its functionality for your use case in a dry lab. Once the maasi system has been moved into a wetlab, it should not be moved back to a drylab or dry workbench.

  

If you plan to build on the maasi spin coater, ensure that you communicate any safety concerns that you may have with your design to potential users and collaborators.

  
  
  

## Authors

  

Maasi was developed by the [Klotzsch Lab](https://www.biologie.hu-berlin.de/en/groupsites/jpexpbp) at HU Berlin by:

-  [Dani Carbonell](https://github.com/dani-carbonell)

-  Willi Weber

-  [Enrico Klotzsch](https://www.biologie.hu-berlin.de/en/groupsites/jpexpbp/members/Enrico_Klotzsch)

  

#### Prior work and references

  
  

The maasi project is based on a number of other open source projects and papers.
A complete references list can be found inside the HardwareX paper.

  
The idea to use the ESC telemetry was based on the DIY spin coater built by [@BenKrasnow](https://www.youtube.com/watch?v=321tptQ8PrU) (Applied Science in Youtube)

  
The documentation format was largely inspired by [Pachter Lab](https://pachterlab.github.io/):

[Booeshaghi, A.S., Beltrame, E.d.V., Bannon, D. et al. Principles of open source bioinstrumentation applied to the poseidon syringe pump system. Sci Rep 9, 12385 (2019). https://doi.org/10.1038/s41598-019-48815-9](https://www.nature.com/articles/s41598-019-48815-9)
