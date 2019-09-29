# Iridium_Mini_Radio_Beacon


An 80g version of the [Iridium 9603N Beacon](https://github.com/PaulZC/Iridium_9603_Beacon) with eRIC4/9 radio support
.

Suitable for high altitude ballooning and many other asset tracking applications.

![Mini_Beacon](https://github.com/PaulZC/Iridium_Mini_Radio_Beacon/blob/master/img/Mini_Beacon.JPG)

This version is powered by a single Energiser® Ultimate Lithium L522 (PP3) 9V battery, which helps keep the design as compact as possible.

If you want to keep the size and weight as low as possible, and don't need a radio link, have a look at
the [Iridium 9603N Mini Beacon](https://github.com/PaulZC/Iridium_9603N_Mini_Beacon) instead.

The Iridium 9603N transceiver and u-blox MAX-M8Q GNSS receiver share a single Maxtena M1600HCT-P-SMA antenna, again to keep the weight as low as possible.
The antenna sharing circuit has been flight tested on the [Iridium 9603N Solar Beacon](https://github.com/PaulZC/Iridium_9603N_Solar_Beacon).

Full details of how to track your beacon via the Rock7 RockBLOCK Gateway are included in the
[Iridium 9603N Beacon repo](https://github.com/PaulZC/Iridium_9603_Beacon/blob/master/RockBLOCK.md).

[ASSEMBLY.md](https://github.com/PaulZC/Iridium_Mini_Radio_Beacon/blob/master/ASSEMBLY.md) describes how to assemble the PCB.

The [Arduino folder](https://github.com/PaulZC/Iridium_Mini_Radio_Beacon/tree/master/Arduino) contains the code for the SAMD21 processor.
You can find the [programming instructions](https://github.com/PaulZC/Iridium_9603_Beacon/blob/master/LEARN.md#how-do-i-install-the-atsamd21g18-bootloader)
and [upload instructions](https://github.com/PaulZC/Iridium_9603_Beacon/blob/master/LEARN.md#how-do-i-upload-the-arduino-code) in the Iridium 9603N Beacon repo.

The Eagle files are available in the [Eagle folder](https://github.com/PaulZC/Iridium_Mini_Radio_Beacon/tree/master/Eagle).

The schematic, dimensions and BOM are available [here](https://github.com/PaulZC/Iridium_Mini_Radio_Beacon/blob/master/Iridium_Mini_Radio_Beacon.pdf).

The [OpenSCAD folder](https://github.com/PaulZC/Iridium_Mini_Radio_Beacon/tree/master/OpenSCAD) contains .scad and .stl files for the 3D-printed cover.

## Licence

This project is distributed under a Creative Commons Attribution + Share-alike (BY-SA) licence.
Please refer to section 5 of the licence for the "Disclaimer of Warranties and Limitation of Liability".

Enjoy!

**_Paul_**


