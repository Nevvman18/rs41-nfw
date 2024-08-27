# RS41-NFW - Versatile, custom firmware for new revision of Vaisala RS41 radiosondes
**NOTE:** This firmware currently only works with the new variant of RS41 radiosondes. More below.<br><br>
Vaisala some time ago began launching new RS41 sondes, with new internal design. They can be recognized by a last digit of 4 of the PCB model (eg. `RSM414`, `RSM424`). This firmware is an approach for reusing them as amateur devices for many different purposes.<br>

[Radiosondes?](radiosondes)
[RS41-NFW Firmware features](rs41-nfw-firmware-features)
[Installation guide](installation-guide)
[Firmware flashing](firmware-flashing)
[Firmware compilation](firmware-compilation)
[Firmware and device operation](firmware-and-device-operation)
[RSM414 hardware](rsm414-hardware)
[Authors and contributors to this branch](authors-and-contributors-to-this-branch)
[References](references)

## Radiosondes?
These small electronic devices are used by weather instututes to perform atmospheric sounding and high altitude measurements, up to the stratosphere (HAB - high altitude balooning). After the flight, usually they are *meaningless* for the launch company, so they can be collected by people (verify this according to the certain launch site). This acvitivty is called *radiosonde hunting*<br><br>
The most simple and costless way of collecting radiosondes is to track them on sites like [radiosondy.info](https://radiosondy.info/) or [SondeHub](https://sondehub.org/) (previously HabHub). <br>
Another, more advanced way is to hunt them with radio receivers. Most of them transmit on the EU 400-406 MHz radiosonde band, near an amateur 70cm band. You can do that by a simple direction finding with a directional (for example Yagi) antenna and a handheld receiver. <br><br>
But, currently the best way is to utilize a Software-Defined Radio (SDR, for example an RTL-SDR v3 / v4, Nooelec SDR, RSP1 or HackRF) together with a 70cm band antenna (dipole should work for sondes in air as far as 100km, the best is a high gain Yagi, with this setup you could easily hear a radiosonde hundreds kilometers away) and a specialized software for a computer, laptop or a Raspberry Pi. On the internet you will find lots of tutorials for receiver setup, tracking and hunting of them.<br><br>
**Kind note:** After each hunt, either successful or not, please change the radiosonde status on the previously mentioned trakcing sites. This will not only let many people save on fuel and patience, but also allow everyone to take a look on sounding statistics and other things. On the most simple site for tracking (SondeHub) you don't even need to create an account to change the sonde status, which only takes a minute. <br>
*It's an unpleasant feeling, when after driving dozens of kilometers in search for radiosonde you come across an empty field without any ballon traces.*<br><br>
For more details about HAB and sonde hunting, please look on google and social media, there is a ton of valuable content.

## RS41-NFW Firmware features
* Multiple, customizable transmission modes
    * [Horus Binary 4FSK v2](https://github.com/projecthorus/horusdemodlib/wiki)
        * One of the most efficient radio modes for HAB and other simple telemetry designs, allows for decoding of very weak signals
    * RTTY
        * Customizable 45 and 75 baud rates, possibly other available
        * Customizable tone spacing as a multiplication of 270Hz (minimum Si4032 offset)
    * Morse code (CW)
        * Customizable wpm speed and dot length
    * PIP
        * Beacon operation, transmitting short beep with a specified interval, which could be used as a foxhunting TX device
* Thorough support of RS41 hardware, including GPS, radio, power circuitry, reference heating etc.
* Detailed in-built debugging features via LED status and serial messages
* Onboard button allowing user to change different operation modes and parameters 'in-flight'
* Safety features, including battery voltage and temperature protection
* Support for extending hardware capabilities, including external I2C or UART sensors (coming soon to the fw)
* User-friendly firmware and IDE allows users to easily customimze the device operation

## Installation guide
A thorough, detailed project guide is available at the links below to other .MD documents.<br>
**If you want to fully utilize all capabilities of this firmware**, please, read the documentation in the following header order:

## Firmware flashing
See: [fw/FLASHING.md](./fw/FLASHING.md)

## Firmware compilation
See: [fw/COMPILE.md](./fw/COMPILE.md)

## Firmware and device operation
See: [fw/OPERATION_MANUAL.md](./fw/OPERATION_MANUAL.md)

## RSM414 hardware
See: [hw/README.md](./hw/README.md)

## Authors and contributors to this branch
* Franek 'nevvman' (no callsign)

## References
* [*RS41ng* - inspiration for this project](https://github.com/mikaelnousiainen/RS41ng)<br>
* [*RS41HUP* - also inspiration](https://github.com/darksidelemm/RS41HUP)<br>
* [*radiosonde_hardware* - made reversing the new version easier](https://github.com/bazjo/radiosonde_hardware)<br>