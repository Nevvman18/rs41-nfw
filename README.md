# RS41-NFW - Versatile, custom firmware for all revisions of Vaisala RS41 radiosondes
## **Vaisala RS41 New Firmware** (*RS41 Nevvman's Firmware*) <br>
**NOTE:** This firmware works with the all variants of RS41 radiosondes, with the new RSM414 too, bringing full hardware and software support for everyone. More below.<br><br>
Vaisala some time ago began launching new RS41 sondes, with new internal design. They can be recognized by a last digit of 4 of the PCB model (eg. `RSM414`, `RSM424`). This firmware is an approach for reusing them as amateur devices for many different purposes. It also brings support for older models.<br>

* [Radiosondes?](#radiosondes)
* [RS41-NFW Firmware features](#rs41-nfw-firmware-features)
* [Installation guide](#installation-guide)
* [Firmware flashing](#firmware-flashing)
* [Firmware compilation](#firmware-compilation)
* [Firmware and device operation](#firmware-and-device-operation)
* [RSM414 hardware](#rsm414-hardware)
* [Firmware changelog](#firmware-changelog)
* [Authors and contributors to this branch](#authors-and-contributors-to-this-branch)
* [References](#references)
* [Final notes](#final-notes)

## Radiosondes?
These small electronic devices are used by weather instututes to perform atmospheric sounding and high altitude measurements, up to the stratosphere (HAB - high altitude balooning). After the flight, usually they are *meaningless* for the launch company, so they can be collected by people (verify this according to the certain launch site). This acvitivty is called *radiosonde hunting*<br><br>
The most simple and costless way of collecting radiosondes is to track them on sites like [radiosondy.info](https://radiosondy.info/) or [SondeHub](https://sondehub.org/) (previously HabHub). <br>
Another, more advanced way is to hunt them with radio receivers. Most of them transmit on the EU 400-406 MHz radiosonde band, near an amateur 70cm band. You can do that by a simple direction finding with a directional (for example Yagi) antenna and a handheld receiver. <br><br>
But, currently the best way is to utilize a Software-Defined Radio (SDR, for example an RTL-SDR v3 / v4, Nooelec SDR, RSP1 or HackRF) together with a 70cm band antenna (dipole should work for sondes in air as far as 100km, the best is a high gain Yagi, with this setup you could easily hear a radiosonde hundreds kilometers away) and a specialized software for a computer, laptop or a Raspberry Pi. On the internet you will find lots of tutorials for receiver setup, tracking and hunting of them.<br><br>
**Kind note:** After each hunt, either successful or not, please change the radiosonde status on the previously mentioned trakcing sites. This will not only let many people save on fuel and patience, but also allow everyone to take a look on sounding statistics and other things. On the most simple site for tracking (SondeHub) you don't even need to create an account to change the sonde status, which only takes a minute. <br>
*It's an unpleasant feeling, when after driving dozens of kilometers in search for radiosonde you come across an empty field without any ballon traces.*<br><br>
For more details about HAB and sonde hunting, please look on google and social media, there is a ton of valuable content.

## RS41-NFW Firmware features
* **Full support for ALL** RS41 versions (eg. `RSM421`, `RSM414`, `RSM424`), including the new ones with the '4' at the end of the PCB model.
* Multiple, customizable transmission modes
    * [**Horus Binary 4FSK v2**](https://github.com/projecthorus/horusdemodlib/wiki)
        * One of the most efficient radio modes for HAB and other simple telemetry designs, allows for decoding of very weak signals
    * RTTY
        * Customizable 45 and 75 baud rates, possibly other available
        * Customizable tone spacing as a multiplication of 270Hz (minimum Si4032 offset)
    * Morse code (CW)
        * Customizable wpm speed and dot length
    * PIP
        * Beacon operation, transmitting short beep with a specified interval, which could be used as a foxhunting TX device
* Thorough support of RS41 hardware, including GPS, radio, power circuitry, reference heating etc. ...and:
* Support for **onboard boom sensors**, including temperature sensors (humidity sensor incoming in next releases)
* Detailed in-built **debugging** features via LED status and serial messages
* Onboard **button** allowing user to change different operation modes and parameters 'in-flight'
* **Safety features**, including battery voltage and temperature protection
* Support for extending hardware capabilities, including external I2C or UART sensors - including **OIF411**
* **Power saving** features
* **User-friendly** firmware and IDE allows users to easily customimze the device operation


## Installation guide
A thorough, detailed project guide is available at the links below.<br>
**If you want to fully utilize all capabilities of this firmware**, please, read the documentation in the following header order:


## Firmware flashing
See: [fw/FLASHING.md](./fw/FLASHING.md)


## Firmware compilation
See: [fw/COMPILE.md](./fw/COMPILE.md)


## Firmware and device operation
See: [fw/OPERATION_MANUAL.md](./fw/OPERATION_MANUAL.md)


## RSM414 hardware
See: [hw/README.md](./hw/README.md)


## Firmware changelog
* `v28` - full compatibility of default Horus v2 payload format with RS41ng default format. Detection of external power supply (for example via programmer) disabling button functionality to prevent unpredictable device operation.
* `v27` - support for sensor boom - accurate readings of external temperature and humidity module temperature (humidity measurement not supported **yet**), Horus v2 format corrections (default payload uses the same format as RS41ng).
* `v26` - added missing feature on older RS41 (`RSM4x1`, `RSM4x2`) sondes - reference heating, which is activated by Si4032 GPIO pin.
* `v25` - powerSave mode, featuring automaticaly-adjustable TX power and transmission intervals, based on the altitude. Changed default values of the firmware.
* `v24` - some improvements in the GPS Dynamic Model setting, support for both old and new sondes, simple VALSET & VALGET validation mechanism for the new sondes. The Airborne 1G mode is still experimental!
* `v231` - fix issues of the previous *release v2.3*, thanks to [(still) this issue #3](https://github.com/Nevvman18/rs41-nfw/issues/3). Changed OIF411 data interpretation according to the operation manual (raw data division).
* `v23` - added experimental uBlox GPS Airborne 1G Dynamic Model selection via UART of the module, thanks to [this issue #3](https://github.com/Nevvman18/rs41-nfw/issues/3)
* `v22` - support for OIF411 ozone sounding system (including decoding the ozone info and transmision to ground), XDATA port mode selection (disabled, debug UART, XDATA UART), easy method of changing the payload-ID in Horus v2 mode, height activated oscillator heater
* `v21` - added support for old RS41 models (`RSM412`)
* `v20` - initial public release, releases before this were never published

<br>

Incoming features:
* External sensors - BME280 (developer of the firmware wanted to give this capability in the `v22` version, but due to some develpoment site issues this will probably be added ater on)
* More power saving features
* Fox hunting mode, featuring very low power consumption mode, transmission planning and beacon customization
* Transmission scheme planning and schedule, according to GNSS time and coordinates


## People responsible for this project
* Authors
  * Franek 'nevvman' (no callsign, *yet*)
* Contributors
  * [videobuff](https://github.com/videobuff)
  * [darksidelemm](https://github.com/darksidelemm)
  * [zanco](https://github.com/zanco)
  * and others


## References
* [*RS41ng* - inspiration for this project](https://github.com/mikaelnousiainen/RS41ng)<br>
* [**RS41ng - issues discussion about new models and supporting them**](https://github.com/mikaelnousiainen/RS41ng/issues/92)
* [*RS41HUP* - also inspiration](https://github.com/darksidelemm/RS41HUP)<br>
* [*radiosonde_hardware* - made reversing the new version easier](https://github.com/bazjo/radiosonde_hardware)<br>


## Final notes
The creator of this project isn't at all responsible for any kind of harm made by devices operated with these instructions. Follow your local law about radio transmissions and ballon flights. This device isn't a certified airplane. This firmware wasn't tested in HAB scenarios, yet. Cheers.