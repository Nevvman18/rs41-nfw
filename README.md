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
    * APRS
      * 1200 baud, with 1200Hz and 2200Hz tones achieved by FSK switching in a loop
      * Standard balloon and WX reporting format
    * RTTY
        * Customizable 45 and 75 baud rates, possibly other available
        * Customizable tone spacing as a multiplication of 270Hz (minimum Si4032 offset)
        * Compliant with UKHAS format
    * Morse code (CW)
        * Customizable wpm speed and dot length
        * Compliant with UKHAS format
    * PIP
        * Beacon operation, transmitting short beep with a specified interval, which could be used as a foxhunting TX device
* Thorough support of RS41 hardware, including GPS, radio, power circuitry, reference heating etc. ...and:
* Support for **onboard boom sensors**, including temperature sensors (humidity sensor incoming in next releases)
* Detailed in-built **debugging** features via LED status and serial messages
* Onboard **button** allowing user to change different operation modes and parameters 'in-flight'
* **Safety features**, including GPS watchdog and position improvement (these two improve flights in environments with interference/noise), battery voltage and temperature protection
* Support for extending hardware capabilities, including external I2C or UART sensors - including **OIF411**
* **Power saving** features, including GPS power management (together with powersaving modes and complete disable for stationary use cases) and automatic radio adjusting
* **User-friendly** firmware and IDE allows users to easily customimze the device operation
* Weather station mode, supporting APRS WX reporting
* Flight controling algorithms. Ability to rapidly send packets when below set altitude.
* And many more - mentioned in [changelog](#firmware-changelog) and [manual](./fw/OPERATION_MANUAL.md)


## Installation guide
A thorough, detailed project guide is available at the links below.<br>
**If you want to fully utilize all capabilities of this firmware**, please, read the documentation in the following header order:


## Firmware flashing
See: [fw/FLASHING.md](./fw/FLASHING.md)


## Firmware compilation
See: [fw/COMPILE.md](./fw/COMPILE.md)


## Firmware and device operation
See: [fw/OPERATION_MANUAL.md](./fw/OPERATION_MANUAL.md)
The options in the firmware file should be self explanatory (alongside with the comments near them). This manual should be up-to-date, but there may be some issues with it (it's hard to manage that long markdown file, sorry :) ). If you have **any** problems, questions and suggestions, feel free to open issues here!


## RSM414 hardware
See: [hw/README.md](./hw/README.md)


## Firmware changelog
* `v33` - added option to disable the button operation. New dataRecorder feature, that saves most important statistics from flight and transmits them in APRS comments once for a specified time (by default every 10 minutes). Flight computing abilities, that measure flight parameters (min/max values; flight, burst and landing phase detection, etc.). New ultraPowerSaveAfterLanding feature, that lowers the power consumption as much as possible 30 minutes after landing (GPS is fully OFF, sensors and peripherals get disabled) and transmits the last coordinates every 10 minutes via Horus and APRS (+ dataRecorder statistics). Added a watchdog that resets the CPU after specified time (2 weeks by default) to protect from overflowing.
* `v32` - added APRS 2 reporting modes - normal HAB tracking format and WX report format for weather station use. GPS operation modes - disabled, max performance, power saving; utilizing automatic powersaving switching to ensure best power consumption with reliable tracking. New feature, which transmits Horus packets as fast as possible (6s) at low altitudes (< 1km) for specified time when descending, to ensure that the lowest frame is captured. RTTY and Morse TX format now compliant with UKHAS formatting.
* `v31` - added APRS support (not copied), GPS no-fix timeout watchdog (resets the chip if it can't get a fix for long enough), GPS performance improvement in position gathering, 2nd Horus TX frequency (with alternating mode and repeating packest), code clean-up, other small fixes and preparations for next releases
* `v30` - added vertical velocity calculation and reporting via Horus v2. New and more accurate synchronization of delays between TX modes, fixnig issue were the delay was doubled in certain conditions.
* `v29` - final fix for GPS dynamic model setting - both old and new boards tested (new tested in-flight, old checked through u-center and will be tested in-flight soon). GPS tracking should be now trustworthy (still no verification mechanism, but shouldn't be for now a big concern).
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
* Contributors - speical thanks to:
  * My dad, for his help in catching the radiosondes and suggestions
  * videobuff - helped with lots of issues
  * darksidelemm - for help with GPS settings, Horus and sensors
  * zanco PE2BZ - for a test flight and other notes
  * Ludwik SP6MPL - for a few flights and big help in development
  * Mateusz SP3WRO - for test flights, kind words and help in development, and also his team
  * [radioszynka blog](https://radioszynka.pl/) team - for article about this project
  * PC9DB - TX formats etc.
  * KB8RCO - big thread about APRS, WX use and others
  * whallman DF7PN - much help in discussions
  * Damian SQ2DEF and his team - SQ6KXY, Paweł, Lechu, McXander, wprzyb, Michał, Hubert...
  * obi7zik
  * People from *Radiosondy Polska* group - Jarosław, Wojtek... - kind words in a local group argument


## References
* [*RS41ng* - inspiration for this project (NFW is not a fork of any kind (too much differing to be called so), just a few pieces of it's code structure is used here)](https://github.com/mikaelnousiainen/RS41ng)<br>
* [**RS41ng - issues discussion about new models and supporting them**](https://github.com/mikaelnousiainen/RS41ng/issues/92)
* [*RS41HUP* - also inspiration](https://github.com/darksidelemm/RS41HUP)<br>
* [*radiosonde_hardware* - made reversing the new version easier](https://github.com/bazjo/radiosonde_hardware)<br>
* [*Horus Binary* - awesome HAB radio protocol](https://github.com/projecthorus/horusdemodlib)<br>
* [*Arduino APRS*](https://handiko.github.io/Arduino-APRS/) library, partially utilized here to create APRS messages

## Final notes
The creator of this project isn't at all responsible for any kind of harm made by devices operated with these instructions. Follow your local law about radio transmissions and ballon flights. This device isn't a certified airplane. Altough the firmware tested successfully on a dozen of flights, it still is a 'hobbyst' project. Have a nice day!