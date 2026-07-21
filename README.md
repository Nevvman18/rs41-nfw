# RS41-NFW - Versatile, custom firmware for ALL revisions of Vaisala RS41 radiosondes
## **Vaisala RS41 New Firmware** (*RS41 Nevvman's Firmware*)

This project is an approach for reusing [all revisions](./hw/README.md#older-vs-newer---how-do-i-know-which-one-im-holding-now) of **Vaisala RS41** radiosonde as amateur tracking and measurement devices for many different purposes - as **balloon trackers**, **atmosphere sounding** devices, weather stations, radio beacons and more. It provides advanced support of **original measurement sensors** (such as **factory-calibrated sensor boom** readings or **RPM411** pressure add-on module measurements), good algorithms and operation modes, **very advanced GPS** support and more.

> [!TIP]
> ## 🎈 Start here - do everything in your browser at [nfw.flada.ovh](https://nfw.flada.ovh)
> **This is all you need. No Arduino IDE, no toolchain, no desktop flashing tool.** The
> **NFW Sounding Software** takes you from a blank sonde to a flight-ready one, entirely in
> the browser (preferably Chromium-based). You don't even need to go through all the docs. Just do these three steps:
>
> 1. **Configure** - open the **Firmware Builder**, pick your board (`RSM4x4/4x5` or `RSM4x2/4x1`) and set your options with guided help.
> 2. **Compile** - one click builds your firmware on the server and hands you a ready `.bin`.
> 3. **Flash** - press **Flash over WebUSB** and the page writes it to your sonde through your ST-Link. A factory-new sonde is **unlocked automatically** first. You do **not** need to flash manually - the browser does it. A full flashing reference is in [FLASHING.md](./fw/FLASHING.md).
>
> The same page also has a **Ground Control**: connect the sonde over USB-serial for pre-flight checks, live sensor readout, diagnostics and a map. Wiring for both the programmer and the serial adapter is in the [flashing guide](./fw/FLASHING.md).
>
> Full description in the [RS41-NFW Sounding Software](#rs41-nfw-sounding-software) section below. Prefer to do it by hand? The [manual flashing](#firmware-flashing) and [local build](#firmware-compilation) routes are still fully documented.

* [RS41-NFW Firmware features](#rs41-nfw-firmware-features)
* [Terms of Use](#terms-of-use)
* [Radiosondes?](#radiosondes)
* [Installation guide](#installation-guide)
* [Firmware flashing](#firmware-flashing)
* [Firmware compilation](#firmware-compilation)
* [Firmware and device operation](#firmware-and-device-operation)
* [RS41-NFW Sounding Software](#rs41-nfw-sounding-software)
* [RSM425 and RSM414 hardware](#rsm425-and-rsm414-hardware)
* [Firmware changelog](#firmware-changelog)
* [Authors and contributors to this branch](#authors-and-contributors-to-this-branch)
* [References](#references)
* [Final notes](#final-notes)

## RS41-NFW Firmware features
* **Full support for ALL** RS41 versions (eg. `RSM421`, `RSM412`, `RSM414`, `RSM424`, `RSM425`), including the old and new ones with the '4' at the end of the PCB model
* **For current, up-to-date list of featurse, please refer to the [firmware builder at nfw.flada.ovh](nfw.flada.ovh), which lists all available features, together with guides and descriptions.**
* Multiple, customizable transmission modes
    * [**Horus Binary 4FSK v3**](https://github.com/xssfox/horusbinaryv3)
        * One of the most efficient radio modes for HAB and other simple telemetry designs, allows for decoding of very weak signals
        * Multi-QRG operation (ability to specify second Horus one, user can modify the code easily to get as much QRGs as wanted on every mode)
        * User can specify any payload callsign without requesting for ID number, additional sensor fields can be sent easily. Encoder provided by Mark VK5QI.
    * [**Horus Binary 4FSK v2**](https://github.com/projecthorus/horusdemodlib/wiki)
        * One of the most efficient radio modes for HAB and other simple telemetry designs, allows for decoding of very weak signals
        * Multi-QRG operation (ability to specify second Horus one, user can modify the code easily to get as much QRGs as wanted on every mode)
    * **APRS**
      * 1200 baud, with 1200Hz and 2200Hz tones, achieved by FSK switching in a precisely tuned loop (very good performance)
      * Many config options, including HAB (PSTVC structure compliant with RS41ng) and WX station reporting formats, overlay symbol modifier, SSID, destination (`APZNFW` supported and decoded by SondeHub), digi, etc.
    * RTTY
        * Customizable 45, 75 and 100 baud rates, also other available
        * Customizable tone spacing as a multiplication of 270Hz, default 570Hz
        * Data bits and stop bits customizable (default 7N2)
        * Compliant with UKHAS format. Provided by OM3BC
    * Morse code (CW)
        * Customizable wpm speed and dot length
        * Compliant with UKHAS format
    * PIP
        * Beacon operation, transmitting short beep with a specified interval
* Thorough support of RS41 hardware, including GPS, radio, power circuitry, heaters etc., timers ...and:
* **Precise atmospheric measurements from the original silver sensor boom** - external air **temperature** and **relative humidity**, read through hardware timers with custom correction algorithms, using **factory calibration** data, for fast, low-noise output!
* **Usage of original Vaisala sensor stalk calibration data** - allows to reproduce readings 1:1 of what you see when you receive one! There is also a legacy NFW calibration mode which doesn't use Vaisala calibration data
* **Simple usage** of sensor boom - just type in the serial number in the sounding software and the firmware will be compiled ready-to-launch, without the need for any hard calibration procedures!
* Automatic calibration modes - **temperature check**, **humidity check** and reconditioning - all automatic to ensure best accuracy
* **Onboard pressure sensing** via the original Vaisala **RPM411** BARO-CAP board (RS41-SGP), or a built-in pressure estimation when no sensor is fitted (RS41-SG approach)
* **Intelligent GPS management** - automatic constellation, power and tracking optimisation, u-blox Super-S, and a radio-silence cold-start booster on all boards. RSM4x4, together with NFW algorithms, provides on of the best performance in these type of GPS trackers
* **GPS-clock synchronised scheduler** - every transmission mode fires on a precise, configurable time slot aligned to the top of the hour
* Detailed in-built **debugging** features via LED status and serial messages
* Onboard **button** allowing user to shutdown the sonde
* **Safety features**, including GPS watchdog and position improvement (these two nicely improve flights in environments with interference/noise), battery voltage protection, sensors defrosting and condensation prevention, system reset watchdog
* **Power saving** features, including GPS power management (together with powersaving modes and OFF mode for stationary use cases), flight modes, after-landing powersave and advanced heating algorithms.
* **User-friendly** firmware and IDE allows users to easily customimze the device operation
* Weather station mode, supporting APRS WX report, constant coordinates
* Fox hunting mode, with CW and FM audio beacons
* Flight controlling algorithms, including the ability to rapidly send packets when below a set altitude, to capture the lowest frames on descent
* Support for extending hardware capabilities, including **XDATA port**, which allows usage of Sounding Software or external XDATA instruments (ozone interface)
* **Atmospheric ozone sounding** with the Vaisala **OIF411** ECC instrument over the XDATA port - full telemetry decoded and transmitted to ground
* **Data Recorder** - extended diagnostic telemetry (GPS quality, flight statistics, thermal/heater state) sent as extra Horus V3 pages at a set interval
* Dedicated **[RS41-NFW Sounding Software](#rs41-nfw-sounding-software)** ([nfw.flada.ovh](https://nfw.flada.ovh)) - a **browser-based firmware builder and ground station**, so no Arduino IDE or toolchain setup is needed!
* And many many more... (open the firmware builder in NFW Sounding Software to get an idea of all the available features)


## Terms of Use

<details>
<summary><b>Licensing, permitted use and disclaimer</b> - click to expand</summary>


**RS41-NFW** project (author - Franek *nevvman SP5FRA*) is released under the **GNU GPL-3.0 license**, with two clarifications new in this release (full details in [`LICENSING.md`](LICENSING.md)):

* Most of the firmware and the whole Sounding Software stay **GPL-3.0**, now with an explicit **linking exception** so the GPL code and the source-available modules below can legally share one compiled binary.
* The **sensor boom** acquisition/calibration and the **GPS readout** code are carved out as **Source-Available Modules** under the [RS41-NFW Source-Available License](LICENSE.source-available). You can read, build and use them for personal/non-commercial purposes, but not sell them or ship modified closed versions. Every covered function is marked in-source with an `// [RS41-NFW-SA]` banner.

This software is **open source** and **free to use for amateur projects**, **which cannot lead to profit** in any kind. This excludes usage of this firmware as a commercial product. **For commercial use, please contact me**. Bundled third-party code (Horus/Golay, STM32 variant files) keeps its own upstream license.
The creator of this project isn't at all responsible for any kind of harm made by devices operated with these instructions. Follow your local law about radio transmissions and balloon flights. Altough the firmware **tested successfully** on several dozen of flights, keep in mind that it is still a *hobbyst project*.

</details>


## Radiosondes?

<details>
<summary><b>What radiosondes are, and how to hunt them</b> - click to expand</summary>


These small electronic devices are used by weather instututes to perform atmospheric sounding and high altitude measurements, up to the stratosphere (HAB - high altitude ballooning). After the flight, usually they are *meaningless* for the launch company, so they can be collected by people (verify this according to the certain launch site). This acvitivty is called *radiosonde hunting*

The most simple and costless way of collecting radiosondes is to track them on sites like [radiosondy.info](https://radiosondy.info/) or [SondeHub](https://sondehub.org/) (previously HabHub). <br>
Another, more advanced way is to hunt them with radio receivers. Most of them transmit on the EU 400-406 MHz radiosonde band, near an amateur 70cm band. You can do that by a simple direction finding with a directional (for example Yagi) antenna and a handheld receiver.

But, currently the best way is to utilize a Software-Defined Radio (SDR, for example an RTL-SDR v3 / v4, Nooelec SDR, RSP1 or HackRF) together with a 70cm band antenna (dipole should work for sondes in air as far as 100km, the best is a high gain Yagi, with this setup you could easily hear a radiosonde hundreds kilometers away) and a specialized software for a computer, laptop or a Raspberry Pi. On the internet you will find lots of tutorials for receiver setup, tracking and hunting of them. Also, mobile tracking devices gain popularity, such as Lilygo TTGO boards flashed with RDZsonde (suggested) or MySondy (alternative to rdz) firmwares.

**Kind note:** After each hunt, either successful or not, please change the radiosonde status on the previously mentioned trakcing sites. This will not only let many people save on fuel and patience, but also allow everyone to take a look on sounding statistics and other things. On the most simple site for tracking (SondeHub) you don't even need to create an account to change the sonde status, which only takes a minute. <br>
*It's an unpleasant feeling, when after driving dozens of kilometers in search for radiosonde you come across an empty field without any balloon traces.*

For more details about HAB and sonde hunting, please look on google and social media, there is a ton of valuable content.

</details>


## RS41-NFW Sounding Software
A single web application that takes you from a blank sonde to a flight-ready one, entirely in the browser, at [nfw.flada.ovh](https://nfw.flada.ovh) - ready to use, nothing to install. It is the recommended way to use RS41-NFW.

The interface has **two tabs**.

### Tab 1 - Firmware Builder
Builds a ready-to-flash `.bin` without any local toolchain (no Arduino IDE, no STM32 core, no libraries).

* **Firmware source** - fetch the latest firmware straight from this GitHub repository. The page remembers which source and version is currently loaded.
* **Board selection** - pick `RSM4x4 / RSM4x5` (STM32L412) or `RSM4x2 / RSM4x1` (STM32F100). A side-by-side comparison table lists every hardware and feature difference (GPS module, radio modes, algorithms), so you know what your board supports before you build - I really recommend only to use the new RS41 revisions if possible.
* **Guided configuration** - every firmware option from `CONFIG.h` is exposed as a form with tons of manuals and guides. Irrelevant options hide themselves based on the board and your other choices.
* **Online compile** - one click compiles your configuration on the server with live build log output. When it finishes, you flash it in the browser or download the compiled `.bin`, and can **Save settings** (a small text file of the exact configuration) to reload later.
* **Settings file (local only)** - save the whole configuration to a small text file and load it back later, all in the browser. The file is generated and parsed on your machine and never uploaded, so a configuration you share stays yours. Handy for keeping a per-sonde profile or reusing a known-good setup.
* **Flashing** - after a successful build the page shows exactly what to do next: **flash from your browser** (Chromium WebUSB) with your ST-Link, flash the `.bin` with [OpenOCD](#firmware-flashing) (cross-platform terminal commands auto-matched to your MCU), or with [STM32CubeProgrammer](#firmware-flashing) (Windows/Linux GUI)

### Tab 2 - Ground Station
Connects directly to the sonde over USB (to the sonde's XDATA UART pins) and turns the pre-flight routine into a clear checklist. It uses the browser **Web Serial API**, so it runs in a Chromium-based browser (Chrome / Edge) with no driver or app to install. <br>
If you don't know how to connect the sonde to your computer via serial adapter, check the [serial connection guide](./fw/FLASHING.md#connecting-the-serial-interface-to-the-sonde).

* **Live telemetry** - decodes the compact serial frames the firmware emits: operation stage, all temperatures, humidity, pressure, diagnostics and debug, battery voltage, radio status and more.
* **Pre-flight ground check** - a multi-phase guided procedure that walks you through start-up, calibration, sensor and GPS verification, up to the *ready for flight* stage, with detailed instructions and diagnostics.
* **Sensors and charts** - inline charts for the sensor and heater values, a pressure readout, and a GPS with stats and live position pin.


## Installation guide
**The simplest path is the [NFW Sounding Software](#rs41-nfw-sounding-software) ([nfw.flada.ovh](https://nfw.flada.ovh)): configure and compile in the browser, then flash.** No more hard compilation guides.
However, if you prefer to do everything by hand, read the documentation in the following header order:


## Firmware flashing
See: [fw/FLASHING.md](./fw/FLASHING.md) <br>
Covers OpenOCD (cross-platform terminal, the recommended route), STM32CubeProgrammer (Windows/Linux, GUI and CLI), and flashing from the browser over WebUSB with an ST-Link.


## Firmware compilation
See: [fw/COMPILE.md](./fw/COMPILE.md) <br>
**Note:** with the [NFW Sounding Software](#rs41-nfw-sounding-software) you no longer need to set up Arduino IDE, the STM32 core or libraries by hand - this guide is only for those who want to build locally (for example to modify the source code - remember LICENSE).


## Firmware and device operation
See: [fw/OPERATION_MANUAL.md](./fw/OPERATION_MANUAL.md) <br>
The manual describes how the sonde behaves and how each option works. The per-option comments in [`CONFIG.h`](./rs41-nfw_sonde-firmware/CONFIG.h) are always the authoritative, up-to-date reference.


## RSM425 and RSM414 hardware
See: [hw/README.md](./hw/README.md)


## Firmware changelog
* `v72` - factory-locked RSM4x2 sondes now flash correctly from the browser, and Horus v3 stops sending disabled sensors.
  * **Locked RSM4x2 flashing fixed.** A never-unlocked RSM4x2 reported "no sonde detected on the ST-Link" even with correct wiring. [Issue 55](https://github.com/Nevvman18/rs41-nfw/issues/55) fixed.
  * **Horus v3 omits disabled sensors.** With `pressureMode 0` the pressure field was still sent as a present 0.0 hPa, which a receiver cannot tell apart from a real reading. It is now left out of the frame entirely. The data recorder packets had the same issue for pressure, main temperature, heater temperature and humidity.
  * **Firmware Builder.** Small corrections.
* `v71` - small Firmware Builder improvements and documentation improvements. APRS WX inconsistency corrected.
* `v70` - a large GNSS overhaul: the receiver is now driven entirely in the native **binary UBX protocol** (NMEA removed), for lower latency, much richer data, and no external-libraries dependency.
  * **Native UBX driver.** Both boards talk UBX only: `UBX-NAV-PVT` on the M10 (RSM4x4/4x5), the legacy `NAV-POSLLH`/`VELNED`/`SOL`/`TIMEUTC` set on the u-blox 6 (RSM4x2/4x1). The reader is event-driven and returns the instant a fresh fix arrives (no more fixed ~1 s NMEA wait). New data: true GPS-computed **vertical speed** (velD), **fix type**, horizontal/vertical/speed **accuracy estimates**, and satellite counts beyond the old NMEA cap of 12. Every config message is **ACK-validated** and retried; a rejected one is logged with its key.
  * **NFW Intelligent GNSS Algorithms** are now **available for all revisions**. On the M10 it adaptively tunes constellations and power tiers by fix strength, and also uses many uBlox algorithms and options; on the G6010 it automatically uses the chip's power-save nav mode. Configurable **cyclic-tracking period**. **Cyclic tracking (M10 PSMCT)** obeys the uBlox rules and works properly.
  * **Constellations.** The M10 runs **GPS + Galileo + SBAS** plus a selectable secondary GNSS: **BeiDou (B1C) + GLONASS** together (default - B1C shares the 1575.42 MHz L1 centre so it can coexist with GLONASS, giving the most satellites), **BeiDou (B1I) only** (older signal on 1561 MHz, more resilient to L1-band interference, but no GLONASS), or **GLONASS only**. Optional **QZSS** (regional). **Per-constellation satellite stats** are decoded from `NAV-SAT` and sent to Ground Control and the data recorder now!
  * **GPS Interference & integrity.** A raw 0-255 CW **jamming** indicator (from `UBX-MON-RF` on the M10, `UBX-MON-HW` on the u-blox 6) and **spoofing** detection (from `UBX-NAV-STATUS`).
  * **Data recorder** pages reworked: A GNSS diagnostics (gpspwr, pdop, resets, ubxerrs), B GNSS integrity (jam level, spoofing, integrity), C satellite counts, D flight stats, E thermal, then the ozone pages. RSM4x2 (single-constellation u-blox 6) skips the M10-only pages B and C.
  * **Scheduler & clock.** The transmit scheduler now locks to **sub-second GPS time** (using the NAV-PVT/NAV-TIMEUTC nanosecond fraction), so transmissions land on the GPS second rather than up to a second off. Fixes: the minimum-interval guard now holds a slot instead of skipping it; the heavy GPS read and the sensor-boom read no longer run right before a slot (they were landing transmissions seconds late); the stale NAV-PVT backlog after a transmission is flushed so the clock cannot lurch; the interface keeps sending telemetry through the pre-TX wait (no more Ground Control freeze); and a momentary post-transmission GPS glitch no longer drops the clock or triggers acquisition mode. The `flightHeatingHandler` now runs at ~1 Hz instead of every loop (it was flooding the log and starving GPS reads).
  * **Extras & fixes.** **SBAS, AssistNow Autonomous**, **better uBlox dynamic model configuration** and **satellites engine profile/filter** (Max sensitivity / Balanced / Ultra power saving), **simultaneous GNSS setup** (default on) that powers the receiver up during sensor calibration for a faster first fix, and a configurable **extra radio-quiet hold after fix** (default 20 s) so the fix can settle before the first transmission. Firmware Builder: **pre-defined settings** (Accurate / Energy-efficient), an RSM4x4-only algorithm section, and renamed GNSS options. `gpspwr` dropped from the standard Horus V3 packet (available in data recorder).
* `v69` - stability and polish on top of v68.
  * **Fixed a rare in-flight hard fault on `RSM4x2`.** The Horus V3 telemetry packet (a >1 KB ASN.1 structure) was built on the stack every transmission; on the tight 8 KB F100 this could overrun the stack into the heap and corrupt other data, hard-faulting the sonde after the first transmission. It is now built in a shared, off-stack buffer, so the fault is gone.
  * **Factory (Vaisala) calibration on `RSM4x2` / `RSM4x1`.** The older F100 boards now run the same **Vaisala factory calibration** as the new boards (the maths was moved to single precision so it fits the 64 KB flash), for manufacturer-grade temperature and humidity. They no longer use the NFW calibration model. **Estimated pressure (`pressureMode = 2`)** is now available on the F100 too.
  * **Sounding Software:** a **compilation progress bar**, the page **scrolls to the flashing section** once a build finishes, a **larger flash log**, clearer **pop-up errors** when no ST-Link or no sonde is found (with a shortcut into first-time setup), and the human-verification window is now **1 hour** (was 10 min). Also: disabling the sensor boom no longer asks for a boom serial, and a couple of stale descriptions were corrected.
* `v68` - a big tooling, flashing and licensing release. Highlights:
  * **Browser flashing (WebUSB + ST-Link).** Flash the compiled firmware straight from the Sounding Software with your ST-Link, nothing to install, on **both board families**. A **factory-new sonde is unlocked automatically** (read-out + write protections cleared) and the MCU is reset into the new firmware. ST-Link access check and steps for non-set computers are built in (though if you have flashed STM32s before you will have a working setup already - try the ST-Link veirication button), documented as [Method D](./fw/FLASHING.md#method-d---flash-from-the-browser-webusb--st-link).
  * **Guidance popups.** The builder now speaks up before you commit to a discouraged setting, and guides more throughout the configuration process. Turning the **sensor boom off now automatically disables both heaters** and says so on the heating card - to minimise misconfiguration risks.
  * **Private landing mode (RSM4x4, community-request feature).** Optionally moves every enabled mode onto its own private frequency once the sonde has flown, climbed above a threshold (default 3500 m) and descended back below it, keeping the touchdown spot off public maps. Labelling the payload "reward if returned" with your contact details is the better, friendlier choice, than hiding them from the maps.
  * **Licensing.** The firmware stays GPL-3.0 with a linking exception; the **sensor-boom, heating and GPS-readout / intelligent-GPS modules** are now source-available under the [RS41-NFW Source-Available License](LICENSE.source-available) (see [`LICENSING.md`](LICENSING.md)).
  * **Firmware fixes.** The calculated-pressure path and a couple of other spots moved from double to single-precision math, which drops the double-precision libm from the F100 build (fixes the overflow when switching pressure to *calculations*); added `-fno-strict-aliasing` as an LTO safeguard.
* `v67` - **Vaisala factory calibration mode** (now the default sensor-boom mode). A new master switch in the sensor-boom section chooses between the **Factory (Vaisala) calibration** data or the **NFW calibration algorithms**, which reproduces the boom's original factory calibration - the full temperature polynomials and the humidity matrix surface fetched from SondeHub by the measurement-boom serial number - for manufacturer-grade temperature and humidity, the same accuracy you get from a normally received factory sonde. In factory mode the two optional **self-checks** run on start-up: a **temperature check** (to ensure that temperature sensors are working properly) and a **humidity check** (one-minute reconditioning at ~138 °C, then verifies a dry reading during so-called zero-humidity check at ~135 °C), each with its own operation stage and a diagnostic flag. New **MCU internal temperature** reading on both board families, also folded into the averaged board temperature. The `$NFW` telemetry now carries the measurement mode, MCU temperature, the check flags and the boom serial, and drops unused raw fields (sensor frequencies, resistances and capacitances) for simplicity; **Ground Control auto-detects the mode**, shows the matching command set and a brand-new diagnostics panel. The battery panel shows the pack voltage and a low-voltage warning before flight. Options that apply only to RSM4x4 now show or hide automatically with the selected board, and default `buttonMode` is **0**, as Mark suggested. On new RSM4x4 / RSM4x5 both calibration modes are available and selectable. The older **STM32F100 boards (RSM4x1 / RSM4x2) run the NFW calibration only**: the factory (Vaisala) chain is not compiled there - the F100's flash and 8 KB RAM cannot hold the per-boom coefficients plus the double-precision Vaisala maths - so the Firmware Builder hides the factory option for those boards and they use the proven NFW algorithms. Of course you can compile from source and modify the braces to fit your needs. Ground Control: a separate **Power** tab, a **battery-voltage** and **MCU-temperature** chart, small **OK / FAULT** badges on the temperature and humidity charts, and a **log live-sync** (auto-scroll pauses when you scroll up). Docs improvements and stability checks.
* `v66` - **Brand-new, fully stable transmission scheduler** - rewritten from the ground up so every mode fires on its exact GPS-clock time slot, with the timing jitter of earlier versions gone. **Full, proper OIF411 ozone support** - complete decode of the Vaisala OIF411 ECC instrument over the XDATA port, with ozone partial pressure and ppbv computed and transmitted to ground. The firmware now uses the **board's external TCXO** as the radio reference, for more stable transmission frequency. **NFW Sounding Software** released ([nfw.flada.ovh](https://nfw.flada.ovh)) - a browser-based firmware builder and ground station that replaces the previous *Ground Control Software*, the *gcs_webserial* prototype and the standalone *dataRecorder decoder* (all three removed from the repo). All user settings moved to a dedicated `CONFIG.h` file, which the builder edits for you. Data Recorder reworked: it now rides dedicated Horus V3 pages (packet A - GPS quality, packet B - flight statistics, packet C - thermal/heater state) at a configurable interval, and runs on the RSM4x2 too (the older 64 KB board is now built with LTO so everything fits). New robust flight detection - the sonde declares flight after a configurable climb above the launch baseline (`flightStartClimbThreshold`, default 150 m), which works at any launch elevation and any ascent rate, replacing the old fixed-altitude trigger. Status-LED scheme clarified (5 quick green blinks = setup complete, red blinks = error). Docs overhaul.

<details>
<summary><b>Older releases</b> (<code>v65</code> and below) - click to expand</summary>


* `v65` - great code size optimisation (Horus V3 fits with all other freatures on the RSM4x2), lowering RSM4x2 memory usage down to 90%. GPS parser rewritten - now reports up to 99 satellites (no 12 GNSS SVS limits). RSM4x4 big GPS improvements - message and navigation rates optimised, intelligent GPS constellation management (user can select whether the sonde should focus on power efficiency or tracking performance, sonde automatically switches according to the signal quality), automatic GPS PSM modes, u-Blox Super-S technology, for further receiver sensitivity, trakcing performance and jamming-mitigation. GPS satellite filter optimisation - receiver now uses many more satellites for position calculation (can be sometimes over 30 sats). Final fix for overflow problems - dataRecorder is currently available in Horus V3 extraSensors (sent every 3 minutes). Others.
* `v64` - APRS WX pressure bugfix
* `v63` - Scheduler overflow issue, now program will happily run infinitely (auto reset is every 7 days). Docs.
* `v62` - Redefined sensor stalk measurements - now using **hardware timers** and completely new correction algorithms. Much faster and **much more precise** readout, with **measurement noise absent** now. Humidity measurement is now done via actual sensor capacitance measurement, which provides more accurate readings, especially high in the atmosphere. Humidity measurement should now be *somehow accurate* with great claibration (temperature is very accurate), due to new temperature as well as heating correction algorithms. New pressure estimation algorithm, based on the barometric formula. Fixed NTC thermistor R25 and beta values. Rewritten scheduler, addressing non-consistent intervals. Data recorder frames are now send via Horus V3 (apart from APRS), more in the code comments of `dataRecorder`. Pressure readings from RPM411 are now passed through a responsive, light Kalman filter. Device status and fault detection improvements. Fixed morse and RTTY payload creation issue, where comment with lower-case symbols could lock the uC. Quicker zero-humidity check and optimised calibration temperature. Minor fixes. Docs.
* `v61` - Full support for original Vaisala **RPM411** pressure sensor add-on board.
* `v60` - Improved APRS timings and baud rate consistency, with help of Wolfgang DF7PN and Ayar HB9EVW.
* `v59` - Horus Binary V3 bugfix for RSM4x2 revisions, APRS WX wind reporting correction, docs.
* `v58` - **Horus Binary 4FSK V3** support for **all** revisions of sondes. Horus Binary v2/v3 preamble improvements. Code size optimization for old models. Users aren't now required to download libraries in Library Manager - **libraries are now provided** with the downloaded sonde-firmware directory, and work out of the box! [Issue 37](https://github.com/Nevvman18/rs41-nfw/issues/37).
* `v57` - New APRS telemetry format and tocall, with much more data sent, bugfixes.
* `v56` - Introducing **Scheduler** - new CPU time management function. Introducing **RS41-NFW Ground Control Software** (later superseded by the [NFW Sounding Software](#rs41-nfw-sounding-software)), which guides you through different ground check stages and lets you inspect a lot(!) of flight parameters and sensor data. Improved sensor readout - much faster, more acurate, and with slightly reduced oscillations. Support for infinite number of cycling Horus and APRS frequencies. Redefined heating algorithms - now based on precise PID control and PWM, also with new in-flight contrl, allowing for sensor defrosting, while having no impact on the radings. New humidity correction algorithms, based on empirical data from many previous flights. Faster program operation. New LED status messages. Redefined documentation - operation manual markdown file became outdated, but new detailed code comments are here.
* `v55` - Si4032 radio now consumes 10-15mA less current than before, due to a misunderstanding of operation stages, allowing now to sleep properly between transmissions. Ability to specify TX power in each mode. Removed power save mode which changed interval and power below a certain altitude, which was a clone of a newer feature called `lowAltitudeFastTx`. Ability yo specify constant altitude when stationary use with no GPS. Others. + support of the newest `RSM425` boards.
* `v54` - power OFF routine modification, making the procedure status more clear. APRS temperature reporting length improvement.
* `v53` - timing improvements down to milliseconds, with help of Bernd DL1XH. More talkative UART debug messages.
* `v52` - heating bug with time management, APRS format improvement
* `v51` - small corrections and improvements
* `v50` - issue [20 fix](https://github.com/Nevvman18/rs41-nfw/issues/20) corrected description. Improved accuracy of both temperature and humidity reading based on empirical tests. Added optional reconditioning phase before zero-humidity check. Improved reference area heating and humidity sensor heating control algorithms, allowing to run more smoothly and precisely. More customization options.
* `v49` - issue [18 fix](https://github.com/Nevvman18/rs41-nfw/issues/18). Si4032 temperature measurement change, to accurately measure negative temperatures. Option to calibrate temperature sensors using: manual offset setting, automatic calibration with entered air temperature and automatic calibration based on PCB temperature. Sensor boom diagnostics improvement. Vertical velocity calculation algorithm returns 0 if no fix. Default settings changed slightly. Stability and code structure improvements. Repo versioning fix.
* `v48` - RTTY format, timing and customization bugfix and enhancements by OM3BC ([issue 15](https://github.com/Nevvman18/rs41-nfw/issues/15)), firmware crash fix ([issue 16](https://github.com/Nevvman18/rs41-nfw/issues/16)), APRS coordinate bugfix by KB8RCO ([issue 17](https://github.com/Nevvman18/rs41-nfw/issues/17)). Improvement in APRS timings.
* `v47` - watchdog bugfix, which could to a sudden restart of GPS during flight (once every 30 minutes), due to a wrong condition check. Fixed and no problem.
* `v46` - GPS bugfix, previous flights shouldn't have been affected
* `v45` - pressure estimation can now be enabled/disabled. Also added what was forgotten - user input of sea level pressure.
* `v44` - humidity readings accuracy improvement, temperature compensation improvement, GPS abnormalities detection with sending to ground in dataRecorder APRS extended packets, HDOP value in dataRecorder packets, other minor improvements
* `v43` - minor improvements
* `v42` - dataRecorder functionality improvement, which additionally sends power of all 3 heaters, temperatures of radio and references area and battery voltage in mV
* `v41` - PWM humidity sensor heating, just like in the Vaisala firmware, which maintains the humidity module temperature ~5K above air temperature (by default). References heating improvement, which maintains the temperature more stabily and using 3 different power levels for power saving and efficiency. Improved calibration function based on variable heating power, speeding up the process. Pressure estimation (like in RS41-SG models?, here based on altitude, temperature and RH) sent via Horus and APRS WX packets. APRS coordinates conversion script adjustment using the standard APRS coordinate encoding. 
* `v40` - now the `referenceHeating` option works like in the Vaisala firmware, maintaing temperature around 18*C.
* `v39` - performance improvements, minor changes
* `v38` - bugfix for delay variable initialization, which could lead to overflow if set at 65 seconds or higher
* `v37` - fox hunting mode, which can transmit a melody over NFM, CW pip, CW morse with additional message contents of location below voltage warning and battery voltage reporting, this mode operates in as low power consumption as possible (measurement circuits disabled, heaters disabled, GPS disabled, radio sleeping between transmissions). The gpsSpeed value in Horus v2 payload is now reported in km/h (instead of m/s as before). Small changes in startup and calibration procedure.
* `v36` - changes in button operation algorithm - user can select between completely disabled button functions, sonde shutdown on button press or extended mode, which allows to change some parameters of the sonde. The button operation is available now throughout the whole operation of the sonde, in some limited areas (like calibrartion) the button can shutdown the sonde now.
* `v35` - fixed boom calibration issue, which could lead to worse humidity compensation. Enhanced defrosting algorithm, that doesn't impact the readings.
* `v34` - support for onboard humidity sensor reading (not the most accurate method, but gives reasonable readings with easy use). Humidity module calibration modes and auto-compensation (some parts of it known as zero-humidity check). Defrosting features. Reference resistors warming to maintain their temperature. Automatic temperature compensation for humidity module sensor readings. Depreciated oscillator heating capabilities - RTTY isn't widely used, especiallu at 75-baud and the option was very power hungry - now the resistors are used like from factory, to slightly warm up the reference resistors and capacitor.
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

</details>


## People responsible for this project
* Authors
  * **Franek SP5FRA 'nevvman'**
* Contributors - speical thanks to:
  * My dad, for his help in catching the radiosondes and suggestions
  * Erik PA0ESH - helped with lots of issues
  * Mark VK5QI - for help with GPS settings, Horus and sensors. New Horus v3 decoder.
  * zanco PE2BZ - for a test flight and other notes
  * Ludwik SP6MPL - for a few flights and big help in development
  * Mateusz SP3WRO - for test flights, kind words and help in development, and also his team
  * [radioszynka blog](https://radioszynka.pl/) team - for article about this project
  * Mark PC9DB - TX formats etc.
  * KB8RCO - big thread about APRS (also code to fix coordinate issue), WX use and others
  * whallman DF7PN - much help in discussions, frameCounter fix
  * Damian SQ2DEF and his team - SQ6KXY, Paweł, Lechu, McXander, wprzyb, Michał, Hubert...
  * obi7zik - thread about GPS power mode issue
  * OM3BC - big help in getting the RTTY to finally have a proper format and timings
  * maxkup14 - suggested accuracy of sensor measurements is affected by GPS (to be investigated) and successfully tested the RS41-NFW with 2025 `RSM425` revision!
  * People from *Radiosondy Polska* group - Jarosław, Wojtek... - kind words in a local group argument
  * People from horus_flights group - I can't remember all of you!
  * If I have missed anyone - sorry, please contact me


## References
* [*RS41ng* - an inspiration for this project](https://github.com/mikaelnousiainen/RS41ng). RS41-NFW is **not** a fork and does **not** reuse code that originates from RS41ng - the two projects took inspiration from each other's ideas, but the RS41-NFW hardware sequences (radio, GPS, sensor boom) are implemented independently from the manufacturers' datasheets and reference material. Where formats are described as "compatible with RS41ng" it means on-air/wire-format interoperability, not shared source code.<br>
* [**RS41ng - issues discussion about new models and supporting them**](https://github.com/mikaelnousiainen/RS41ng/issues/92)
* [*RS41HUP* - also inspiration](https://github.com/darksidelemm/RS41HUP)<br>
* [*radiosonde_hardware* - made reversing the new version easier](https://github.com/bazjo/radiosonde_hardware)<br>
* [*Horus Binary* - awesome HAB radio protocol](https://github.com/projecthorus/horusdemodlib)
