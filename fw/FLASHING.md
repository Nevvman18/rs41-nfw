# Firmware flashing

This page explains how to upload an RS41-NFW `.bin` to a Vaisala RS41 radiosonde, and how to connect the sonde after flashing to a serial converter.

> **You usually don't need this page.** The **[NFW Sounding Software](../README.md#rs41-nfw-sounding-software)** ([nfw.flada.ovh](https://nfw.flada.ovh)) can flash the firmware it just built **straight from your browser** over WebUSB with your ST-Link - see [Method D](#method-d---flash-from-the-browser-webusb--st-link). It even unlocks a factory-new sonde for you. This page is the full reference and the manual alternatives for when you prefer a desktop tool or your browser cannot do WebUSB.


You only need a `.bin` file and an ST-Link programmer. There are several ways to flash, all covered below; pick one:

* [Requirements](#requirements)
* [Connecting the programmer to the sonde](#connecting-the-programmer-to-the-sonde)
* [Connecting the serial interface to the sonde](#connecting-the-serial-interface-to-the-sonde)
* [Method A - OpenOCD (terminal, cross-platform)](#method-a---openocd-terminal-cross-platform)
* [Method B - STM32CubeProgrammer (GUI, Windows / Linux)](#method-b---stm32cubeprogrammer-gui-windows--linux)
* [Method C - STM32CubeProgrammer CLI (terminal)](#method-c---stm32cubeprogrammer-cli-terminal)
* [Method D - Flash from the browser (WebUSB + ST-Link)](#method-d---flash-from-the-browser-webusb--st-link)
* [Uploading directly from the Arduino IDE](#uploading-directly-from-the-arduino-ide)


<br>

> **Which method should I pick?** **Method D (browser / WebUSB)** is the recommended path for most people - it flashes from the ([Sounding Software](https://nfw.flada.ovh)) with nothing to install and unlocks a factory sonde automatically. The command-line and GUI methods (A/B/C) are the desktop alternatives: use them if you prefer them, or if your browser cannot use WebUSB.

<br>

Whichever method you use, the procedure for a sonde is always the same three ideas:
1. **Unlock** the factory protection (read-out protection + write protection). Only needed once in the sonde's life.
2. **Erase** the chip.
3. **Write** the new `.bin` to flash address `0x08000000` and start it.


## Requirements

* **Hardware**
  * A Vaisala RS41 sonde (this firmware supports [all versions](../hw/README.md#older-vs-newer---how-do-i-know-which-one-im-holding-now)).
  * An ST-Link compatible SWD programmer. A cheap ST-Link V2 clone works fine. Any programmer that speaks ST's SWD protocol will do.
  * A few jumper wires (often included with the programmer).
  * A Windows or Linux computer (macOS should also work).
* **Software** (choose the method you will use)
  * [OpenOCD](https://openocd.org/) - the popular open-source on-chip debugger / programmer (the same tool the rs41ng project documents). Cross-platform and available straight from most package managers. This is the method the NFW Sounding Software hands you to copy after a build (Method A).
  * Or [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) - the official ST tool, available for Windows, Linux and macOS. It ships with both a GUI (Method B) and a command-line tool `STM32_Programmer_CLI` (Method C).

> **Which MCU is on my board?** The simplest way is to read the model printed on the bottom of the PCB - it is marked `RSM4x` followed by the revision digit (e.g. `RSM425`, `RSM414`, `RSM412`). The MCU also decides the option-byte values used when unlocking:
> * `RSM4x2` / `RSM4x1` (older boards) - **STM32F100C8** (64 KB flash).
> * `RSM4x4` / `RSM4x5` (newer boards) - **STM32L412RBT6** (128 KB flash).
>
> Not sure which one you hold? See [hw/README.md](../hw/README.md#older-vs-newer---how-do-i-know-which-one-im-holding-now).

**Note:** pre-compiled binaries are not shipped with releases, because of the per-sonde calibration and the large number of options. Build your `.bin` with the [NFW Sounding Software](../README.md#rs41-nfw-sounding-software) or [locally](./COMPILE.md). Factory-new sondes still running the original Vaisala firmware must be [unlocked](#method-a---openocd-terminal-cross-platform) once before they accept any new firmware.


## Connecting the programmer to the sonde

Connect the programmer to the sonde's XDATA connector.

The connector has a pinout like this:
```
Pinout of XDATA connector                                    Pinout of connected ribbon cable
                 -------                                     
           GND  | o   o |  XDATA_RX(PB11)                    > 1  GND
                |       |                                      2  XDATA_RX(PB11)
XDATA_TX(PB10)  | o   o |  +3V_MCU                             3  XDATA_TX(PB10) 
               -        |                                      4  +3V_MCU
       V_Boost|   o   o |  VBAT                                5  V_Boost
               -        |                                      6  VBAT
       MCU_RST  | o   o |  SWCLK(PA14)                         7  MCU_RST
                |       |                                      8  SWCLK(PA14)
   SWDIO(PA13)  | o   o |  GND                                 9  SWDIO(PA13)
                 -------                                      10  GND

PAY ATTENTION TO THE CUT-OUT
```

**Note:** the RS41 XDATA connector has a non-standard 2 mm pitch (instead of the usual 2.54 mm). For only four wires, jumper cables push in well enough that you do not need a dedicated 2 mm connector.

Wire it as follows:
* RS41-**GND** -> STLink-**GND** (RS41-GND is on XDATA pin 1 and 10)
* RS41-**SWCLK** -> STLink-**SWCLK** (RS41-SWCLK is on XDATA pin 8; sometimes labelled CLK)
* RS41-**SWDIO** -> STLink-**SWDIO** (RS41-SWDIO is on XDATA pin 9; sometimes labelled DIO)
* Powering the sonde:
  * If your programmer can supply a stable 3.3 V at 200 mA or more (most cheap clones can):
    * **Take both batteries out.**
    * Connect RS41-**+3V_MCU** -> STLink-**+3.3V** (RS41-+3V_MCU is on XDATA pin 4).
  * If not, leave both batteries in, do **not** connect any programmer power pin to the XDATA power pins, and power the sonde on with its button before flashing.

The system should now be ready. If you have trouble at this stage, double-check the wiring and the programmer selection before going further.


## Connecting the serial interface to the sonde

To use the **RS41-NFW Sounding Software** capabitilies for **Ground Control and preparation**, after successful firmware flashing you need to connect a serial adapter to the sonde's XDATA port.

The connector has a pinout like this:
```
Pinout of XDATA connector                                    Pinout of connected ribbon cable
                 -------                                     
           GND  | o   o |  XDATA_RX(PB11)                    > 1  GND
                |       |                                      2  XDATA_RX(PB11)
XDATA_TX(PB10)  | o   o |  +3V_MCU                             3  XDATA_TX(PB10) 
               -        |                                      4  +3V_MCU
       V_Boost|   o   o |  VBAT                                5  V_Boost
               -        |                                      6  VBAT
       MCU_RST  | o   o |  SWCLK(PA14)                         7  MCU_RST
                |       |                                      8  SWCLK(PA14)
   SWDIO(PA13)  | o   o |  GND                                 9  SWDIO(PA13)
                 -------                                      10  GND

PAY ATTENTION TO THE CUT-OUT
```

Wire it as follows:
* RS41-**GND** -> UART adapter **GND** (RS41-GND is on XDATA pin 1 and 10)
* RS41-**XDATA_RX** -> UART adapter **TX pin** (RS41-XDATA_RX is on XDATA pin 2)
* RS41-**XDATA_TX** -> UART adapter **RX pin** (RS41-XDATA_TX is on XDATA pin 3)
* Power the sonde via system power - for example with 2xAA batteries (as the sonde could for a while draw a larger current during reconditioning phase).

The Ground Control should now be ready and the serial adapter after connecting shoud be visible in conenction settings in NFW Sounding Software.


## Method A - OpenOCD (terminal, cross-platform)

[OpenOCD](https://openocd.org/) is a popular open-source programmer. This is exactly what the [NFW Sounding Software](../README.md#rs41-nfw-sounding-software) gives you to copy after a build, and it works the same on Linux, macOS and Windows.

Install it from your package manager, for example on Debian / Ubuntu:
```bash
sudo apt install openocd
```

Run the commands from the folder where you saved the `.bin`, and replace `FIRMWARE.bin` with the file you downloaded. OpenOCD resets the target between invocations on its own, so there is nothing to unplug or power-cycle by hand. All RS41 MCUs flash to `0x08000000`.

**Older boards (`RSM4x2` / `RSM4x1`, STM32F100):**
The STM32F100C8 has 64 KB of flash, which is pages 0...63 (1 KB each).
```bash
# 1. Clear write protection on every flash page
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "init; halt; flash protect 0 0 63 off; exit"

# 2. Flash, verify and start the firmware
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program FIRMWARE.bin verify reset exit 0x08000000"
```

**Newer boards (`RSM4x4` / `RSM4x5`, STM32L412):**
On the L412, `stm32l4x unlock 0` drops read-out protection to level 0, which mass-erases a factory-locked chip.
```bash
# 1. Drop read-out protection to level 0 (mass-erases a locked chip)
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "init; halt; stm32l4x unlock 0; exit"

# 2. Clear write protection on all sectors
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "init; halt; flash protect 0 0 last off; exit"

# 3. Flash, verify and start the firmware
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "program FIRMWARE.bin verify reset exit 0x08000000"
```

Notes:
* `interface/stlink.cfg` selects an ST-Link programmer; `target/stm32f1x.cfg` / `target/stm32l4x.cfg` select the MCU family. These config files ship with OpenOCD.
* `program ... verify reset exit 0x08000000` writes the `.bin`, verifies it, resets the chip so the firmware starts, then exits OpenOCD.
* The protection-clearing steps are the one-time unlock. Once a sonde is unlocked you only need the final `program` step for future updates.
* If OpenOCD cannot attach because the chip is running or protected, add `-c "reset_config srst_only srst_nogate"` before the action, or briefly hold the sonde in reset while it connects.


## Method B - STM32CubeProgrammer (GUI, Windows / Linux)

The friendliest GUI option, and a convenient way to change the option bytes by hand on a factory-new sonde.

### A.1 Connect

* Start STM32CubeProgrammer. By default you should see the [main window](./photos/main_window.png).
* Power the sonde on, then select **ST-LINK** as the programmer and click **Connect** ([screenshot](./photos/stlink-connection.png)).
  * If the connection is refused because of read-out protection, set the connection **mode** to **Under reset** (top-right of the ST-LINK panel) and connect again.

### A.2 Unlock the factory protection (one time per sonde)

Open the **Option bytes** (OB) page and set the values for your MCU:

* **Older boards (`RSM4x2` / `RSM4x1`, STM32F100)** - see [this picture](./photos/mcu-unlock-rsm412.png):
  * **Read Out Protection**: set `RDP` to **unchecked / disabled** (level 0).
  * **Write Protection**: **check every** box (`WRP0`...`WRP3`) so no sector is write-protected.
* **Newer boards (`RSM4x4` / `RSM4x5`, STM32L412)** - see [this picture](./photos/mcu-unlock-rsm414.png):
  * **Read Out Protection**: set the `RDP` byte to `AA` (level 0).
  * **Write Protection** (an empty region = no protection, so set start above end):
    * `WRP1A_STRT` -> value `0x1`
    * `WRP1A_END`  -> value `0x0`
    * `WRP1B_STRT` -> value `0x1`
    * `WRP1B_END`  -> value `0x0`
* Click **Apply**.

Then open **Erasing & Programming** ([page](./photos/erase_and_upload.png)) and run a **Full chip erase**. This clears the memory and finishes removing the protection. If you still see RDP / access errors, go back to the Option bytes page and make sure RDP is at level 0.

The MCU is now unlocked and stays unlocked for the rest of its life.

### A.3 Flash the firmware

* On the **Erasing & Programming** page, select your `.bin` in the **File path** field (make sure it is the build for *your* board).
* Tick **Verify programming**, and make sure **Skip flash erase before programming** is **unchecked**.
* Set the start address to `0x08000000` (the default).
* Click **Start Programming**. After a few seconds the firmware is written and verified.
* Disconnect everything, insert the batteries, and continue with the [Operation manual](./OPERATION_MANUAL.md). The sonde stays off until you turn it on with the button.


## Method C - STM32CubeProgrammer CLI (terminal)

The official ST command-line alternative to OpenOCD, driven with `STM32_Programmer_CLI`. Same tool as Method B's GUI, and a good fallback if you already have STM32CubeProgrammer installed. It works identically on Windows and Linux.

The command is named `STM32_Programmer_CLI` (on Windows `STM32_Programmer_CLI.exe`). If it is not on your `PATH`, call it by full path, e.g. on Linux it is usually:
`~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI`.

Replace `FIRMWARE.bin` with the file you downloaded. All RS41 MCUs flash to `0x08000000`.

**Older boards (`RSM4x2` / `RSM4x1`, STM32F100):**
```bash
# 1. Remove read-out protection (this mass-erases a protected chip)
STM32_Programmer_CLI -c port=SWD mode=UR -rdu

# 2. Clear write protection on all sectors (0xFF = unprotected)
STM32_Programmer_CLI -c port=SWD mode=UR -ob WRP0=0xFF WRP1=0xFF WRP2=0xFF WRP3=0xFF

# 3. Mass-erase the flash
STM32_Programmer_CLI -c port=SWD mode=UR -e all

# 4. Write the firmware, verify it, and start it
STM32_Programmer_CLI -c port=SWD mode=UR -w "FIRMWARE.bin" 0x08000000 -v -rst
```

**Newer boards (`RSM4x4` / `RSM4x5`, STM32L412):**
```bash
# 1. Remove read-out protection (this mass-erases a protected chip)
STM32_Programmer_CLI -c port=SWD mode=UR -rdu

# 2. Clear write protection on all flash (empty WRP region: start above end)
STM32_Programmer_CLI -c port=SWD mode=UR -ob WRP1A_STRT=0x1 WRP1A_END=0x0 WRP1B_STRT=0x1 WRP1B_END=0x0

# 3. Mass-erase the flash
STM32_Programmer_CLI -c port=SWD mode=UR -e all

# 4. Write the firmware, verify it, and start it
STM32_Programmer_CLI -c port=SWD mode=UR -w "FIRMWARE.bin" 0x08000000 -v -rst
```

Notes:
* `mode=UR` means "connect under reset", which is the most reliable way to attach to a protected or running chip.
* `-rdu` is read-unprotect: on a factory-locked sonde it drops RDP and triggers a mass erase. On an already-unlocked sonde it is harmless.
* Steps 1 and 2 are the one-time unlock. Once a sonde is unlocked you only need steps 3 and 4 for future updates.
* `-rst` issues a system reset over SWD so the firmware starts immediately after flashing.


## Method D - Flash from the browser (WebUSB + ST-Link)

The [NFW Sounding Software](../README.md#rs41-nfw-sounding-software) can drive your ST-Link straight from the browser over **WebUSB**, so you can flash the `.bin` it just built without installing OpenOCD or STM32CubeProgrammer. Under the hood it speaks the same ST-Link protocol as the other methods and writes to the same `0x08000000`; the only difference is that the browser talks to the programmer instead of a desktop tool.

### What you need

* An **ST-Link V2 / V2-1 / V3** wired to the sonde exactly as in [Connecting the programmer to the sonde](#connecting-the-programmer-to-the-sonde). The wiring is identical to every other method.
* A **WebUSB-capable browser on desktop**: Google Chrome, Microsoft Edge or Opera. For example Firefox does not implement WebUSB and will not work.
* The **operating-system access** for the browser to open the ST-Link as a raw USB device (below). This is the one extra step compared to the desktop tools, and it is a one-time setup per computer.
  * **Chances are you don't need to do any changes to the system** - many people already have a working system (if they played with STM32s). To verify this, please compile a firmware, and after plugging the ST-Link, click the button to check the connection. 

### Windows: give the browser access to the ST-Link (WinUSB via Zadig)

On Windows the ST-Link normally binds to ST's own driver, which the browser cannot open. You need to attach the generic **WinUSB** driver to it once, using [Zadig](https://zadig.akeo.ie/):

1. Plug in the ST-Link (leave the sonde wiring as-is).
2. Run **Zadig**. Choose **Options -> List All Devices**.
3. In the dropdown pick **STM32 STLink** (interface with USB ID `0483`/`3748` for a V2, or `0483`/`374B` for a V2-1).
4. Set the target driver on the right to **WinUSB** and click **Replace Driver** (or *Install Driver*).
5. Reconnect the ST-Link. The browser can now see it.

> Swapping to WinUSB means STM32CubeProgrammer / the ST-Link GUI may no longer see this ST-Link until you restore ST's driver (Zadig -> pick the ST driver again, or reinstall the ST-Link driver). HOWEVER - when I have tested, while having the STM32Cube installed, the ST's drivers actually support WebUSB flashing - therefore just click the Check ST-Link Access button after compiling.

### Linux: allow non-root access (udev rule)

On Linux WebUSB works once your user is allowed to open the ST-Link. Add a udev rule:

```bash
# /etc/udev/rules.d/49-stlink-webusb.rules
# ST-Link V2
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="0660", TAG+="uaccess"
# ST-Link V2-1
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374b", MODE="0660", TAG+="uaccess"
# ST-Link V3
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374f", MODE="0660", TAG+="uaccess"
```

Then reload the rules and reconnect the programmer:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

No kernel module needs to be unloaded: the `stlink`/`st-link` modules do not claim the device, so the browser can open it directly.

### macOS

No driver or permission setup is needed; a WebUSB browser can open the ST-Link directly. Just wire it up and connect.

### Board support

Both board families flash in the browser:

* **`RSM4x2` / `RSM4x1` (STM32F100)** - via the `webstlink` library's STM32FP driver.
* **`RSM4x4` / `RSM4x5` (STM32L412)** - via an STM32L4 flash + unlock driver added for RS41-NFW (upstream `webstlink` has none).

> **Experimental.** The L4 driver and the automatic unlock below were written against the STM32 reference manuals and are newer than the desktop methods. They are recoverable (SWD stays open), but keep an ST-Link + OpenOCD handy and prefer a spare sonde until you have flashed one successfully.

### Automatic unlock (factory-new sondes)

Unlike the desktop steps, you do **not** need a separate unlock pass. When the flasher sees a factory-locked sonde (read-out or write protection set) it clears **all** protection first - which mass-erases the chip, exactly like `stm32l4x unlock 0` / `stm32f1x unlock` - then re-attaches and writes your firmware. An already-unlocked sonde (any RS41-NFW sonde) is flashed straight away.

### Flashing

1. Build your firmware in the Sounding Software.
2. Click **Check ST-Link access** first if you are unsure your setup works, then **Flash `<board>` over WebUSB** and pick your ST-Link in the device prompt.
3. The tool identifies the MCU, clears protection if needed, then erases, writes the `.bin` at `0x08000000`, verifies and resets - with a live log.
4. Disconnect, insert the batteries. You have a flashed sonde now!

If the device picker is empty, the OS access step above has not taken effect yet: re-check the Zadig driver (Windows) or the udev rule (Linux), and reconnect the ST-Link. If a flash fails partway, nothing is bricked - re-flash with this or any other method.


## Uploading directly from the Arduino IDE

If you compiled the firmware locally and just want the IDE to flash it for you:

* Open the Arduino IDE (the 2.x version), with the project set up per the [compilation guide](./COMPILE.md).
* Select the correct board model (see the [operation manual](./OPERATION_MANUAL.md#sonde-pcb-version)).
* Select **(IDE) Tools -> Programmer: -> STMicroelectronics ST-LINK**.
* With the programmer wired up, click **Upload** (Ctrl-U). The IDE compiles and then flashes through STM32CubeProgrammer. When you see a message like *Start application achieved successfully*, the firmware is running.
* A factory-new sonde still has to be [unlocked once](#a2-unlock-the-factory-protection-one-time-per-sonde) before the IDE can program it.
* Disconnect the cables, insert the batteries, and follow the [Operation manual](./OPERATION_MANUAL.md). The sonde stays off until you turn it on with the button.
