# Firmware flashing
To install the firmware on the RS41 radiosonde, you should follow these instructions.

* [Requirements](#requirements)
* [Connecting the programmer to the sonde](#connecting-the-programmer-to-the-sonde)
* [Unlocking the factory MCU protection](#unlocking-the-factory-mcu-protection)
* [*Uploading the pre-compiled binary*](#uploading-the-pre-compiled-binary)
* [Uploading the Arduino Code](#uploading-the-arduino-code)

## Requirements
* **Hardware-side**:
    * Vaisala RS41 sonde (firmware supports [all versions](../hw/README.md#older-vs-newer---how-do-i-know-which-one-im-holding-now))
    * ST-Link compatible programmer - cheap v2 clone was confirmed working. Any expensive original programmer or most clones from china that look like [this](https://sklep.msalamon.pl/wp-content/uploads/2024/07/sklep_msalamon_STLINK_USB_RED.jpg) should work. Also, any other programmer that supports ST's SWD programming protocol and debuging should do the job.
    * Some jumper wires (often included with programmers).
    * Windows, Linux (and probably Mac) computer, software described below.

<br>

**NOTE:** The pre-compiled binaries aren't provided anymore with each release, due to increasing number of customization options and sonde-specific things, such as an automatic sensor boom calibration.
Compiling the firmware is a very straight-forward process, by following the documentation it is rather easy and really takes under 30 minutes.

<br>**Note:** Radiosondes that are factory new, with original firmware, need to be [unlocked](#unlocking-the-factory-mcu-protection).

* **Software-side**:
    * For flasing a **pre-compiled binary**, you will need:
        * A pre-compiled binary (should be in the .bin format) - download a preferred version either from [there](./fw-files/) or from the releases page.
        * [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) or any debuger program (stm32flash on Linux for example) that can utilize the ST-Link to upload the binary to it. This tutorial will cover STM32CubeProgrammer running on Windows and in the future there will be a tutorial for stm32flash on Linux.
    * For **compiling** and flashing **your own code**, you will need:
        * [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) - used by Arduino IDE in the flashing process and to unlock the memory.
        * A properly configured Arduino IDE - more on that further in this text, with tutorial


## Connecting the programmer to the sonde
After you met all needed requirements for your purposes, you have to connect the programmer to the Vaisala sonde.<br>

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
<br>

**Note:** The RS41 XDATA connector has a non-standard raster of 2mm, instead of the standard 2.54mm, usually used in standard jumper cables. However, for only 4 wires it should fit inside well and you shouldn't need to buy separate 2mm connectors.<br>

Using the jumper cables, connect it as follows:
* RS41-**GND** -> STLink-**GND** | RS41-GND is on XDATA pin 1 and 10
* RS41-**SWDCLK** -> STLink-**SWDCLK** | RS41-SWDCLK is on XDATA pin 8; SWDCLK is often named SWCLK or CLK or Clock
* RS41-**SWDIO** -> STLink-**SWDIO** | RS41-SWDIO is on XDATA pin 9; SWDIO is often named SWIO or DIO or Data
* Powering the system:
    * If your programmer is capable of supplying **stable 3.3V DC with at least 200mA** of current (most cheap clones are able to):
        * **Take out both batteries!**
        * Connect: RS41-**+3V_MCU** -> STLink-**+3.3V** | RS41-+3V_MCU is on XDATA pin 4; +3.3V is often named 3.3V or 3v3
    * If not, leave both batteries in the place, **don't** connect any power supply to any of the XDATA power pins, and for the upload, power on the sonde by pressing the onboard button.

<br>
Now your system should be ready to upload the code. <br>
If you encounter any problems at this moment, search in Google for the solution, try to ask AI assistant, and if you don't find the answer, raise an issue (though it should work out of the box).


## Unlocking the factory MCU protection
Factory new sondes with original firmware have read and write protections enabled. Disabling them is only needed once in the sonde's life, to allow the sonde to accept new firmware files. These steps are mandatory for both binary upload and Arduino IDE programming. To disable them, preferably, launch the **STM32CubeProgrammer**:
* In the window, by default you should see something like [this](./photos/main_window.png).
* Now, turn on the sonde, connect it with programmer to the computer and **Select the ST-LINK** as a programmer, then click **connect** [there](./photos/stlink-connection.png).
* Go to **Option bytes** page, and set the following bytes as below:
  * For older models (eg. `RSM412`), set like on [this picture](./photos/mcu-unlock-rsm412.png):
    * In the **Read Out Protection**
      * `RDP byte` to `unchecked` (read out protection disabled)
    * In the **Write Protection**
      * Set **all** boxes to `unchecked`, the write protection has to be disabled on all sectors.
  * For newer models (eg. `RSM414`, `RSM424`, `RSM425`), set like on [this picture](./photos/mcu-unlock-rsm414.png):
    * In the **Read Out Protection**
      * `RDP byte` to `AA`
    * In the **Write Protection**
      * `WRP1A_STRT` - **value** `0x1`, **address** `0x08000800`
      * `WRP1A_END` - **value** `0x0`, **address** `0x08000000`
      * `WRP1B_STRT` - **value** `0x1`, **address** `0x08000800`
      * `WRP1B_END` - **value** `0x0`, **address** `0x08000000`
  * And click **Apply**
* Go to **Erasing & Programming** [page](./photos/erase_and_upload.png), and perform a Full Chip Erase. This will fully clear the MCU memory. If you encoutner any more errors mentioning RDP or any access-protection, please check in the available sources and if you don't know how to resolve it, make an issue there.
* Now your MCU should be **unlocked and ready** to be reprogrammed!

## Uploading the pre-compiled binary
* Start-up the STM32CubeProgrammer. It's not the fastest thing in the world, so wait a second.
* In the window, by default you should see something like [this](./photos/main_window.png).
* Now, turn on the sonde, connect it with programmer to the computer and **Select the ST-LINK** as a programmer, then click **connect** [there](./photos/stlink-connection.png).
* Go to **Erasing & Programming** [page](./photos/erase_and_upload.png), and perform a Full Chip Erase. This will clear the MCU and all protections. If you encoutner any errors mentioning RDP or any access-protection, go to third page and make sure any RDP bytes are OFF.
* If you have connected and erased the MCU properly without errors, select your binary file in the File path area (pay attention to the right version for your sonde), then check the **Verify programming** box and make sure that the **Skip flash erase before programming** is *unchecked*. Optionally, you may want the memory checksum, but it isn't needed and works without that.
* If your settings are similiar to the one on the tutorial and the [photo](./photos/erase_and_upload.png), click **Start Programming** button. Now, after a couple of seconds the sonde should run the firmware from the binary.
* Disconnect all the cables, insert batteries and follow the [**Operation manual**](./OPERATION_MANUAL.md). The sonde should remain OFF until you turn it on with the button.

## Uploading the Arduino Code
* Download and start-up the Arduino IDE, it should be the new, 2.x version.
* Make sure you followed the [**compilation guide**](./COMPILE.md)!!!
* Open the desired version of the .ino project, available in `./fw-files/` folder.
* Select appropriate board model, this operation is described in the operation manual [here](./OPERATION_MANUAL.md#sonde-pcb-version)
* If you double-checked all the things in the Arduino-IDE and on the hardware-side, connect the programmer to the computer and click upload (CTRL-U). The compilation process takes a while, after this time (look at the progress bar in the corner), you should see the upload progress. After seeing something like *Start application achieved successfully...*, the compiled code should be running on your sonde. If you encoutner any errors targeting flash process, triple check your wiring, selected programmer and every step of this guide.
* Now, you can disconnect all the cables and insert batteries. Then follow the [**Operation manual**](./OPERATION_MANUAL.md). The sonde should remain OFF until you turn it on with the button.
