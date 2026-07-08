# Firmware compile

> ## You probably do not need this guide anymore
> The **[NFW Sounding Software](../README.md#rs41-nfw-sounding-software)** ([nfw.flada.ovh](https://nfw.flada.ovh)) compiles the firmware for you in the browser. You select your board, set every option through a guided form, click compile, and flash the sonde **straight from your browser** - **no Arduino IDE, no STM32 core, no board variant files and no libraries to install**.
>
> The manual setup below is only worth following if you specifically want to build locally (for example to modify the source code by hand).

This firmware was written in the Arduino IDE. It is a beginner-friendly IDE with support of tons of different MCUs, made with one of the largest communities that work with embedded coding.<br>

**NOTE:** The [New sondes compiler configuration guide](#new-sondes-compiler-configuration-guide) setup has to be done **only** if you have newer version of sondes, eg. `RSM414`. The older versions have the compiler definitions set in the default core package. The sonde version can be determined [here](../hw/README.md#older-vs-newer---how-do-i-know-which-one-im-holding-now).

All the steps shown here may look hard and confusing, but doing everything step-by-step should do everything for you. This thing will take 5 minutes.<br>

* [Adding an STM32 core](#adding-an-stm32-core-to-the-arduino-ide-board-manager)
* [New sondes compiler configuration guide](#new-sondes-compiler-configuration-guide)
* [Board selection](#board-selection)
* [Additional libraries](#additional-libraries)

## Adding an STM32 core to the Arduino IDE board manager
To make things work, the Arduino IDE has to use the STM32 core to provide functionalities for STM32 microcontrollers:
* Download the new, 2.x version of the Arduino IDE and run it (Arduino IDE had made a big change in its IDE, that's why versions now start with 2.x)
* Click on **(IDE) File -> Preferences**
* In the **Additional Boards Manager URLs**, add the new line with this board manager URL: https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
* Exit the Preferences window. Then open the **Boards Manager** using the toolbar on the right or by entering **(IDE) Tools -> Board -> Boards Manager...**
* Search for the **STM32 MCU based boards** *by STMicroelectronics* (search in the top field for the *stm32* phrase), check that the newest stable version is selected (*3.2?* was tested for now, which is the newest by the day of writing this) and click **Install**.
* The installation will take a while, especially on slower computers and network connections. The progress can be seen in the bottom debug window.
* After the installation is completed, restart the Arduino IDE. Ideally, restart the computer.
* After this is done, make sure that you have installed the **STM32CubeProgrammer**, because Arduino IDE may query it for uploading the binary to the MCU.


## New sondes compiler configuration guide
If you have a **newer** sonde version, you have to follow this guide. <br>

To make sure that everything will work properly, the internal IDE configuration has to be changed. <br>

What we need to do is to add a **new board variant**. Arduino defines it as an *additional folder that is compiled together with the core and allows platform developers to easily add specific configurations.*

After you correctly added the official STM32 core to Arduino IDE board manager, now you have to make a few changes.

* Close the Arduino IDE.
* Download and extract from this repo the folder under this location: `fw/arduino-ide_variant-files/`. It will be needed later.
* For now, delete whole cached data folder named `arduino-ide` (one time work, there is an [issue](https://github.com/arduino/arduino-ide/issues/1030#issuecomment-1152005617) that the new variants don't show up without cache purge), located here (replace USERNAME with your OS username you work on):
    * `C:\Users\USERNAME\AppData\Roaming\arduino-ide\` on Windows 7 and later
    * `~/.config/arduino-ide/` on Linux GUI
    * `~/Library/Application Support/arduino-ide` on macOS
* Locate the sources folder (replace USERNAME with your OS username you work on), this will be the place that you will work in now:
    * `C:\Users\USERNAME\AppData\Local\Arduino15\` on Windows 7 and later
    * `~/.arduino15/` on Linux GUI
    * `~/Library/Arduino15/` on macOS
* Go to the `variants` folder of the STM32 core:
    * `[sources folder]./packages/STMicroelectronics/hardware/stm32/[your STM32 core version]/variants` - default variants folder location
    * `C:\Users\USERNAME\AppData\Local\Arduino15\packages\STMicroelectronics\hardware\stm32\2.8.1\variants` - example for Windows based machine with the newest STM32 core version
* Enter the family name folder, for our case it is a `./STM32L4xx`
    * Here, for our new sonde, you want to locate the `L412RB(I-T)xP` folder, delete it and replace it with [this](./arduino-ide_variant-files/variants/STM32L4xx/L412RB(I-T)xP/) folder, from this repo (`fw/arduino-ide_variant-files/variants/STM32L4xx/L412RB(I-T)xP/`).
* If you swapped the new variant folder in place, make sure that it contains 8 files and that the folder swapped correctly, then go back to the version folder (`[sources folder]./packages/STMicroelectronics/hardware/stm32/[your STM32 core version]/`), from there open `boards.txt` file in a text editor, search for the `Generic L4` line, and add an entry from [`boards.txt` file from this repo](./arduino-ide_variant-files/boards.txt) between other variants (you could add the entry in the alphabetical order). Save the file and close.
* **Your IDE should now be good to go**

## Board selection
This depends on the PCB version you have. The simplest way to find it is to read the model printed on the bottom of the PCB - it is marked `RSM4x` followed by the revision digit (e.g. `RSM425`, `RSM414`, `RSM412`). A trailing `4` or `5` means a newer board (STM32L412); a `1` or `2` means an older board (STM32F100). <br>

* Open Arduino IDE
  * For older versions (eg. `RSM412`):
    * Select the **Generic STM32F1 series** under **(IDE) Tools**, then go to **(IDE) Tools -> Board part number:** and select **Generic F100C8Tx**.
  * For newer versions (eg. `RSM414`, `RSM424`):
    * Select the **Generic STM32L4 series** under **(IDE) Tools**, then go to **(IDE) Tools -> Board part number:** and select the newely installed **Generic L412RBTxP**. Don't see it? Make sure you followed the [above](#new-sondes-compiler-configuration-guide) guide.
* Select the appropriate programmer -  **(IDE) Tools -> Programmer: -> STMicroelectronics ST-LINK**
* Select compiler optimization under **(IDE) Tools -> Optimize:**
  * Newer boards (`RSM4x4` / `RSM4x5`, L412) - **Smallest (-Os default)**. There is plenty of flash, no LTO needed.
  * Older boards (`RSM4x2` / `RSM4x1`, F100) - **Smallest (-Os) with LTO**. The full feature set only fits in the 64 KB flash with Link-Time Optimization enabled. (The Sounding Software already builds the F100 with LTO automatically.) See the note below on why LTO is needed and how to build without it.



## Additional libraries
RS41-NFW does not require additional installation of libraries via Library Manager anymore! All files are included already.


---

For pre-v58 releases:
Besides the compiler configuration, you also have to add necessary libraries to your environment. <br>
Currently needed libraries for this project (only one):
* TinyGPS++
    * Visit https://github.com/mikalhart/TinyGPSPlus and **download** code as a .zip file (green code button).
    * In Arduino IDE, go to **(IDE) Sketch -> Include Library... -> Add .ZIP Library** and select the already downloaded .zip file.
    * After a while, it should be installed, the progress can be seen on the bottom of the IDE.

<br>

**Your compilation environment is now fully set-up!**
