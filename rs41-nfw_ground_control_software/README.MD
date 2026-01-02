# RS41-NFW Ground Control Software

<p float="left">
  <img src="./picture1.png" width="33%"/>
  <img src="./picture2.png" width="33%" /> 
  <img src="./picture3.png" width="33%" />
</p>


This guide explains the required Python dependencies, how to create a virtual environment (venv), install the dependencies, and run the software.


## Creating a Python Virtual Environment (venv)

Using a virtual environment is strongly recommended to keep dependencies isolated from your system Python installation.
Some systems don't require activating Python virtual environment, so treat this as an option. If you do not want a virtual environment, skip to [installing the dependencies](#installing-dependencies).

### Step 1: Create the virtual environment

From the project directory (here):

```bash
python -m venv venv
```

This will create a folder named `venv` containing the isolated Python environment.


### Step 2: Activate the virtual environment

**Linux / macOS:**

```bash
source venv/bin/activate
```

**Windows (Command Prompt):**

```bat
venv\Scripts\activate
```

**Windows (PowerShell):**

```powershell
venv\Scripts\Activate.ps1
```

Once activated, your terminal prompt should indicate that the `venv` is active.


## Installing Dependencies

The following Python packages are required:

* `pyserial`
* `flask`
* `flask-socketio`
* `eventlet`

After activating the virtual environment, install the required packages:

```bash
pip install pyserial flask flask-socketio eventlet
```

This ensures all dependencies are installed only inside the virtual environment.


## Running the Software

### Step 1: Activate the virtual environment

Make sure the virtual environment is active:

```bash
source venv/bin/activate
```

(Use the appropriate activation command for your operating system.)


### Step 2: Execute the software

Run the main application file:

```bash
python3 software.py
```

The RS41-NFW Ground Control Software should now start using the virtual environment and installed dependencies.


## Software Guide

The Python script will print out information about the server - we are interested in it's address and port. In terminal, you'll see:

```bash
user@pc$ python3 software.py 
 * Serving Flask app 'software'
 * Debug mode: off
WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.
 * Running on all addresses (0.0.0.0)
 * Running on http://127.0.0.1:4141
 * Running on http://192.168.0.50:4141
Press CTRL+C to quit
```

Application is available at either localhost (127.0.0.1) or your local IP address (in this case 192.168.0.50), on port 4141. You can either open your browser and open [localhost:4141](localhost:4141), or click the address in the terminal.

You'll then see the RS41-NFW Ground Control Software webGUI in your browser. Ensure your RS41-NFW-programmed sonde is connected via serial connection to your computer - if you don't know how to do it, head over to the [serial connection](#serial-connection). If you don't know the serial port it's available at, please check it now.

Type it in the serial connection window, leave the baudrate at 115200 (default setting for RS41-NFW communication protocol) and click connect.

Now you are ready to go, it's time to turn ON the sonde and observe all different parameters here.


## Serial connection

```
Pinout of XDATA connector                                    Pinout of connected ribbon cable
                 -------                                     
           GND  | o   o |  XDATA_RX                          > 1  GND
                |       |                                      2  XDATA_RX(PB11)
      XDATA_TX  | o   o |  +3V_MCU                             3  XDATA_TX(PB10) 
               -        |                                      4  +3V_MCU
       V_Boost|   o   o |  VBAT                                5  V_Boost
               -        |                                      6  VBAT
       MCU_RST  | o   o |  SWCLK(PA14)                         7  MCU_RST
                |       |                                      8  SWCLK(PA14)
   SWDIO(PA13)  | o   o |  GND                                 9  SWDIO(PA13)
                 -------                                      10  GND
```


Look around for `XDATA_RX`, `XDATA_TX` and `GND` pins. I suggest you to use goldpin female jumper wires. Connect them to the previously mentioned pins in the RS41 sonde XDATA port (it is located on the bottom of the PCB). Grab a serial converter of your choice (CP2102, FT232, CH340 in correct mode, Arduino/Espressif/any_other_manufacturer development board with onboard UART converter and MCU held on reset, other thing that will allow us to receive UART data with our computer), and connect them as a usual serial connection (ground connected, RX->TX, TX->RX):

```
XDATA connector             Serial converter
  
  | GND ----------------------- GND |
  | XDATA_RX ------------------ TX  |
  | XDATA_TX ------------------ RX  |

```

After doing so, connect to the computer and find port it's using:

### Linux

List available serial devices:

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

Common serial port names:

* `/dev/ttyUSB0`
* `/dev/ttyACM0`

### macOS

List available serial ports:

```bash
ls /dev/tty.*
ls /dev/cu.*
```

Typical examples:

* `/dev/tty.usbserial-XXXX`
* `/dev/cu.usbmodemXXXX`

The `/dev/cu.*` device is usually recommended for serial communication.

### Windows

1. Open **Device Manager**
2. Expand **Ports (COM & LPT)**
3. Note the assigned port name, for example:

* `COM3`
* `COM5`

Use this COM port in your software configuration.

