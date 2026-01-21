/*
RS41-NFW - versatile, feature-rich and user-friendly custom firmware for ALL revisions of Vaisala RS41 radiosondes
Released on GPL-3.0 license.
Authors: Franek Łada (nevvman, SP5FRA)

Version 65 (public, stable)

All code and dependencies used or modified here that don't origin from me are described in code comments and repo details.
https://github.com/Nevvman18/rs41-nfw
*/



/*
Thanks for choosing RS41-NFW.
If You have any questions, don't hesitate to leave an issue in GitHub repo, I will respond to every message and try to help.

Please carefully read all the comments in this configuration, as they are currently the most up-to-date documentation of all the firmware features.

I hope You find this project exciting, helpful and straightforward to use.
I wish You high, successful flights with a lot of data gathered with this firmware.

Franek,
Author of RS41-NFW
*/
#define NFW_VERSION "RS41-NFW v65, GPL-3.0 Franek Lada (nevvman, SP5FRA)"  //This is the firmware version You are running




//===== Libraries and lib-dependant definitions (nothing to modify)
/* No libraries are required to be installed, all dependencies are shipped within the project folder. */
#include "horus_l2.h"
//#include "horus_l2.cpp"
#include <SPI.h>
#include "gps.h"
#include "HorusBinaryV3.h"

TinyGPSPlus gps;

#include <HardwareTimer.h>




/* ===== BEGINING OF THE RS41-NFW CONFIGURATION SECTION ===== */



//===== Device revision definitions
/* CHANGE-ME! SELECT YOUR BOARD REVISION BY UNCOMMENTING THE RIGHT DEFINITION BELOW (model can be found written on the PCB at the bottom part, see compilation manual for more info)*/
 
// #define RSM4x4 // New PCB versions, RSM4x4 AND RSM4x5 (based on MCU STM32L412RBT6 LQFP64)
// #define RSM4x2  // Old PCB versions, RSM4x2 and RSM4x1 (based on MCU STM32F100C8T6B LQFP48)









//==== Firmware-internal definitions:
#ifdef RSM4x4
bool rsm4x2 = false;
bool rsm4x4 = true;
//===== Pin definitions
#define RED_LED_PIN PC8    //red led - reversed operation, pin HIGH=led_off, pin LOW=led_on!
#define GREEN_LED_PIN PC7  //green led - reversed operation, pin HIGH=led_off, pin LOW=led_on!

#define PSU_SHUTDOWN_PIN PA9  //battery mosfet disable pin
#define VBAT_PIN PA5          //battery voltage divider
#define VBTN_PIN PA6          //button state voltage divider, CHECK VALUES AT LOWER SUPPLY VOLTAGES

#define MOSI_RADIO_SPI PB15
#define MISO_RADIO_SPI PB14
#define SCK_RADIO_SPI PB13
#define CS_RADIO_SPI PC13
#define HEAT_REF PC6  //reference heating resistors - LOW=disabled, HIGH=enabled - draw about 180mA of current and after a while may burn skin
bool heaterPinControlAvail = true;
#define REF_THERM PB1   //reference heating thermistor
#define PULLUP_TM PB12  //ring oscillator activation mosfet for temperature reading
#define PULLUP_HYG PA2  //ring oscillator activation mosfet for wxHumidity reading
#define SPST1 PB3       //idk spst1
#define SPST2 PA3       //idk spst2
#define SPST3 PC14      //boom hygro heater temperature
#define SPST4 PC15      //boom main temperature
#define SPDT1 PC10
#define SPDT2 PC11
#define SPDT3 PC12
#define MEAS_OUT PA1  //ring oscillator measurement output
#define HEAT_HUM1 PA7
#define HEAT_HUM2 PB8
#define GPS_RESET_PIN PB9
#define CS_SPI PB2

#define SI4032_CLOCK 26.0

#define aprsSpaceTime 201
#define aprsMarkTime 400

//===== Interfaces
//SPI_2 interface class (radio communication etc.)
SPIClass SPI_2(PB15, PB14, PB13);  // MOSI, MISO, SCK for SPI2
// ublox gps              rx    tx
HardwareSerial gpsSerial(PB7, PB6);
int gpsBaudRate = 38400;

#elif defined(RSM4x2)
bool rsm4x2 = true;
bool rsm4x4 = false;
//===== Pin definitions
#define RED_LED_PIN PB8    //red led - reversed operation, pin HIGH=led_off, pin LOW=led_on!
#define GREEN_LED_PIN PB7  //green led - reversed operation, pin HIGH=led_off, pin LOW=led_on!

#define PSU_SHUTDOWN_PIN PA12  //battery mosfet disable pin
#define VBAT_PIN PA5           //battery voltage
#define VBTN_PIN PA6           //button state voltage divider, CHECK VALUES AT LOWER SUPPLY VOLTAGES

#define MOSI_RADIO_SPI PB15
#define MISO_RADIO_SPI PB14
#define SCK_RADIO_SPI PB13
#define CS_RADIO_SPI PC13
#define HEAT_REF 0      //not available on older versions, switched only by si4032 gpio
bool heaterPinControlAvail = false;
#define REF_THERM PB1   //reference heating thermistor
#define PULLUP_TM PB12  //ring oscillator activation mosfet for temperature reading
#define PULLUP_HYG PA2  //ring oscillator activation mosfet for wxHumidity reading
#define SPST1 PB6       //calibration resistors - 90kHz
#define SPST2 PA3       //cal. res. - 62kHz
#define SPST3 PC14      //boom hygro heater temperature
#define SPST4 PC15      //boom main temperature
#define SPDT1 PB3
#define SPDT2 PB4
#define SPDT3 PB5
#define MEAS_OUT PA1  //ring oscillator measurement output
#define HEAT_HUM1 PA7
#define HEAT_HUM2 PB9
#define GPS_RESET_PIN PA15
#define CS_SPI PB2

#define SI4032_CLOCK 26.0

#define aprsSpaceTime 190
#define aprsMarkTime 394

//===== Interfaces
//SPI_2 interface class (radio communication etc.)
SPIClass SPI_2(PB15, PB14, PB13);  // MOSI, MISO, SCK for SPI2
// ublox gps              rx    tx
HardwareSerial gpsSerial(PA10, PA9);
int gpsBaudRate = 9600;

#else
#error "Please define the PCB model!"
#endif

// XDATA (2,3)          rx    tx
HardwareSerial xdataSerial(PB11, PB10);





//===== Radio signals config

bool radioEnablePA = true;  // Default radio state



/*
TX timing configuration

GPS-clock synchronized transmission intervals.
Select desired interval using '...TimeSyncSeconds' variable (example: 15 means every transmission 15 seconds from round hour, so at 10:40:00, 10:40:15, 10:40:30 etc.),
note that setting the interval to zero disables the transmission, and setting transmissions too often could result in no time for other tasks.
Suggested stable intervals are >7s for each transmission, but feel free to test everything.
Offset however (example: offset 2 means 2 seconds after set interval time, so 10:40:02, 10:40:17, 10:40:32, etc.) is useful when flying multiple sondes.
*/

// Pip:
constexpr uint16_t pipTimeSyncSeconds = 15;
constexpr uint16_t pipTimeSyncOffsetSeconds = 0;


// Horus V3:
constexpr uint16_t horusV3TimeSyncSeconds = 15;
constexpr uint16_t horusV3TimeSyncOffsetSeconds = 0;


// Horus V2:
constexpr uint16_t horusTimeSyncSeconds = 15;
constexpr uint16_t horusTimeSyncOffsetSeconds = 0;


// APRS:
constexpr uint16_t aprsTimeSyncSeconds = 30;
constexpr uint16_t aprsTimeSyncOffsetSeconds = 0;


// RTTY:
constexpr uint16_t rttyTimeSyncSeconds = 15;
constexpr uint16_t rttyTimeSyncOffsetSeconds = 0;


// Morse:
constexpr uint16_t morseTimeSyncSeconds = 15;
constexpr uint16_t morseTimeSyncOffsetSeconds = 0;




// Modes configuration:

// Pip:
bool pipEnable = false;               // Enable pip tx mode (carrier)
constexpr float pipFrequencyMhz = 432.7;        // Pip tx frequency
constexpr uint16_t pipLengthMs = 100;               // Pip signal length in ms
constexpr uint16_t pipRepeat = 3;                    // Pip signal repeat count in 1 transmit window
constexpr int8_t pipRadioPower = 7;                // TX power, 0 = -1dBm (~0.8mW), 1 = 2dBm (~1.6mW), 2 = 5dBm (~3 mW), 3 = 8dBm (~6 mW), 4 = 11dBm (~12 mW), 5 = 14dBm (25 mW), 6 = 17dBm (50 mW), 7 = 20dBm (100 mW)


/* Horus V3 transmission mode - encoder provided by Mark VK5QI
Lately released 3rd generation of Horus 4FSK modem and protocol - read more about experimental releases at: https://github.com/xssfox/horusbinaryv3, and soon at: https://github.com/projecthorus/horusdemodlib/wiki/6-TBA-Horus-Binary-V3 and all other official sources.
The biggest change is ASN.1 encoding. Thanks to it, you no longer need to request a payload ID nor a custom packet format anymore. From now, you can specify the callsign below in the definition, and it will be sent in the packet itself.
Also, the new additional sensor data can be easily sent now.

horusV3LongerPacket - Enabling longer packet adds:
temperatures:
  custom1 - humidity module temperature sensor

extraSensors:
  gps_0_0 - gpsStatus - Current GPS power mode. If RSM4x2: 0=disabled, 1=max performance, 2=powersave. If RSM4x4: 0=disabled, 1=max performance, 2=max tracking, powersave, 3=efficient tracking, powersave
  gps_0_1 - gpsHdop   - Horizontal GPS precision value
  
, leaving the horusV3LongerPacket at false will transmit using V3 the same telemetry contents as with V2. I suggest leaving the longerPacket true for most cases.

For more information of encoding, refer to documentation and horus v3 function in code.*/
bool horusV3Enable = true;              // Enable horus v3 tx mode
constexpr float horusV3FreqTable[] = {437.6};     // Specify all horus frequencies You want (example {437.6, 434.714, 433.8};), the sonde will cycle through all of them one-by-one during a transmit cycle. Useful when flying long flights in different places of the world (Poland 437.6Mhz, most of the europe 434.714Mhz) Note - lowAltitudeFasTxMode will only use primary frequency - the first one specified.
#define HORUS_V3_CALLSIGN "4FSKTEST-V3"      // Payload callsign for Horus V3 mode. Note that adding every single character increases your packet size by 6 bits.
constexpr uint16_t horusV3Bdr = 100;                   // Transmission baudrate, default 100
constexpr int8_t horusV3RadioPower = 7;              // TX power, 0 = -1dBm (~0.8mW), 1 = 2dBm (~1.6mW), 2 = 5dBm (~3 mW), 3 = 8dBm (~6 mW), 4 = 11dBm (~12 mW), 5 = 14dBm (25 mW), 6 = 17dBm (50 mW), 7 = 20dBm (100 mW)
bool horusV3LongerPacket = true;        // See explanation above.
constexpr uint16_t horusBdr = 100;                   // Transmission baudrate, default 100
constexpr int8_t horusPreambleLength = 8;   // Horus V2/V3 preamble length in bits - default at 16. Allows receivers to lock on the carriers etc.


// Horus v2 - NOTE: I HIGHLY! SUGGEST SWITCHING TO V3 MODE, v2 is becoming outdated and v3 is here as a new, much more flexible standard. RX stations should also keep their software updated to the latest releases, which support Horus V3.
bool horusEnable = false;              // Enable horus v2 tx mode
constexpr float horusFreqTable[] = {437.6};     // Specify all horus frequencies You want (example {437.6, 434.714, 433.8};), the sonde will cycle through all of them one-by-one during a transmit cycle. Useful when flying long flights in different places of the world (Poland 437.6Mhz, most of the europe 434.714Mhz) Note - lowAltitudeFasTxMode will only use primary frequency - the first one specified.
constexpr uint16_t horusPayloadId = 256;    // Horus v2 (v1 is outdated and NFW only sends extedned v2 format, v1 IDs seem to work with v2 too) payload ID. Please obtain Yours by creating an issue at https://github.com/projecthorus/horusdemodlib/issues
constexpr int8_t horusRadioPower = 7;              // TX power, 0 = -1dBm (~0.8mW), 1 = 2dBm (~1.6mW), 2 = 5dBm (~3 mW), 3 = 8dBm (~6 mW), 4 = 11dBm (~12 mW), 5 = 14dBm (25 mW), 6 = 17dBm (50 mW), 7 = 20dBm (100 mW)


// APRS:
bool aprsEnable = true;               // Enable APRS tx mode
constexpr float aprsFreqTable[] = {432.5};      // APRS frequency table (same format as Horus table). Same as for horus frequency table. Note - dataRecorder will only use primary frequency (first specified). Note - lowAltitudeFasTxMode will only use primary frequency - the first one specified.
char aprsCall[] = "N0CALL";           // Callsign
String aprsComment = " NFWv65";          // APRS message comment
constexpr char aprsSsid = 11;                   // SSID for the call sign
constexpr char aprsDest[] = "APRNFW";           // Destination address for APRS
constexpr char aprsDigi[] = "WIDE2";            // Digipeater callsign
constexpr char aprsDigiSsid = 1;                // Digipeater SSID
constexpr char aprsSymbolOverlay = 'O';         // Symbol overlay - 'O' for balloon icon, '_' for WX station icon
constexpr char aprsSymTable = 'a';              // Symbol table (e.g., 'a' for standard symbol)
/* APRS Operation Mode:
  1 - Standard RS41-NFW tracker telemetry format, where in APRS comment:
    F - frame
    S - sats
    V - batt (mV -> V)
    C - ascent_rate (cm/s -> m/s)
    I - temp
    T - ext_temperature
    H - ext_humidity
    P - ext_pressure (daPa (dekaPascal) -> hPa)
    J - jam_warning (1 or 0)
    R - PCB revision (determines model string)
  
  2 - weather station format, sends APRS WX weather reports */
constexpr int8_t aprsOperationMode = 1;
constexpr int8_t aprsRadioPower = 7;               // TX power, 0 = -1dBm (~0.8mW), 1 = 2dBm (~1.6mW), 2 = 5dBm (~3 mW), 3 = 8dBm (~6 mW), 4 = 11dBm (~12 mW), 5 = 14dBm (25 mW), 6 = 17dBm (50 mW), 7 = 20dBm (100 mW)


// RTTY (Franek's original code was modified by OM3BC (thanks!)):
#define CALLSIGN "N0CALL"             // Callsign used for morse and rtty

bool rttyEnable = false;              // Enable rtty tx mode, compliant with UKHAS format
constexpr float rttyFrequencyMhz = 434.6;       // RTTY tx frequency
constexpr uint16_t rttyBitDelay = 10000;             // RTTY delay between transmitted bits - 22000 ~= 45bdrate, 13333 ~= 75bdr, 10000 ~= 100bdr
constexpr uint8_t rttyBits = 7;                     // RTTY 7 = 7bit character, 8 = 8bit character
constexpr uint8_t rttyStopBits = 2;               // RTTY stop bits (1, 1.5, 2)
#define RTTY_RADIO_MARK_OFFSET 0x03   // TX-related, specifies radio offset settings
#define RTTY_RADIO_SPACE_OFFSET 0x01  // TX-related, specifies radio offset settings
constexpr int8_t rttyRadioPower = 7;               // TX power, 0 = -1dBm (~0.8mW), 1 = 2dBm (~1.6mW), 2 = 5dBm (~3 mW), 3 = 8dBm (~6 mW), 4 = 11dBm (~12 mW), 5 = 14dBm (25 mW), 6 = 17dBm (50 mW), 7 = 20dBm (100 mW)

// Morse:
bool morseEnable = false;             // Enable morse tx mode
constexpr float morseFrequencyMhz = 434.6;      // Morse tx frequency
constexpr uint16_t morseUnitTime = 40;               // Morse unit time
constexpr int8_t morseRadioPower = 7;              // TX power, 0 = -1dBm (~0.8mW), 1 = 2dBm (~1.6mW), 2 = 5dBm (~3 mW), 3 = 8dBm (~6 mW), 4 = 11dBm (~12 mW), 5 = 14dBm (25 mW), 6 = 17dBm (50 mW), 7 = 20dBm (100 mW)



/* Fox hunting mode
This mode is separate from standard HAB operation mode. It can only transmit morse and FM melody sounds on a specified frequency.The firmware minimises the current consumption.
In this mode, only the simplified button operation mode is used (1 only power OFF) and the status lights are simplified - green LED blinking when OK, orange LED when something is wrong (for example low battery)
*/
bool foxHuntMode = false;                                          // Enables the fox hunting mode.
bool foxHuntFmMelody = true;                                       // In fox hunting mode, enables a melody transmitted with the onboard radio
bool foxHuntCwTone = false;                                        // If You prefer a CW tone instead of a FM melody, the tone length is 10s
bool foxHuntMorseMarker = true;                                    // In fox hunting mode, enables a morse marker transmitted after the melody with the text specified below. Warning - the marker is transmitted in CW, not in FM! Use a side-band capable receiver to receive it.
String foxMorseMsg = "N0CALL N0CALL FOX";                          // Morse message contents
bool foxHuntLowVoltageAdditionalMarker = true;                     // When the low voltage threshold is met (specified with vBatWarnValue), the additional marker is activated, to send for example transmitter location
String foxMorseMsgVbat = "N0CALL N0CALL FOX 11.123456 12.456789";  // Additional morse message when vBatWarnValue is met
constexpr float foxHuntFrequency = 434.5;                                    // Foxhunt tx freq
constexpr uint16_t foxHuntTransmissionDelay = 0;                         // Delay in fox hunting mode between transmission cycles in ms
constexpr int8_t foxHuntRadioPower = 7;                                         // TX power, 0 = -1dBm (~0.8mW), 1 = 2dBm (~1.6mW), 2 = 5dBm (~3 mW), 3 = 8dBm (~6 mW), 4 = 11dBm (~12 mW), 5 = 14dBm (25 mW), 6 = 17dBm (50 mW), 7 = 20dBm (100 mW)




// ===== Hardware operation config

/* ===== LED Status
You can enable them and set an altitude at which they automatically turn OFF.

LED status description during operation:
- Constant Red - important error, such as sensor boom error, initial sensor calibration error, RPM411 connection error (if configured)
- Constant Orange - warning, like no GPS fix or battery voltage below 'vBatWarnValue'
- Blinking Orange - improved GPS performance mode enabled and sonde is still searching for satelliltes
- Blinking Green - improved GPS performance mode enabled and sonde successfully found many satellites and soon will return to ready-to-flight mode
- Constant Green - all systems working correctly and no errors, sonde is ready to be launched

LED status during startup:
- Red blinks 3 times - wrong conditions for zero humidity calibration, exiting zero humidity calibration
- Red blinks 5 times - zero humidity calibration cancelled due to a sensor boom error
- Constant red - hardware init
- Constant orange - sensor boom and heating circuitry init, also partly GPS initialization
- Blinking orange shortly - calibration in progress, either reconditioning, temperature correction or zero-humidity-check
- Green blinks 5 times - firmware setup done, entering main program
*/
bool ledStatusEnable = true;
constexpr int16_t ledAutoDisableHeight = 1000;                            // Altitude in meters above which the status LEDs get disabled



/* ===== XDATA port and instruments
Choose Your XDATA port operation mode.
For most users, Mode 4 is suggested, which enables NFW serial protocol compatible with RS41-NFW Ground Control Software, an advanced and user-friendly webGUI interface.
Optionally You may select Mode 1, which allows user to connect to a serial terminal and view text log of firmware operation.
Modes 0,2, shouldn't be used as they do nothing, mode 3 is still in development.
*/
constexpr uint8_t xdataPortMode = 4;   //0 - disabled, 1 - debug uart, 2 - i2c (NO implementation now, does nothing), 3 - xdata sensors (oif411, bad implementation, needs to be rewritten for user-friendliness), 4 - RS41-NFW Control Software communication mode. 5 - GPS bridge mode @default baudrates (115200)
constexpr uint16_t oif411MsgWaitTime = 1100;  //waiting time for oif411 message



// ===== Power management
float vBatWarnValue = 0;                                    // Battery warning voltage
float batteryCutOffVoltage = 0;                             // Good for nimh rechargable cell life, below 0.8V per AA cell the damage could occur; if You plan on maximising the working time, set to 0V to disable the auto turn OFF, or if using lithium batteries, which are safer to use
bool ultraPowerSaveAfterLanding = false;                    // 20 minutes after landing the sonde will turn OFF the GPS completely, turn OFF all sensors and change the transmit interval to 5 minutes and switch to Horus and APRS, transmitting the last coordinates. Useful for preserving as much battery as possible



// ===== GPS configuration
/* gpsOperationMode:
  0 - fully OFF (stationary use, like WX station, the stationary coordinates can be specified in gpsLat-gpsLong)
  1 - default, always ON;
  2 - standard powersaving when stable position (only RSM4x2, old feature which lowers power consumption when GPS signal is strong, automatically goes back to Max Performance when necessary)
  3 - INTELLIGENT GPS management - algorithm available only for RSM4x4 boards, which together with the M10 u-blox GPS' Super-S, PSMCT, ITFM, constellations and GNS messages allows
      for a BIG improvement in terms of power consumption and interference and jamming resiliance.

Suggested values for tracker use - choose 3 for RSM4x4 or 2 for RSM4x2.
Both 2 and 3 power modes don't affect tracking performance, as the NFW constantly maintains best performance/power.
IF SET TO 3 READ BELOW OPTION */
uint8_t gpsOperationMode = 3;

/* gpsManagement options - available only for RSM4x4 - and only when gpsOperationMode = 3 is selected:
  m10ConstellationOptimization - NFW automatically adjusts used constellations to provide both the best performance and lowest power consumption. Detects signal quality and navigation properties. 
  m10AggressiveOpt - prioritizes power efficiency over tracking quality in constellation optimization.
  m10CyclicTracking - in good signal scenarios the receiver will enter a cyclic tracking mode, where navigation processor wakes up every 10 seconds, calculates the position, and goes to sleep. This option provides a big power improvement.
  m10PerformanceImprovements - provides many improvements in terms of signal reception, satellite orbit parameters and refresh rates.
  m10SuperS - u-blox's Super-S technology maximizes performance in different conditions, and optimizes power consumption at the same time.
  
Suggested values:
  m10constellationOptimization = true
  m10AggressiveOpt = false, set to true if you really really care about those last milliamps. Note: disabling WILL significantly impact tracking performance. This is the only option that can badly impact tracking.
  m10CyclicTracking = true;
  m10PerformanceImprovements = true, always;
  m10SuperS = true, only profit
*/
bool m10ConstellationOptimization = true;
bool m10AggressiveOpt = false;
bool m10CyclicTracking = true;
bool m10PerformanceImprovements = true;
bool m10SuperS = true;


/* ubloxGpsAirborneMode - Sets the uBlox GPS module to the Airborne 1G Dynamic Model, which allows flights above 18km altitude
Suggested setting - true (only consider false if You don't use the sonde at altitudes over 18km and want a slightly better position precision) */
bool ubloxGpsAirborneMode = true;


/* gpsTimeoutWatchdog - In milliseconds, the time after which the GPS chip resets if the position is not valid (no fix), kind of a watchdog.
Also helps to regain the fix quicker, default 30 minutes (1800000 ms), set to 0 to disable.
Suggested setting - 900000 ms (reset every 15 minutes of no valid fix) */
unsigned long gpsTimeoutWatchdog = 900000;


/* improvedGpsPerformance - If true, the device improves the gps fix achieving performance.
The issue is that the radio chip (Si4032) makes wideband noises (so-called spurious emmissions), which affects the GPS L-band too, causing the receiver to have an overall lower sensitivity.
This option automatically changes the TX interval to 120s if the GPS didn't catch a fix; after GPS sees enough satelites, the TX interval goes back to default set.
Mode status is available via LED status and RS41-NFW Ground software.
Suggested setting - true */
bool improvedGpsPerformance = true;


/* This setting disables the improvedGpsPerformance features when the sonde is in-flight, because it can cause a loss of data for up to 2 minutes.
Suggested: If You fly under interference conditions, very worth to set this to false. Else - usually set to true. */
bool disableGpsImprovementInFlight = true;

unsigned long radioSilenceDuration = 120000;  // How long the radio is silent (by default tx every 2 minutes) when improvedGpsPerformance feature is active


/* GPS default coordinates
Worth setting during stationary use when gpsOperation mode is set to 0.
If GPS is active, they will get overwritten. */
float gpsLat = 0;                                           // Latitude
float gpsLong = 0;                                          // Longitude
float gpsAlt = 0;                                           // Altitude


constexpr uint8_t gpsSatsWarnValue = 4;                     // Warning value for GPS satellites number
unsigned long gpsPowerSaveDebounce = 300000;                // Debounce to limit setting the GPS back and forth into the power saving mode



// ===== Sensors
bool sensorBoomEnable = true;                              // Enables sensor boom measurements and diagnostics
bool sensorBoomPowerSaving = false;                         // Lowers the sensors refresh rate to 30s

//NOTE - calibration process is very clear and fully guided if You use RS41-NFW Ground Control Software together with selected values here. Keep reading :D

/* Temperature measurement subsystem */
/* mainTemperatureCorrectionC - correction offset for main temperature sensor (silver hook).
Vaisala sensor booms are nearly similairly linear between each other and the only difference between them is this temperature offset here.
Note that all calibration parameters here are only for a particular sensor boom. Switching to a different one will more than likely require recalibration.
Usage (suggested option 1 and then option 2):
  1. 
  Enable 'autoTemperatureCalibration', which will automatically calibrate Your sensor boom during initialization and calculate this value.
  Beware that during each sonde startup it will need to auto-calibrate. If You want to have a pre-programmed value, look into 2 below (3 is more complicated but works too).

  2.
  You can then read this value through serial port of RS41-NFW Ground Software (xdataPortMode 1 or 4).
  After reading it You can disable 'autoTemperatureCalibration' and manually set the calculated 'mainTemperatureCorrectionC' in the firmware, so in the future the calibration of a particular sensor boom won't be needed.

  3. If options 1 and 2 aren't for You:
  Turn ON the sonde with 'mainTemperatureCorrectionC' set to 0, read the reported external temperature value (either in radio transmission or in RS41-NFW Ground Control Software) and compare it with a known thermometer.
  Calculate (actualTemperature - sondeRawReading = mainTemperatureCorrectionC) and set this offset down below.
*/
float mainTemperatureCorrectionC = 0;


/* extHeaterTemperatureCorrectionC and autoHumidityModuleTemperatureCorrection - offset and auto-calibration of external humidity module heater temperature sensor.
Usage (suggested option 1):
  1.
  Enable 'autoHumidityModuleTemperatureCorrection' and forget about this variable. Seriously.
  The firmware will compare all temperature values and correct this sensor automatically.
  Note that for a real and accurate correction Your main temperature sensor needs to be calibrated.

  2. If option 1 is not for You:
  Optionally set this to 0, launch RS41-NFW Ground Control Software and read the reported temperature value. Manually calculate the offset and enter it below.
*/
float extHeaterTemperatureCorrectionC = 35;
bool autoHumidityModuleTemperatureCorrection = true;


/* autoTemperatureCalibration - enables automatic calibration of main temperature sensor
Calibration uses a method selected below with 'autoTemperatureCalibrationMethod'
For suggested settings read about 'mainTemperatureCorrectionC' and 'extHeaterTemperatureCorrectionC'.
*/
bool autoTemperatureCalibration = true;


/* autoTemperatureCalibrationMethod - method of automatic calibration of the main temperature sensor
  1. autoTemperatureCalibrationMethod = 1
  Calibration based on the known constant air temperature, which is specified with 'environmentStartupAirTemperature'.
  For example Your room has 24*C and You turn ON the sonde in it, the sonde will correct automatically itself. You MUST then turn ON the sonde in this environment.
  This option gives most accurate results of calibration when used properly (accurately specified environment temperature).
  
  2. autoTemperatureCalibrationMethod = 2
  Based on the average PCB temperature.
  The PCB temperature should be adapted to the environment temperature (which means raw PCB with no cover should be left in the calibration environment for about 10 minutes).
  The function corrects the calibration slightly by an empirical polynomial, which uses PCB temperature as input data.
  Much worse accuracy, works best in environment temperatures ranging from 5C to 32C.

Automatic temperature calibration is indicated by orange LED during startup, for more status information, read about LED status above or launch RS41-NFW Ground Control Software. */
constexpr uint8_t autoTemperatureCalibrationMethod = 1;
float environmentStartupAirTemperature = 24;




/* Humidity measurement subsystem */
bool humidityModuleEnable = true;  // Setting that enables the support of humidity module


/* reconditioningEnabled - humidity sensor reconditioning
Phase before zero-humidity check, lasting for a minute. Heats the sensor to the specified value and removes impurities and debris from the humidity module.
Suggested with the zero-humidity check. (Also used during the original Vaisala ground check, lasts a couple of minutes and heats the sensor to 180*C) */
bool reconditioningEnabled = false;
constexpr uint8_t reconditioningTemperature = 145;


/* Zero humidity check (how Vaisala calls it) or more accurately Zero-Humidity Calibration 
This option automatically checks and calibrates the humidity sensor, by heating it to a temperature where water evaporates - above 100C in normal conditions.
Note that all this is completely safe, as factory Vaisala firmware does the same during launch, and NFW uses carefully written and adjusted PID control for heater to achieve stable temperature.

The process is completely automatic and straightforward. After turning ON the sonde, eventually an orange light will blink, indicating heating of the sensor.
You can monitor the calibration progress by launching RS41-NFW Ground Control Software (or by using serial terminal if You don't like the easier method).
The calibration should last under a minute, usually less than 30 seconds.

The sonde during calibration should stay in a warm, not wet environment, with little to no wind (preferably indoors, with temperature raning from 5 to 40C and RH from 0 to 60%).

As with temperature calibration, there are 2 paths to choose (no suggestion, choose what is suitable for You):
  1. Automatic calibration on each startup.
  This won't be however possible when sonde will be turned ON outdoors during winter, because wind and cold will not allow the heater to achieve a working temperature, here comes the option 2:

  2. Zero humidity calibration and manual config
  Before turing ON the sonde, launch RS41-NFW Ground Control Software (or serial terminal and xdataPortMode=1).
  Turn ON the sonde, wait for the calibration to end and look in the Sensors tab for a "zero humidity capacitance". Copy it and paste here in "zeroHumidityCapacitance".
  Now Your sonde will remember the calibration data for this sensor boom. */
bool zeroHumidityCalibration = true;
float zeroHumidityCapacitance = 0;


/* humidityRangeDelta
Default value should be okay, as it is an average from many different sensor booms.
However some are different, and if You notice that Your humidity readings are of%, this is the cause, as some humicaps react differently to humidity.

As always, two methods to choose from (for beginners suggested option 1, if You have 10 minutes time more today check out option 2 :D):
  1.
  Leave this value as is and keep humidityCalibrationDebug disabled.

  2. 'humidityRangeDelta' calibration with 'humidityCalibrationDebug'
  Before turning ON the sonde prepare 100%RH environment. Well working home options:
    - Boiling water (kettle, pot) or very hot water (big cup, bowl, keep in mind it should release water fog).
    When later placing the sensor boom, hold it sensor-side upwards in the boiling fog about 5-8cm above the water.
    - Your mouth, but it isn't as accurate. Do a warm, humid whoooh on the sensor for a period of time. If using this method, consider adding +0.3 to the value.
  It doesn't need to be a lab environment, just wet environment close to 100%RH.

  If You want the most accurate humidity readings, enable 'humidityCalibrationDebug' below and connect Your sonde through a serial port:
    - Using RS41-NFW Ground Control Software: after turning ON the sonde and waiting for zero humidity calibration to pass, the sonde will enter 'humidityCalibrationDebug' mode.
      From there, switch to the Sensors tab, look around for the humidity range delta number (it should be changing a bit) and place the sensor in a 100%RH environment.
      Observe the value and the sensor. Write down the highest number You have seen, BUT keep the sensor without condensation (in boiling fog with the sensor upwards and look at it to be clear of water).
  The observed max delta value change below in 'humidityCapacitanceRangeDelta' and disable 'humidityCalibrationDebug'. */
float humidityCapacitanceRangeDelta = 6;         // Empirical tests average
bool humidityCalibrationDebug = false;                    // (Bla bla bla after calibration the sonde enters special mode that prints out on serial port or RS41-NFW Ground Control Software the frequencies and a suggested humidityRangeDelta value. After it enters this mode, place the sensor in a 100%RH environment (for example close over a boiling water) and read the rangeDelta. This will give You a higher accuracy of the readings for each sensor boom.)
constexpr unsigned long humidityCalibrationTimeout = 300000;        // Zero humidity calibration timeouts if it can't finish in (by default) 5 minutes (300000 milliseconds)
constexpr uint8_t humidityCalibrationMeasurementTemperature = 115;      // Minimum sensor temperature, at which the calibration function takes measurements. Must be over 110 for reliable calibration, suggested 115



/* pressureMode - determines pressure measurement method:
  0 - pressure measurement disabled, reported value is 0.#
  
  1 - Vaisala RPM411 pressure sensor measurement method. Simply plug the sensor on the back side of the board (note - it must be an RS41-SGP model) and configure this option. You'll now heave extremely precise pressure, measured with an official, factory-calibrated BARO-CAP sensor.
      You do not need to calibrate anything, nor specify any values (don' change the sea level pressure as it will do nothing).
  
  2 - pressure estimation - works like in RS41-SG models, with a pressure model, which expects user's input of `seaLevelPressure`. Estimates the pressure of dry air based on altitude, temperature and humidity. Not so precise, more of an 'order of magnitude'.
      Change 'seaLevelPressure' [Pa] value to the correct Mean Sea Level Pressure (pressure reduced to the level of sea) in the region of launch.
      If launching in the near future, please look at Your weather forecast and weather models.

The pressureValue is sent via Horus v2, Horus v3 and APRS .

Suggested option - 1, using the RPM411 pressure sensor.*/
constexpr uint8_t pressureMode = 1;
constexpr float seaLevelPressure = 1013.25;                  // Sea level pressure in hPa, used to correctly estimate the pressure in the upper layers



/* Heating */

/* referenceHeating, referenceAreaTargetTemperature
Enabling 'referenceHeating' option provides advanced heating algorithm for the cut-out part of the PCB, where reference elements are located.
This function works with the same method as the Vaisala firmware - maintaing temperature of around 20*C at the cut out PCB area.
When enabled, warm resistors give a notable improvement in temperature readings accuracy, while also increasing the power consumption a bit.
Suggested setting: when sensor boom is present and You're flying with 2xAA batteries - set to true if You are doing a flight lasting up to 18h. With either 1xAA or no sensor boom - definitely false.

referenceAreaTargetTemperature - suggested values (temperature in degrees celcius):
  - 18, if flying with original styrofoam box (Vaisala default is 20C and with good thermal isolation NFW maintains 1.5C above target)
  - 8, if flying without original box, but with some form of wind cover like a few wraps of tape or foil or a small foil bag with a few layers.
  - 0, if flying with raw PCB. Raw PCB heating will consume much (I mean very much) more power, so beware of it when planning the flight. */
bool referenceHeating = true;                             // Enable option for reference area heating
constexpr int8_t referenceAreaTargetTemperature = 18;                  // Target temperature of reference area heating.


/* humidityModuleHeating, 
Enabling 'humidityModuleHeating' option provides advanced heating algorithm for humidity module. It safely and precisely adjusts the heater power using PWM and PID.
During flight, the humidity module is kept above 'humicapMinimumTemperature', because below that humicap sensor doesn't work that well.
Above 'humicapMinimumTemperature', the sensor is kept 'defrostingOffset' degrees above the air temperature.

Suggested values:
  - humidityModuleHeating = true when flying with the sensor boom (unless You do really long flights above 18h, then false).
  - defrostingOffset = 3, default for Vaisala algorithm is 5. Works very well to keep the sensor free of frost and condensation. Values higher than 5 shouldn't be used as they could affect the readings too much.
  - humicaMinimumTemperature = -44, below this value humicap's performance degrades much.
*/
bool humidityModuleHeating = true;
constexpr int8_t defrostingOffset = 5;                                 // This is the positive offset added to the sensor target temperature to prevent sensor frosting in icing conditions
constexpr int8_t humicapMinimumTemperature = -44;                      // Humicap sensor minimum operating temperature (below this value the sensor gets significantly less accurate and responsive)
constexpr int8_t humidityModuleHeatingTemperatureThreshold = 35;    // NFW will activate the heating only when the sensor's temperature is < this threshold. Vaisala heats at all times.



/* lowAltitudeFastTx mode
When sonde is descending after a burst, after it goes below 'lowAltitudeFastTxThreshold' altitude threshold, algorithms switch into a 'lowAltitudeFastTx' mode.
It is active for 'lowAltitudeFastTxDuration'. During this time it transmits Horus and APRS as fast as it can (or slower by setting the 'lowAltitudeFastTxInterval'), and only refreshes position and sensor data. */
constexpr uint16_t lowAltitudeFastTxThreshold = 1000;                    // Meter altitude threshold for this mode to enable (set to 0 to disable)
constexpr unsigned long lowAltitudeFastTxDuration = 480000;         // How long this mode will work, in milliseconds (default 480000ms = 8minutes)
constexpr uint16_t lowAltitudeFastTxInterval = 1;                        // Delay in ms between transmissions in this mode, leave at minimum 1 to provide the most data possible (note that it overloads Horus and APRS infrastructure very much for a short period of time).



/* Flight computing 
NFW firmware also processes collected data.
To ensure propper operation, look at these values and correct them if needed. */
constexpr uint16_t flightDetectionAltitude = 750;               // Flight detection altitude, if exceeded, the sonde knows that the flight began. I suggest to set it a few hundred meters above Your terrain level.
constexpr uint16_t burstDetectionThreshold = 800;               // Threshold value, which if exceeded (below maxAlt), deterimnes that the balloon has burst. Suggested 1000m, because some balloons get unsealed or float.



// System
bool autoResetEnable = true;                              // Automatically reset the CPU after specified time below, useful in stationary continuous use, to prevent from overflowing some variables and improving overall stability.
#define SYSTEM_RESET_PERIOD (7UL * 24 * 60 * 60 * 1000)   // 7 days in milliseconds
bool aprsToneCalibrationMode = false;                     // DON'T use for flight! transmits tones at 1200 and 2200 hz to calibrate the APRS delays for perfect sound frequencies, development mode, not for use
#define THERMISTOR_R25 10000                              // Onboard thermistor R value at 25C
#define THERMISTOR_B 3900                                 // Onboard thermistor Beta factor



/* buttonMode 
Specifies button operation mode:
  0 - Button operation disabled
  1 - Hold briefly for sonde shutdown
  2 - Extended menu mode, allowing to manually cancel 'improveGpsPerformance' and if the transmission should be enabled and shutdown. Holding the button cycles through 3 options, indicated by the LED, one blink is cancel the 'improveGpsPerformance', two blinks disable radio PA, and the longest hold shutdowns the sonde. Release on the desired option, quite hard so not suggested ever.

Suggested settings: Leave at 1 for a button shutdown and simple operation.
                    If You fly a sonde with PV or on 1xAA power supply, consider disabling the button and shorting its pins for always closed state.

NOTE: The button sometimes would need a longer hold than usual, about 3s instead of 1.5s, time varies due to CPU time availability, but never should exceed a longer hold.*/
constexpr int8_t buttonMode = 1;



/* dataRecorder
Set 'dataRecorderEnable' to true to enable this mode.
If active, the sonde transmits recorded, computed and debugging data to the ground via Horus V3 extraSensors.

Warning - APRS dataRecorder has been currently deprecated - there was a memory overflow issue somewhere, which I couldn't resolve and it caused the device to behave erraticaly. Also it seems to be a much better option in Horus V3 extaSensors array.
*/
bool dataRecorderEnable = true;
constexpr unsigned int dataRecorderInterval = 180000;               // 3 minutes frame interval by default (180000 milliseconds)
bool dataRecorderFlightNoiseFiltering = true;             // Filter out noisy data on ground and during position gathering, include in the measurements only the data captured in flight




// Data Kalman filter constants, no need to modify
float pressureKalmanError = 3;
float pressureKalmanQ = 2;
float humidityKalmanError = 2;
float humidityKalmanQ = 5;
float pressureKalmanEst = 1013.25; 
float pressureKalmanErrorEst = pressureKalmanError;
float humidityKalmanEst = 5;
float humidityKalmanErrorEst = humidityKalmanError;

/* PID values for heater control*/
float extHeaterProportionalK = 2.32;                      // proportional gain
float extHeaterIntegralK = 0.35;                          // integral gain
float extHeaterDerivativeK = 0.85;                        // derivative gain


/* ===== END OF THE RS41-NFW CONFIGURATION SECTION ===== */






//Support of ability to heat up the radio/gps oscillator by reference heating resistors has ENDED. This is due to the decision, that RTTY (especially 75 baud) is not used widely, so the heating isn't needed, and also that the heating of them at max power is very power hungry. Also, they are used for temperature calibration and now have been implemented to correct the readings properly. This can be configured in the sensorBoom area
/*
//Oscillator heating system (don't activate unless You plan to use fast RTTY in extremely small temperatures and fly with large batteries)
int refHeatingMode = 0;                    //0 - off, 1 - auto, 2 - always on
int refHeaterAutoActivationHeight = 0;     //set to 0 to disable auto heater enable above set level, if other than 0 then it means height above which the heater gets auto enabled
unsigned long heaterWorkingTimeSec = 600;  //heater heating time
unsigned long heaterCooldownTimeSec = 3;   //heater cooldown time after heating process
int autoHeaterThreshold = 6;               //auto heater temperature threshold, WARNING! If rtty used baud is faster than 45bdr, the threshold should be at 14*C to prevent the radio from loosing PLL-lock, best would be even to set the heating to always on. If only horus mode is used, it is not mandatory, altough for standard flights that dont require more than 12h of operation the 6*C is advised for defrosting and keeping the internals slightly above ice temperature.
int refHeaterCriticalDisableTemp = 72;     //heater critical temp which disables the heating if exceeded
int refHeaterCriticalReenableTemp = 67;    //heater temperature at which heating gets re-enabled after being cut down due to too high temperature*/

//Support for the first development release of humidity module heating has ended, these options are no longer available, please use the new 'humidityModuleHeating'
/*bool humidityModuleHeating = false; //This option enables the defrosting of humidity module and 'prevents condensation'. This function works the same as in Vaisala firmware, keeping the module slightly warmer than air, 5K above air temperature. NOTE: May alter the readings.
int humidityModuleHeatingAmount = 4; //algorithms will keep the temperature by this number more than air temperature [K]
int heatingPwmUpperLimit = 75; //These three values shouldn't be changed, unless You know what they mean. Max PWM value of humidity module heating
int heatingPwmCurrentValue = 3; //default start value, should be around the lower limit
int heatingPwmLowerLimit = 3; //minimum PWM value of heating
int heatingTemperatureThreshold = 2; //turns on only in conditions where condensation would be really possible, offers more accurate readings
int heatingHumidityThreshold = 90; //turns on only in conditions where condensation would be really possible*/

//===== System internal variables, shouldn't be changed here
uint8_t btnCounter = 0;
uint8_t bufPacketLength = 64;
uint16_t txRepeatCounter = 0;
float batVFactor = 1.0;
bool ledsEnable = ledStatusEnable;  //internal boolean used for height disable etc.
String rttyMsg;
String morseMsg;
unsigned long gpsTime;
uint8_t gpsHours;
uint8_t gpsMinutes;
uint8_t gpsSeconds;
float gpsSpeed;
float gpsSpeedKph = 0;
uint8_t gpsSats;  //system wide variables, for use in functions that dont read the gps on their own
float gpsHdop;
bool err = false;   //const red light, status state
bool warn = false;  //orange light, status state
bool ok = true;
;  //green light, status state
bool vBatWarn = false;
bool gpsFixWarn = false;
int horusPacketCount;
int horusV3PacketCount;
uint8_t xdataInstrumentType = 0;
int xdataInstrumentNumber = 0;
float xdataOzonePumpTemperature = 0;
float xdataOzoneCurrent = 0;
float xdataOzonewxVoltage = 0;
int xdataOzonePumpCurrent = 0;
float lastGpsAlt;
unsigned long lastGpsAltMillisTime = 1;
float vVCalc;
bool gpsTimeoutCounterActive = false;
unsigned long gpsTimerBegin = 0;
int currentGPSPowerMode = 0;  // 1 normal (max powerformance/continuous), 2 powersave
unsigned long lastPowerSaveChange = 0;
unsigned int rttyFrameCounter = 0;
unsigned long lowAltitudeFastTxModeBeginTime = 0;
unsigned int gpsResetCounter = 0;
unsigned long lastDataRecorderTransmission = 0;
unsigned long landingTimeMillis = 0;
bool cancelGpsImprovement = false;
bool gpsJamWarning = false;
int8_t currentRadioPwrSetting = 0;
int8_t currentM10IntelligentMode = 1;
int8_t gpsStatus = 1;

uint16_t maxAlt = 0;
int16_t maxSpeed = 0;
int maxAscentRate = 0;
int maxDescentRate = 0;
int8_t maxMainTemperature = 0;
int8_t minMainTemperature = 0;
int8_t maxInternalTemp = 0;
int8_t minInternalTemp = 0;
bool beganFlying = false;
bool burstDetected = false;
bool recorderInitialized = false;  //not init by default
bool hasLanded = false;
bool lowAltitudeFastTxModeEnd = false;

// Sensor boom
float mainTemperatureFrequency;
float mainTemperaturePeriod;
float mainTemperatureResistance;
float mainTemperatureValue;
float extHeaterTemperatureFrequency;
float extHeaterTemperaturePeriod;
float extHeaterTemperatureResistance;
float extHeaterTemperatureValue;
float humidityFrequency;
float zeroHumidityFrequency;
float maxHumidityFrequency;
uint16_t humidityRangeDelta = 850; //old humidity measurement method, depreciated
float refCapHighFrequency;
float refCapLowFrequency;
float humidityCapacitance;
float maxHumidityCapacitance;
uint16_t humidityValue;
float pressureValue;
float tempSensorBoomCalibrationFactor = 0;
bool sensorBoomMainTempError = false;
bool sensorBoomHumidityModuleError = false;
bool sensorBoomFault = false;
bool calibrationError = false;
int extHeaterPwmStatus = 0;
int referenceHeaterStatus = 0;


// APRS - misc.
bool aprsTone = 0;
char statusMessage[320];  // Status message
char aprsLocationMsg[32];
char aprsOthersMsg[256];
char aprsWxMsg[256];
char aprsBitStuffingCounter = 0;  // Bit stuffing counter
unsigned short aprsCrc = 0xffff;  // CRC for error checking
unsigned int aprsPacketNum = 0;


// Scheduler:
// Time tracking variables
unsigned long systemTimeMillis = 0;        // System clock in milliseconds
unsigned long lastMillisUpdate = 0;        // Last millis() reading
bool gpsTimeSynced = false;                // Whether we have GPS time sync

// Transmission tracking - stores next scheduled transmission time in seconds
unsigned long nextPipTxTime = 0;
unsigned long nextHorusV3TxTime = 0;
unsigned long nextHorusTxTime = 0;
unsigned long nextAprsTxTime = 0;
unsigned long nextRttyTxTime = 0;
unsigned long nextMorseTxTime = 0;




#ifdef RSM4x4
//Based on https://github.com/cturvey/RandomNinjaChef/blob/main/uBloxHABceiling.c , and  https://github.com/Nevvman18/rs41-nfw/issues/3
uint8_t ubxCfgValSet_dynmodel6[] = {                     // Series 9 and 10
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00,                    // Header/Command/Size  UBX-CFG-VALSET (RAM)
  0x00, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x06,  // Payload data (0x20110021 CFG-NAVSPG-DYNMODEL = 6)
  0xF2, 0x4F
};  //hardcoded checksum

uint8_t ubxCfgValGet_dynmodel6[] = {
  0xB5, 0x62, 0x06, 0x8B, 0x08, 0x00,  // Header for UBX-CFG-VALGET
  0x00, 0x00, 0x00, 0x00,              // Reserved
  0x21, 0x00, 0x11, 0x20,              // Key for CFG-NAVSPG-DYNMODEL
  0xEB, 0x57                           //hardcoded checksum
};

// More new messages for M10:

uint8_t ubxCfgValSet_msgRate4Hz[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00,                    // Header
  0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0xFA, 0x00, // 250ms (0x00FA)
  0xE7, 0xE5                                             // Checksum
};

uint8_t ubxCfgValSet_enableGns[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xB6, 0x00, 0x91, 0x20, 0x01, 0x02, 0xB3
};

uint8_t ubxCfgValSet_disableGga[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xBB, 0x00, 0x91, 0x20, 0x00, 0x06, 0xCB
};

uint8_t ubxCfgValSet_navRate2500[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00 ,0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0xC4, 0x09, 0xBA, 0x82                                            // Checksum
};

uint8_t ubxCfgValSet_enableSuperS[] = { // Super-S power saving mode
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xD6, 0x00, 0x11, 0x20, 0xFF, 0xA0, 0xD1
};
/*uint8_t ubxCfgValSet_disableSuperS[] = { // Super-S power saving mode
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xD6, 0x00, 0x11, 0x20, 0x00, 0xA1, 0xD2
};*/

uint8_t ubxCfgValSetPsmct[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0xD0, 0x20, 0x02, 0x8D, 0xE8
};

uint8_t ubxCfgValSetPsmctPeriod10[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0xD0, 0x40, 0x0A, 0x00, 0x00, 0x00, 0x00, 0xB9, 0x81
};

uint8_t ubxCfgValSetContinuous[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0xD0, 0x20, 0x00, 0x8B, 0xE6
};

uint8_t ubxEnableGal[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x21, 0x00, 0x31, 0x10, 0x01, 
  0xFD, 0x8A
};
uint8_t ubxEnableGlo[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x25, 0x00, 0x31, 0x10, 0x01, 
  0x01, 0x9E
};
uint8_t ubxDisableGal[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x21, 0x00, 0x31, 0x10, 0x00, 
  0xFC, 0x89
};
uint8_t ubxDisableGlo[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x25, 0x00, 0x31, 0x10, 0x00, 
  0x00, 0x9D
};
uint8_t ubxEnableGps[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x1F, 0x00, 0x31, 0x10, 0x01, 
  0xFB, 0x80
};
uint8_t ubxEnableBds[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 
  0x00, 0x01, 0x00, 0x00, 
  0x22, 0x00, 0x31, 0x10, 0x01, 
  0xFE, 0x8F
};


// CFG-NAVSPG-MAX_SVS set to 64 (0x40)
uint8_t ubxCfgValSet_maxSvs64[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00,          // Header
  0x00, 0x01, 0x00, 0x00,                      // Layer: RAM
  0x21, 0x00, 0x11, 0x20, 0x40,                // Key ID for MAX_SVS, Value: 64
  0x1A, 0x02                                   // Checksum
};

// Elevation: 3 deg | C/N0: 10 dBHz
uint8_t ubxCfgElev3[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xA3, 0x00, 0x11, 0x20, 0x0A, 0x78, 0xDD
};

uint8_t ubxCfgSig10[] = {
  0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xA4, 0x00, 0x11, 0x20, 0x02, 0x71, 0xDA
};

#endif


// 6 series

uint8_t ubxCfgNav5_dynmodel6[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,
  0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
  0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
  0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x4D, 0xDB
};

uint8_t ubxCfgNav5_maxPerformance[] = {
  //ublox 6-series max performance mode
  0xB5, 0x62, 0x06, 0x11, 0x02, 0x00,  // Header/Command/Size
  0x08, 0x00,                          // Payload
  0x21, 0x91                           // Checksum
};

uint8_t ubxCfgNav5_powerSave[] = {
  // ublox 6-series powersave mode
  0xB5, 0x62, 0x06, 0x11, 0x02, 0x00,  // Header/Command/Size
  0x08, 0x01,                          // Payload
  0x22, 0x92                           // Checksum
};


//===== Horus mode deifinitions
// Horus v2 Mode 1 (32-byte) Binary Packet
//== FOR MORE INFO SEE int build_horus_binary_packet_v2(char *buffer) function around line 600
struct HorusBinaryPacketV2 {
  uint16_t PayloadID;
  uint16_t Counter;
  uint8_t Hours;
  uint8_t Minutes;
  uint8_t Seconds;
  float Latitude;
  float Longitude;
  uint16_t Altitude;
  uint8_t Speed;  // Speed in Knots (1-255 knots)
  uint8_t Sats;
  int8_t Temp;          // Twos Complement Temp value.
  uint8_t BattVoltage;  // 0 = 0.5v, 255 = 2.0V, linear steps in-between.
  // The following 9 bytes (up to the CRC) are user-customizable. The following just
  // provides an example of how they could be used.
  int16_t dummy1;     // unsigned int uint8_t
  int16_t dummy2;     // Float float
  uint8_t dummy3;     // battery voltage test uint8_t
  uint16_t dummy4;    // divide by 10 uint8_t
  uint16_t dummy5;    // divide by 100 uint16_t
  uint16_t Checksum;  // CRC16-CCITT Checksum.
} __attribute__((packed));

// Buffers and counters.
// QI - Horus v2 - 32 bytes uncoded -> 65 bytes coded.
// QI - Horus v3 48 bytes -> 94 bytes coded
#define HORUS_UNCODED_BUFFER_SIZE 128
#define HORUS_CODED_BUFFER_SIZE 256
char rawbuffer[HORUS_UNCODED_BUFFER_SIZE];    // Buffer to temporarily store a raw binary packet.
// QI - Expanded to 256 bytes to fit the big 128 byte (Coded) v3 packet
char codedbuffer[HORUS_CODED_BUFFER_SIZE];  // Buffer to store an encoded binary packet
char debugbuffer[256];  // Buffer to store debug strings

uint32_t fsk4_base = 0, fsk4_baseHz = 0;
uint32_t fsk4_shift = 0, fsk4_shiftHz = 0;
uint32_t fsk4_bitDuration;
uint32_t fsk4_tones[4];
uint32_t fsk4_tonesHz[4];

//===== Morse mode definitions
// Morse code mapping for letters A-Z, digits 0-9, and space
const char* MorseTable[37] = {
  ".-",     // A
  "-...",   // B
  "-.-.",   // C
  "-..",    // D
  ".",      // E
  "..-.",   // F
  "--.",    // G
  "....",   // H
  "..",     // I
  ".---",   // J
  "-.-",    // K
  ".-..",   // L
  "--",     // M
  "-.",     // N
  "---",    // O
  ".--.",   // P
  "--.-",   // Q
  ".-.",    // R
  "...",    // S
  "-",      // T
  "..-",    // U
  "...-",   // V
  ".--",    // W
  "-..-",   // X
  "-.--",   // Y
  "--..",   // Z
  "-----",  // 0
  ".----",  // 1
  "..---",  // 2
  "...--",  // 3
  "....-",  // 4
  ".....",  // 5
  "-....",  // 6
  "--...",  // 7
  "---..",  // 8
  "----.",  // 9
  " "       // Space (7 dot lengths)
};






char RPM411SerialNumber[9];

//1 byte uint8_t-stored configuration frame for rpm411 (captured by nevvman's logic analyzer)
const uint8_t RPM411InitFrame[21][7] = {
  {0x03, 0x02, 0x1E, 0x00, 0x00, 0x51, 0x00},
  {0x03, 0x02, 0x28, 0x00, 0x33, 0xFE, 0x00},
  {0x03, 0x02, 0x0A, 0x00, 0xB7, 0x9E, 0x00},
  {0x03, 0x02, 0x64, 0x00, 0x92, 0xB6, 0x00},
  {0x03, 0x02, 0x6E, 0x00, 0x59, 0x59, 0x00},
  {0x03, 0x02, 0x78, 0x00, 0x8c, 0xF0, 0x00},
  {0x03, 0x02, 0x82, 0x00, 0x86, 0x0C, 0x00},
  {0x03, 0x02, 0x8C, 0x00, 0x89, 0x2F, 0x00},
  {0x03, 0x02, 0x96, 0x00, 0x31, 0xC3, 0x00},
  {0x03, 0x02, 0xA0, 0x00, 0x02, 0x6C, 0x00},
  {0x03, 0x02, 0xAA, 0x00, 0xC9, 0x83, 0x00},
  {0x03, 0x02, 0xB4, 0x00, 0xB5, 0xA3, 0x00},
  {0x03, 0x02, 0xBE, 0x00, 0x7E, 0x4C, 0x00},
  {0x03, 0x02, 0xC8, 0x00, 0x81, 0xEE, 0x00},
  {0x03, 0x02, 0xD2, 0x00, 0x39, 0x02, 0x00},
  {0x03, 0x02, 0xDC, 0x00, 0x36, 0x21, 0x00},
  {0x03, 0x02, 0xE6, 0x00, 0x68, 0xCB, 0x00},
  {0x03, 0x02, 0xF0, 0x00, 0xBD, 0x62, 0x00},
  {0x03, 0x02, 0xFA, 0x00, 0x76, 0x8D, 0x00},
  {0x03, 0x02, 0x04, 0x01, 0x99, 0xAD, 0x00},
  {0x03, 0x02, 0x0E, 0x01, 0x52, 0x42, 0x00}
};

const uint8_t RPM411PreReadoutFrame[] = {0x01,0x00,0x3E,0x2E,0x00};
const uint8_t RPM411TriggerReadoutFrame[] = {0x02,0x00,0x6D,0x7B,0x00};

//lets hope it wont run out of RAM
uint8_t RPM411ConfigData[21][33];

bool rpm411Error = false;
float rpm411Pressure = 0.0;
float rpm411InternalTemperature = 0.0;
uint8_t RPM411ReadingsData[33];





//===== Radio config functions

void initSi4032(void) {                  //initial radio config, it is dynamic in code but this is the entry configuration for cw to work
  writeRegister(0x06, 0x00);             // Disable all interrupts
  writeRegister(0x07, 0x01);             // Set READY mode
  writeRegister(0x09, 0x7F);             // Cap = 12.5pF
  writeRegister(0x0A, 0x05);             // Clk output is 2MHz
  writeRegister(0x0B, 0xF4);             // GPIO0 is for RX data output
  writeRegister(0x0C, 0xEF);             // GPIO1 is TX/RX data CLK output
  writeRegister(0x0D, 0x00);             // GPIO2 for MCLK output
  writeRegister(0x0E, 0x00);             // GPIO port use default value
  writeRegister(0x0F, 0x80);             // adc config
  writeRegister(0x10, 0x00);             // offset or smth idk
  writeRegister(0x12, 0x20);             // temp sensor calibration
  writeRegister(0x13, 0x00);             // temp offset
  writeRegister(0x70, 0x20);             // No manchester code, no data whitening, data rate < 30Kbps
  writeRegister(0x1C, 0x1D);             // IF filter bandwidth
  writeRegister(0x1D, 0x40);             // AFC Loop
  writeRegister(0x20, 0xA1);             // Clock recovery
  writeRegister(0x21, 0x20);             // Clock recovery
  writeRegister(0x22, 0x4E);             // Clock recovery
  writeRegister(0x23, 0xA5);             // Clock recovery
  writeRegister(0x24, 0x00);             // Clock recovery timing
  writeRegister(0x25, 0x0A);             // Clock recovery timing
  writeRegister(0x2C, 0x00);             // Unused register
  writeRegister(0x2D, 0x00);             // Unused register
  writeRegister(0x2E, 0x00);             // Unused register
  writeRegister(0x6E, 0x27);             // TX data rate 1
  writeRegister(0x6F, 0x52);             // TX data rate 0
  writeRegister(0x30, 0x8C);             // Data access control
  writeRegister(0x32, 0xFF);             // Header control
  writeRegister(0x33, 0x42);             // Header control
  writeRegister(0x34, 64);               // 64 nibble = 32 byte preamble
  writeRegister(0x35, 0x20);             // Preamble detection
  writeRegister(0x36, 0x2D);             // Synchronize word
  writeRegister(0x37, 0xD4);             // Synchronize word
  writeRegister(0x3A, 's');              // Set TX header 3
  writeRegister(0x3B, 'o');              // Set TX header 2
  writeRegister(0x3C, 'n');              // Set TX header 1
  writeRegister(0x3D, 'g');              // Set TX header 0
  writeRegister(0x3E, bufPacketLength);  // Set packet length to 16 bytes
  writeRegister(0x3F, 's');              // Set RX header
  writeRegister(0x40, 'o');              // Set RX header
  writeRegister(0x41, 'n');              // Set RX header
  writeRegister(0x42, 'g');              // Set RX header
  writeRegister(0x43, 0xFF);             // Check all bits
  writeRegister(0x44, 0xFF);             // Check all bits
  writeRegister(0x45, 0xFF);             // Check all bits
  writeRegister(0x46, 0xFF);             // Check all bits
  writeRegister(0x56, 0x01);             // Unused register
  writeRegister(0x6D, 0x07);             // TX power to min (max = 0x07)
  writeRegister(0x79, 0x00);             // No frequency hopping
  writeRegister(0x7A, 0x00);             // No frequency hopping
  writeRegister(0x72, 0x08);
  writeRegister(0x73, 0x00);  // No frequency offset
  writeRegister(0x74, 0x00);  // No frequency offset
  writeRegister(0x75, 0x21);  // Frequency set to 435MHz 36/40
  writeRegister(0x76, 0x18);  // Frequency set to 435MHz
  writeRegister(0x77, 0x0A);  // Frequency set to 435Mhz
  writeRegister(0x5A, 0x7F);  // Unused register
  writeRegister(0x59, 0x40);  // Unused register
  writeRegister(0x58, 0x80);  // Unused register
  writeRegister(0x6A, 0x0B);  // Unused register
  writeRegister(0x68, 0x04);  // Unused register
  writeRegister(0x1F, 0x03);  // Unused register
}

void radioSoftReset() {
  writeRegister(0x07, 0x80);
}

void radioEnableTx() {
  // Modified to set the PLL and Crystal enable bits to high. Not sure if this makes much difference.
  writeRegister(0x07, 0x4B);
}

void radioInhibitTx() {
  // Sleep mode, but with PLL idle mode enabled, in an attempt to reduce drift on key-up.
  writeRegister(0x07, 0x43);
}

void radioDisableTx() {
  writeRegister(0x07, 0x40);
}



//===== Low-Level SPI Communication for Si4032

void writeRegister(uint8_t address, uint8_t data) {
  digitalWrite(CS_RADIO_SPI, LOW);
  SPI_2.transfer(address | 0x80);  // Write mode using SPI2
  SPI_2.transfer(data);
  digitalWrite(CS_RADIO_SPI, HIGH);
}

uint8_t readRegister(uint8_t address) {
  uint8_t result;
  digitalWrite(CS_RADIO_SPI, LOW);
  SPI_2.transfer(address & 0x7F);  // Read mode using SPI2
  result = SPI_2.transfer(0x00);
  digitalWrite(CS_RADIO_SPI, HIGH);
  return result;
}

void setRadioFrequency(const float frequency_mhz) {  //adapted from RS41ng
  uint8_t hbsel = (uint8_t)((frequency_mhz * (30.0f / SI4032_CLOCK)) >= 480.0f ? 1 : 0);

  uint8_t fb = (uint8_t)((((uint8_t)((frequency_mhz * (30.0f / SI4032_CLOCK)) / 10) - 24) - (24 * hbsel)) / (1 + hbsel));
  uint8_t gen_div = 3;  // constant - not possible to change!
  uint16_t fc = (uint16_t)(((frequency_mhz / ((SI4032_CLOCK / gen_div) * (hbsel + 1))) - fb - 24) * 64000);
  writeRegister(0x75, (uint8_t)(0b01000000 | (fb & 0b11111) | ((hbsel & 0b1) << 5)));
  writeRegister(0x76, (uint8_t)(((uint16_t)fc >> 8U) & 0xffU));
  writeRegister(0x77, (uint8_t)((uint16_t)fc & 0xff));
}

void setRadioPower(uint8_t power) {
  writeRegister(0x6D, power & 0x7U);
  currentRadioPwrSetting = power;
}

void setRadioOffset(uint16_t offset) {
  writeRegister(0x73, offset);
  writeRegister(0x74, 0);
}

void setRadioSmallOffset(uint8_t offset) {
  writeRegister(0x73, offset);
}

void setRadioDeviation(uint8_t deviation) {
  // The frequency deviation can be calculated: Fd = 625 Hz x fd[8:0].
  // Zero disables deviation between 0/1 bits
  writeRegister(0x72, deviation);
}

void setRadioModulation(int modulationNumber) {
  /* modulationNumber:
    = 0 = no modulation (CW)
    = 1 = OOK
    = 2 = FSK
    = 3 = GFSK
    */

  if (modulationNumber == 0) {
    writeRegister(0x71, 0x00);  //cw
  } else if (modulationNumber == 1) {
    writeRegister(0x71, 0b00010001);  //OOK
  } else if (modulationNumber == 2) {
    writeRegister(0x71, 0b00010010);  //FSK, FIFO source mode
  } else if (modulationNumber == 3) {
    writeRegister(0x71, 0x22);  //GFSK
  } else {
    writeRegister(0x71, 0x00);  //error handling
  }
}



//===== Radio modes functions

//rtty
void sendRTTYPacket(const char* message) {
  rttySendBit(1);
  delay(100);
  while (*message) {
    rttySendCharacter(*message++);
  }
}

void rttySendBit(bool bitValue) {
  if (bitValue) {
    setRadioSmallOffset(RTTY_RADIO_MARK_OFFSET);
  } else {
    setRadioSmallOffset(RTTY_RADIO_SPACE_OFFSET);
  }
  delayMicroseconds(rttyBitDelay);  // Fixed delay according to baud rate
}



void rttySendStopBits() {
  // Send stop bits
  setRadioSmallOffset(RTTY_RADIO_MARK_OFFSET);
  delayMicroseconds(int(rttyStopBits * rttyBitDelay));  // Delay of stop bits
}


void rttySendCharacter(char character) {
  // Encode character to RTTY format (assuming default encoding and no special encoding needed for . and -)
  // Send start bits
  rttySendBit(0);

  // Character encoding (use default encoding, 7-bit or 8-bit)
  uint8_t encoding = (uint8_t)character;  // Assuming ASCII encoding for characters
  for (int i = 0; i < rttyBits; i++) {
    rttySendBit((encoding >> i) & 0x01);
  }

  // Send stop bits (1.5 stop bits)
  rttySendStopBits();  // stop bit
}


//morse
void transmitMorseChar(const char* morseChar, int unitTime) {
  while (*morseChar) {
    if (*morseChar == '.') {
      // Dot: 1 unit time on
      radioEnableTx();
      delay(unitTime);
      radioDisableTx();
    } else if (*morseChar == '-') {
      // Dash: 3 unit times on
      radioEnableTx();
      delay(3 * unitTime);
      radioDisableTx();
    }
    // Space between elements: 1 unit time off
    delay(unitTime);

    morseChar++;

    buttonHandler();
  }
}

void transmitMorseString(const char* str, int unitTime) {
  while (*str) {
    char c = *str++;

    // Handle letters (A-Z) and digits (0-9)
    if (c >= 'A' && c <= 'Z') {
      transmitMorseChar(MorseTable[c - 'A'], unitTime);
    } else if (c >= '0' && c <= '9') {
      transmitMorseChar(MorseTable[c - '0' + 26], unitTime);
    } else if (c == ' ') {
      // Word space: 7 unit times off
      delay(7 * unitTime);
    }
    // Space between characters: 3 unit times off
    delay(3 * unitTime);
  }
}


//4fsk implementation for horus
void fsk4_tone(int t) {
  switch (t) {
    case 0:
      setRadioSmallOffset(0x01);
      break;
    case 1:
      setRadioSmallOffset(0x02);
      break;
    case 2:
      setRadioSmallOffset(0x03);
      break;
    case 3:
      setRadioSmallOffset(0x04);
      break;
    default:
      setRadioSmallOffset(0x01);
  }

  delayMicroseconds(fsk4_bitDuration);
}

void fsk4_idle() {
  fsk4_tone(0);
}

void fsk4_preamble(uint8_t len) {
  int k;
  for (k = 0; k < len; k++) {
    fsk4_writebyte(0x1B);
  }
}

size_t fsk4_writebyte(uint8_t b) {
  int k;
  // Send symbols MSB first.
  for (k = 0; k < 4; k++) {
    // Extract 4FSK symbol (2 bits)
    uint8_t symbol = (b & 0xC0) >> 6;
    // Modulate
    fsk4_tone(symbol);
    // Shift to next symbol.
    b = b << 2;
  }

  return (1);
}

size_t fsk4_write(char* buff, size_t len) {
  size_t n = 0;
  for (size_t i = 0; i < len; i++) {
    n += fsk4_writebyte(buff[i]);
  }
  return (n);
}



//===== Radio payload creation
String createRttyMorsePayload() {
  rttyFrameCounter++;
  // Start with the payload string
  String payload, payloada;

  // Convert gpsTime (long unsigned int) to String and format as HH:MM:SS
  char formattedTime[8];
  String formattedTimeStr = "";
  int timelen = sprintf(formattedTime, "%02d:%02d:%02d", gpsHours, gpsMinutes, gpsSeconds);
  for (int i = 0; i < timelen; i++) formattedTimeStr += formattedTime[i];
  // Format latitude and longitude to 5 decimal places
  String formattedLat = String(gpsLat, 5);
  String formattedLong = String(gpsLong, 5);

  int rttyTemperature;

  if (sensorBoomFault || !sensorBoomEnable) {
    rttyTemperature = static_cast<int>(readAvgIntTemp());
  } else {
    rttyTemperature = static_cast<int>(mainTemperatureValue);
  }

  char formattedTemp[8];
  int lentemp = sprintf(formattedTemp, "%d", rttyTemperature);
  String formattedTempStr;
  for (int i = 0; i < lentemp; i++) {
    formattedTempStr += formattedTemp[i];
  }

  int intAlt = (unsigned int)gpsAlt;

  // Build the payload following the UKHAS format
  payload = String(CALLSIGN) + "," + String(rttyFrameCounter) + "," + formattedTimeStr + "," + formattedLat + "," + formattedLong + "," + String(intAlt) + "," + String(gpsSats) + "," + String(readBatteryVoltage(), 2) + "," + formattedTempStr;

  payload.toUpperCase();

  // Calculate CRC16 checksum
  unsigned int crcValue = rttyCrc16Checksum((unsigned char*)payload.c_str(), payload.length());
  char crcBuffer[5];
  snprintf(crcBuffer, sizeof(crcBuffer), "%04X", crcValue);

  // Append the CRC16 checksum
  payloada = "$$$$" + payload + "*" + String(crcBuffer) + "\n";

  return payloada;
}


// Horus V3 mode - protocol and code provided by Mark VK5QI - big thanks for awesome work on code and the protocol!!!
int buildHorusV3Packet(char* uncoded_buffer){
  // Horus v3 packets are encoded using ASN1, and are encapsulated in packets
  // of sizes 32, 48, 64, 96 or 128 bytes (before coding)
  // The CRC16 for these packets is located at the *start* of the packet, still little-endian encoded

  // Erase the uncoded buffer
  // This has the effect of padding out the unused bytes in the packet with zeros
  memset(uncoded_buffer, 0, HORUS_UNCODED_BUFFER_SIZE);

  // Increment packet count
  horusV3PacketCount++;

  // Should check how this is allocated in memory.

  // Hardcoded dummy test packet. 
  // Need to check how this is allocated in memory. how much it uses.
  // .. also does it get cleared?


  horusTelemetry asnMessage = {
        .payloadCallsign  = HORUS_V3_CALLSIGN,
        .sequenceNumber = horusV3PacketCount,
        .timeOfDaySeconds  = gpsHours*3600 + gpsMinutes*60 + gpsSeconds,
        .latitude = (int)(gpsLat*100000),
        .longitude = (int)(gpsLong*100000),
        .altitudeMeters = gpsAlt,
        // Example of adding some custom fields.
        .extraSensors = {
          .nCount=1, // Number of custom fields. CHANGE IT IF YOU WANT TO ADD FOR EXAMPLE HEATERS DATA COMMENTED OUT
          .arr = {
            // Example of an array of integers 
            {
                .name = "gps", // This is transmitted in the packet if .exist/name is true
                .values = {
                    .kind = horusInt_PRESENT,
                    .u = {
                        .horusInt = {
                          .nCount = 2,
                            .arr = {gpsStatus, gpsHdop},
                        }
                    }
                },
                .exist = {
                    .name = true,
                    .values = true,
                },
                
                
            }/*,
            {
                .name = "heat", // This is transmitted in the packet if .exist/name is true
                .values = {
                    .kind = horusInt_PRESENT,
                    .u = {
                        .horusInt = {
                          .nCount = 2,
                            .arr = {referenceHeaterStatus, extHeaterPwmStatus},
                        }
                    }
                },
                .exist = {
                    .name = true,
                    .values = true,
                },
                
                
            }*/
          },
        },
        .velocityHorizontalKilometersPerHour = gpsSpeedKph,
        .gnssSatellitesVisible = gpsSats,
        .ascentRateCentimetersPerSecond = vVCalc * 100, // m/s -> cm/s
        .pressurehPa_x10 = (int)(pressureValue*10),
        .temperatureCelsius_x10 = {
            .internal = readAvgIntTemp()*10,
            .external = mainTemperatureValue*10,
            .custom1 = extHeaterTemperatureValue*10,
            // I'm not sure we need to explicitly indicate which of these fields exist, but just to be safe...
            .exist = {
                .internal = true,
                .external = true,
                .custom1 = true,
                .custom2 = false
            }
        },
        .humidityPercentage = humidityValue,
        .milliVolts = {
            .battery = (int)(readBatteryVoltage()*1000),
            // I'm not sure we need to explicitly indicate which of these fields exist, but just to be safe...
            .exist = {
                .battery = true,
                .solar = false,
                .custom1 = false,
                .custom2 = false
            }
        },
        // We need to explicitly specify which optional fields we want to include in the packet
        .exist = {
            .extraSensors = true,
            .velocityHorizontalKilometersPerHour = true,
            .gnssSatellitesVisible = true,
            .ascentRateCentimetersPerSecond = true,
            .pressurehPa_x10 = true,
            .temperatureCelsius_x10 = true,
            .humidityPercentage = true,
            .milliVolts = true
        }
    };

    if(!horusV3LongerPacket) {
      asnMessage.temperatureCelsius_x10.exist.custom1 = false;
      // Disable the extraSensors array
      asnMessage.exist.extraSensors = false;
    }

    // Conditionally disable some of the fields if we have no valid data source for them

    // Don't send external temp and humidity data if the sensor boom isn't in use
    if (sensorBoomEnable == false){
      asnMessage.temperatureCelsius_x10.exist.external = false;
      asnMessage.exist.humidityPercentage = false;
      asnMessage.temperatureCelsius_x10.exist.custom1 = false;
    }

    // The encoder needs a data structure for the serialization
    // Again - how much memory is allocated here?
    BitStream encodedMessage;

    // The Encoder may fail and update an error code
    int errCode;

    // Initialization associates the buffer to the bit stream
    // We want to write the uncoded message starting at 2 bytes into the message.

    BitStream_Init (&encodedMessage,
                    (unsigned char*)(uncoded_buffer+2),
                    HORUS_UNCODED_BUFFER_SIZE-1
    );
    // Originally this function call used a MUCH larger value for count
    //horusTelemetry_REQUIRED_BYTES_FOR_ENCODING);
    
    // Encode the message using uPER encoding rule

    // We patch in assert functionality in assert_override.h
    // Before running encode we set assert_value = 0
    // Then check the value in assert_value
    assert_value = 0;

    if (!horusTelemetry_Encode(&asnMessage,
                        &encodedMessage,
                        &errCode,
                        true) || assert_value != 0)
    {  
        // Not at this error helps that much in a flight, but it helps
        // us when debugging!   
        if (xdataPortMode == 1) {
          if(errCode > 0){
            xdataSerial.print("[ERR]: HORUS v3 Encoding Failed: ");
            xdataSerial.println(errCode);
          }
          if(assert_value != 0){
            xdataSerial.println("[ERR]: HORUS v3 Assert Failure, maybe hit buffer size limit");
          }
        }
        // Need to check what happens here.
        return 0;
    }
    else 
    {
        // Encoding was successful!
        // Now we need to figure out the required frame size, and add the CRC.
        int encodedSize = BitStream_GetLength(&encodedMessage);

        // Determine the required frame size.
        // Probably should do this from a list of valid sizes in a neater manner
        int frameSize = 128;
        if (encodedSize <= 30){
          frameSize = 32;
        } else if (encodedSize <= 46){
          frameSize = 48;
        } else if (encodedSize <= 62){
          frameSize = 64;
        } else if (encodedSize <= 94){
          frameSize = 96;
        } else if (encodedSize <= 126){
          frameSize = 128;
        }

        // Calculate CRC16 over the frame, starting at byte 2
        uint16_t packetCrc = (uint16_t)crc16((unsigned char *)(uncoded_buffer + 2),
                                     frameSize - 2);
        // Write CRC into bytes 0–1 of the packet
        memcpy(uncoded_buffer, &packetCrc, sizeof(packetCrc));  // little‑endian on STM32

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: HORUS v3 ASN1: ");
          xdataSerial.print(encodedSize);
          xdataSerial.print(" Frame: ");
          xdataSerial.println(frameSize);
        }

        return frameSize;
    }

    return 0;
}



int buildHorusV3PacketDataRecorder(char* uncoded_buffer){
  // Horus v3 packets are encoded using ASN1, and are encapsulated in packets
  // of sizes 32, 48, 64, 96 or 128 bytes (before coding)
  // The CRC16 for these packets is located at the *start* of the packet, still little-endian encoded

  // Erase the uncoded buffer
  // This has the effect of padding out the unused bytes in the packet with zeros
  memset(uncoded_buffer, 0, HORUS_UNCODED_BUFFER_SIZE);

  // Increment packet count
  horusV3PacketCount++;


  horusTelemetry asnMessage = {
      .payloadCallsign = HORUS_V3_CALLSIGN,
      .sequenceNumber = horusV3PacketCount,
      .timeOfDaySeconds = gpsHours * 3600 + gpsMinutes * 60 + gpsSeconds,
      .latitude = (int)(gpsLat * 100000),
      .longitude = (int)(gpsLong * 100000),
      .altitudeMeters = gpsAlt,
      
      .extraSensors = {
          .nCount = 3,
          .arr = {
              {
                  .name = "gps",
                  .values = {
                      .kind = horusInt_PRESENT,
                      .u = {
                          .horusInt = {
                              .nCount = 4,
                              .arr = {gpsHdop, gpsJamWarning, gpsResetCounter, currentGPSPowerMode}
                          }
                      }
                  },
                  .exist = { .name = true, .values = true }
              },
              {
                  .name = "stats",
                  .values = {
                      .kind = horusInt_PRESENT,
                      .u = {
                          .horusInt = {
                              .nCount = 3,
                              .arr = {maxAlt, maxSpeed, burstDetected}
                          }
                      }
                  },
                  .exist = { .name = true, .values = true }
              },
              {
                  .name = "temps-heat",
                  .values = {
                      .kind = horusInt_PRESENT,
                      .u = {
                          .horusInt = {
                              .nCount = 4,
                              .arr = { (int)extHeaterPwmStatus, (int)referenceHeaterStatus, (int)readRadioTemp(), (int)rpm411InternalTemperature }
                          }
                      }
                  },
                  .exist = { .name = true, .values = true }
              }
          }
      },
      
      .velocityHorizontalKilometersPerHour = gpsSpeedKph,
      .gnssSatellitesVisible = gpsSats,
      .ascentRateCentimetersPerSecond = vVCalc * 100,
      .pressurehPa_x10 = (int)(pressureValue * 10),
      
      .temperatureCelsius_x10 = {
          .internal = readAvgIntTemp() * 10,
          .external = mainTemperatureValue * 10,
          .custom1 = extHeaterTemperatureValue * 10,
          .custom2 = readThermistorTemp() * 10,
          .exist = { .internal = true, .external = true, .custom1 = true, .custom2 = true }
      },
      
      .humidityPercentage = humidityValue,
      
      .milliVolts = {
          .battery = (int)(readBatteryVoltage() * 1000),
          .exist = { .battery = true, .solar = false, .custom1 = false, .custom2 = false }
      },
      
      .exist = {
          .extraSensors = true,
          .velocityHorizontalKilometersPerHour = true,
          .gnssSatellitesVisible = true,
          .ascentRateCentimetersPerSecond = true,
          .pressurehPa_x10 = true,
          .temperatureCelsius_x10 = true,
          .humidityPercentage = true,
          .milliVolts = true
      }
  };


    // The encoder needs a data structure for the serialization
    // Again - how much memory is allocated here?
    BitStream encodedMessage;

    // The Encoder may fail and update an error code
    int errCode;

    // Initialization associates the buffer to the bit stream
    // We want to write the uncoded message starting at 2 bytes into the message.

    BitStream_Init (&encodedMessage,
                    (unsigned char*)(uncoded_buffer+2),
                    HORUS_UNCODED_BUFFER_SIZE-1
    );
    // Originally this function call used a MUCH larger value for count
    //horusTelemetry_REQUIRED_BYTES_FOR_ENCODING);
    
    // Encode the message using uPER encoding rule

    // We patch in assert functionality in assert_override.h
    // Before running encode we set assert_value = 0
    // Then check the value in assert_value
    assert_value = 0;

    if (!horusTelemetry_Encode(&asnMessage,
                        &encodedMessage,
                        &errCode,
                        true) || assert_value != 0)
    {  
        // Not at this error helps that much in a flight, but it helps
        // us when debugging!   
        if (xdataPortMode == 1) {
          if(errCode > 0){
            xdataSerial.print("[error]: HORUS v3 Encoding Failed: ");
            xdataSerial.println(errCode);
          }
          if(assert_value != 0){
            xdataSerial.println("[error]: HORUS v3 Assert Failure, maybe hit buffer size limit");
          }
        }
        // Need to check what happens here.
        return 0;
    }
    else 
    {
        // Encoding was successful!
        // Now we need to figure out the required frame size, and add the CRC.
        int encodedSize = BitStream_GetLength(&encodedMessage);

        // Determine the required frame size.
        // Probably should do this from a list of valid sizes in a neater manner
        int frameSize = 128;
        if (encodedSize <= 30){
          frameSize = 32;
        } else if (encodedSize <= 46){
          frameSize = 48;
        } else if (encodedSize <= 62){
          frameSize = 64;
        } else if (encodedSize <= 94){
          frameSize = 96;
        } else if (encodedSize <= 126){
          frameSize = 128;
        }

        // Calculate CRC16 over the frame, starting at byte 2
        uint16_t packetCrc = (uint16_t)crc16((unsigned char *)(uncoded_buffer + 2),
                                     frameSize - 2);
        // Write CRC into bytes 0–1 of the packet
        memcpy(uncoded_buffer, &packetCrc, sizeof(packetCrc));  // little‑endian on STM32

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: HORUS v3 ASN1: ");
          xdataSerial.print(encodedSize);
          xdataSerial.print(" Frame: ");
          xdataSerial.println(frameSize);
        }

        return frameSize;
    }

    return 0;
}



int build_horus_binary_packet_v2(char* buffer) {
  // Generate a Horus Binary v2 packet, and populate it with data.
  // The assignments in this function should be replaced with real data
  horusPacketCount++;

  struct HorusBinaryPacketV2 BinaryPacketV2;

  BinaryPacketV2.PayloadID = horusPayloadId;  // 256 = 4FSKTEST-V2. Refer https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt | You can attempt to modify this according to Your needs
  BinaryPacketV2.Counter = horusPacketCount;
  BinaryPacketV2.Hours = gpsHours;
  BinaryPacketV2.Minutes = gpsMinutes;
  BinaryPacketV2.Seconds = gpsSeconds;
  BinaryPacketV2.Latitude = gpsLat;
  BinaryPacketV2.Longitude = gpsLong;
  BinaryPacketV2.Altitude = gpsAlt;
  BinaryPacketV2.Speed = gpsSpeedKph;
  BinaryPacketV2.BattVoltage = map(readBatteryVoltage() * 100, 0, 5 * 100, 0, 255);
  BinaryPacketV2.Sats = gpsSats;
  BinaryPacketV2.Temp = readAvgIntTemp();
  // Custom section. This is an example only, and the 9 bytes in this section can be used in other
  // ways. Refer here for details: https://github.com/projecthorus/horusdemodlib/wiki/5-Customising-a-Horus-Binary-v2-

  //default, matching rs41ng
  BinaryPacketV2.dummy1 = vVCalc * 100;               //-32768 - 32767 int16_t
  BinaryPacketV2.dummy2 = mainTemperatureValue * 10;  //-32768 - 32767 int16_t
  BinaryPacketV2.dummy3 = humidityValue;              //0 - 255 uint8_t
  BinaryPacketV2.dummy4 = pressureValue * 10;         //0 - 65535 uint16_t
  BinaryPacketV2.dummy5 = 0;


  BinaryPacketV2.Checksum = (uint16_t)crc16((unsigned char*)&BinaryPacketV2, sizeof(BinaryPacketV2) - 2);

  memcpy(buffer, &BinaryPacketV2, sizeof(BinaryPacketV2));

  return sizeof(struct HorusBinaryPacketV2);
}



//===== Function-only algorythms (for horus modem etc.)

unsigned int _crc_xmodem_update(unsigned int crc, uint8_t data) {
  crc ^= data << 8;
  for (int i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
    } else {
      crc <<= 1;
    }
  }
  return crc;
}

// CRC16 Calculation
unsigned int crc16(unsigned char* string, unsigned int len) {
  unsigned int crc = 0xFFFF;  // Initial seed
  for (unsigned int i = 0; i < len; i++) {
    crc = _crc_xmodem_update(crc, string[i]);
  }
  return crc;
}

uint16_t rttyCrc16Checksum(unsigned char* string, unsigned int len) {
  uint16_t crc = 0xffff;
  char i;
  unsigned int j = 0;
  while (j < len) {
    //  while (*(string) != 0) {
    crc = crc ^ (*(string++) << 8);
    for (i = 0; i < 8; i++) {
      if (crc & 0x8000)
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      else
        crc <<= 1;
    }
    j += 1;
  }
  return crc;
}

void PrintHex(char* data, uint8_t length, char* tmp) {
  // Print char data as hex
  byte first;
  int j = 0;
  for (uint8_t i = 0; i < length; i++) {
    first = ((uint8_t)data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first;
    j++;

    first = ((uint8_t)data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first;
    j++;
  }
  tmp[length * 2] = 0;
}


//===== System operation handlers

void hardwarePowerShutdown() {
  radioDisableTx();

  if (xdataPortMode == 1) {
    xdataSerial.println("\n SHUTDOWN, bye!");
  }

  for (int i = 0; i < 5; i++) {
    redLed();
    delay(200);
    bothLedOff();
    delay(200);
  }

  delay(2000);
  digitalWrite(PSU_SHUTDOWN_PIN, HIGH);
}


void buttonHandlerSimplified() {  //no special effects compared to normal button handler
  if (analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN) && analogRead(VBAT_PIN) > 80) {
    if (buttonMode > 0) {
      hardwarePowerShutdown();
    }
  }
}

void buttonHandler() {
  if (analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN) && analogRead(VBAT_PIN) > 80) {  //if button pressed (button measurement pin higher than battery voltage) and the sonde is powered on with batteries (so the sonde won't go crazy when plugged for example into a programmer)
    if (buttonMode == 0) {

    } else if (buttonMode == 1) {
      hardwarePowerShutdown();
    } else if (buttonMode == 2) {
      while (btnCounter < 3 && analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN)) {
        greenLed();
        delay(400);
        redLed();
        delay(400);
        bothLedOff();
        if (xdataPortMode == 1) {
          xdataSerial.print("*");
        }
        btnCounter++;
      }


      if (btnCounter == 1) {
        //empty, except canceling some functions:

        if (improvedGpsPerformance && gpsSats < 4 && gpsOperationMode != 0 && !cancelGpsImprovement) {  //disable improvedGpsPerformance wait
          cancelGpsImprovement = true;
          redLed();
          delay(200);
          bothLedOff();
        }

      } else if (btnCounter == 2) {
        if (radioEnablePA == true) {
          radioEnablePA = false;

          for (int i = 0; i < btnCounter; i++) {
            redLed();
            delay(50);
            bothLedOff();
            delay(50);
          }

          if (xdataPortMode == 1) {
            xdataSerial.println("Radio PA disabled");
          }

        } else {
          radioEnablePA = true;

          for (int i = 0; i < btnCounter; i++) {
            greenLed();
            delay(50);
            bothLedOff();
            delay(50);
          }

          if (xdataPortMode == 1) {
            xdataSerial.println("Radio PA enabled");
          }
        }

      } else if (btnCounter == 3) {
        hardwarePowerShutdown();
      }

      btnCounter = 0;
    }
  }
}


void deviceStatusHandler() {
  // Temporary flags for each check
  vBatWarn = false;
  gpsFixWarn = false;

  err = false;
  warn = false;
  ok = true;  // Default to ok until proven otherwise

  // Evaluate battery voltage
  float vBat = readBatteryVoltage();
  if (vBat < vBatWarnValue) {
    vBatWarn = true;
  }

  // Evaluate GPS status
  if (gpsSats < gpsSatsWarnValue) {
    if (gpsOperationMode == 0) {
      gpsFixWarn = false;

    } else {
      gpsFixWarn = true;

      if (xdataPortMode == 4 && !improvedGpsPerformance) {
        xdataSerial.println("stage: 50");
      }
    }
  } else {
    if (xdataPortMode == 4) {
      xdataSerial.println("stage: 59");
    }
  }

  // Combine the results to determine the final state
  if (sensorBoomFault || calibrationError || rpm411Error) {
    err = true;
    ok = false;
  } else if (vBatWarn || gpsFixWarn) {
    warn = true;
    ok = false;
  } else {
    // Explicitly clear warning if no errors or warnings are active
    warn = false;
    ok = true;
  }

  // LED Handling
  if (ledStatusEnable) {
    if (gpsAlt > ledAutoDisableHeight) {  // disable leds after launch
      ledsEnable = false;
    } else {
      ledsEnable = true;
    }

    if (ledsEnable) {
      if (err) {
        redLed();
      } else if (warn) {
        orangeLed();
      } else if (ok) {
        greenLed();
      }
    }
  } else {
    bothLedOff();
  }
}


void serialStatusHandler() {
  if (xdataPortMode == 1) {
    if (vBatWarn) {
      xdataSerial.println("[WARN]: vBatWarn - low voltage");
    }

    if (gpsFixWarn) {
      xdataSerial.println("[WARN]: gpsFix - gps isn't locked on position, give sky clearance, waiting for fix...");
    }

    if (ok) {
      xdataSerial.println("[ok]: Device working properly");
    }
  }
}


float readBatteryVoltage() {
  float batV;

  if (rsm4x4) {  //12bit adc
    batV = ((float)analogRead(VBAT_PIN) / 4095) * 3 * 2 * batVFactor;
  } else {  //10bit adc
    batV = ((float)analogRead(VBAT_PIN) / 1024) * 3 * 2 * batVFactor;
  }

  return batV;
}

float readThermistorTemp() {
  int adcValue = analogRead(REF_THERM);

  float voltage;

  // Convert ADC value to voltage
  if (rsm4x4) {
    voltage = adcValue * (3.0 / 4095);  // 3.0V reference, 12-bit ADC
  } else {
    voltage = adcValue * (3.0 / 1023);  //10bit adc
  }


  // Convert voltage to thermistor resistance
  float resistance = (3.0 / voltage - 1) * THERMISTOR_R25;

  // Convert resistance to temperature using the Beta equation
  // Note: The Beta equation is T = 1 / ( (1 / T0) + (1 / B) * ln(R/R0) ) - 273.15
  // Adjust for correct temperature calculation

  float temperatureK = 1.0 / (1.0 / (25 + 273.15) + (1.0 / THERMISTOR_B) * log(THERMISTOR_R25 / resistance));
  float temperatureC = temperatureK - 273.15;

  return temperatureC;
}

float readRadioTemp() {  //slightly modified script from rs41ng
  // Configure ADC for temperature sensor and internal reference
  writeRegister(0x0F, 0b00000000);  // ADCSEL = 0 (temperature sensor), ADCREF = 0 (internal ref)
  writeRegister(0x12, 0b00100000);  // TSRANGE = -64°C to +64°C, slope 8mV/°C, offset enabled

  // Trigger ADC reading
  writeRegister(0x0F, 0b10000000);  // ADCSTART = 1

  // Read the raw ADC value (wait for conversion if needed)
  uint8_t raw_value = readRegister(0x11);

  // Convert raw ADC value to temperature in degrees Celsius
  float temperature = -64.0f + (raw_value * 0.5f);

  return temperature;  // Temperature in degrees Celsius
}

bool ackWait(uint16_t timeoutMs = 100)
{
  uint8_t buf[10];
  uint8_t idx = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (gpsSerial.available()) {
      uint8_t b = gpsSerial.read();

      // Sync char 1
      if (idx == 0 && b != 0xB5) continue;

      // Sync char 2
      if (idx == 1 && b != 0x62) {
        idx = 0;
        continue;
      }

      buf[idx++] = b;

      // ACK/NAK packets are always 10 bytes
      if (idx == 10) {
        // UBX-ACK class?
        if (buf[2] == 0x05) {
          if (buf[3] == 0x01) return true;   // ACK-ACK
          if (buf[3] == 0x00) return false;  // ACK-NAK
        }
        idx = 0; // restart scan
      }
    }
  }
  return false; // timeout treated as failure
}

void sendUblox(int Size, uint8_t* Buffer) {
  gpsSerial.write(Buffer, Size);  // Arduino style byte send

  if(rsm4x4) {
    ackWait();
  }
}


void GPSManagement() {
  if(gpsOperationMode == 0) { // GPS disabled
    shutdownGPS();
  }
  else if(gpsOperationMode == 1) {
    startGPS();

    if (currentGPSPowerMode != 1) {
      if(rsm4x4) {
#ifdef RSM4x4
        sendUblox(sizeof(ubxCfgValSetContinuous), ubxCfgValSetContinuous);
        sendUblox(sizeof(ubxEnableGps), ubxEnableGps);
        sendUblox(sizeof(ubxEnableGlo), ubxEnableGlo);
        sendUblox(sizeof(ubxEnableGal), ubxEnableGal);
        sendUblox(sizeof(ubxEnableBds), ubxEnableBds);
#endif
      }
      else if(rsm4x2) {
        sendUblox(sizeof(ubxCfgNav5_maxPerformance), ubxCfgNav5_maxPerformance);
      }
      currentGPSPowerMode = 1;
    }
  }
  else if (gpsOperationMode == 2) {
    startGPS();

    if(rsm4x2) {
      if(gpsSats < 7 && currentGPSPowerMode != 2) {
        sendUblox(sizeof(ubxCfgNav5_powerSave), ubxCfgNav5_powerSave);
        currentGPSPowerMode = 1;
      }
      else if(gpsSats >= 7 && currentGPSPowerMode != 1) {
        sendUblox(sizeof(ubxCfgNav5_maxPerformance), ubxCfgNav5_maxPerformance);
        currentGPSPowerMode = 2;
      }
    }
    else if(rsm4x4) {
#ifdef RSM4x4
      gpsOperationMode = 3;
#endif
    }
  }
  else if (gpsOperationMode == 3) {
    startGPS();

    if(rsm4x4) {
#ifdef RSM4x4
      if(gpsSats <= 10 && currentM10IntelligentMode != 1) {
        if(m10ConstellationOptimization && !m10AggressiveOpt) {
          sendUblox(sizeof(ubxEnableGps), ubxEnableGps);
          sendUblox(sizeof(ubxEnableGlo), ubxEnableGlo);
          sendUblox(sizeof(ubxEnableGal), ubxEnableGal);
          sendUblox(sizeof(ubxEnableBds), ubxEnableBds);
        }
        else if(m10ConstellationOptimization && m10AggressiveOpt) {
          sendUblox(sizeof(ubxEnableGps), ubxEnableGps);
          sendUblox(sizeof(ubxDisableGlo), ubxDisableGlo);
          sendUblox(sizeof(ubxEnableGal), ubxEnableGal);
          sendUblox(sizeof(ubxEnableBds), ubxEnableBds);
        }
        sendUblox(sizeof(ubxCfgValSetContinuous), ubxCfgValSetContinuous);
        currentM10IntelligentMode = 1;
      }
      else if(gpsSats <= 15 && gpsSats > 10 && currentM10IntelligentMode != 2) {
        if(m10ConstellationOptimization && !m10AggressiveOpt) {
          sendUblox(sizeof(ubxEnableGps), ubxEnableGps);
          sendUblox(sizeof(ubxEnableGlo), ubxEnableGlo);
          sendUblox(sizeof(ubxEnableGal), ubxEnableGal);
          sendUblox(sizeof(ubxEnableBds), ubxEnableBds);
        }
        else if (m10ConstellationOptimization && m10AggressiveOpt) {
          sendUblox(sizeof(ubxEnableGps), ubxEnableGps);
          sendUblox(sizeof(ubxDisableGlo), ubxDisableGlo);
          sendUblox(sizeof(ubxDisableGal), ubxDisableGal);
          sendUblox(sizeof(ubxEnableBds), ubxEnableBds);
        }

        if(m10CyclicTracking && !m10AggressiveOpt) {
          sendUblox(sizeof(ubxCfgValSetContinuous), ubxCfgValSetContinuous);
        }
        else if(m10CyclicTracking && m10AggressiveOpt) {
          sendUblox(sizeof(ubxCfgValSetPsmct), ubxCfgValSetPsmct);
          sendUblox(sizeof(ubxCfgValSetPsmctPeriod10), ubxCfgValSetPsmctPeriod10);
        }
        currentM10IntelligentMode = 2;
      }
      else if(gpsSats >= 15 && currentM10IntelligentMode != 3) {
        if(m10ConstellationOptimization && !m10AggressiveOpt) {
          sendUblox(sizeof(ubxEnableGps), ubxEnableGps);
          sendUblox(sizeof(ubxDisableGlo), ubxDisableGlo);
          sendUblox(sizeof(ubxEnableGal), ubxEnableGal);
          sendUblox(sizeof(ubxEnableBds), ubxEnableBds);
        }
        else if (m10ConstellationOptimization && m10AggressiveOpt) {
          sendUblox(sizeof(ubxEnableGps), ubxEnableGps);
          sendUblox(sizeof(ubxDisableGlo), ubxDisableGlo);
          sendUblox(sizeof(ubxDisableGal), ubxDisableGal);
          sendUblox(sizeof(ubxEnableBds), ubxEnableBds);
        } 

        if(m10CyclicTracking && !m10AggressiveOpt) {
          sendUblox(sizeof(ubxCfgValSetPsmct), ubxCfgValSetPsmct);
          sendUblox(sizeof(ubxCfgValSetPsmctPeriod10), ubxCfgValSetPsmctPeriod10);
        }
        else if(m10CyclicTracking && m10AggressiveOpt) {
          sendUblox(sizeof(ubxCfgValSetPsmct), ubxCfgValSetPsmct);
          sendUblox(sizeof(ubxCfgValSetPsmctPeriod10), ubxCfgValSetPsmctPeriod10);
        }
        currentM10IntelligentMode = 3;
      }
#endif
    }
    else if (rsm4x2) {
      gpsOperationMode = 2;
    }
  }
  else {
    gpsOperationMode = 1;
  }
}

void gpsHandler() {

  GPSManagement();

  if(rsm4x4) {
    gpsStatus = currentM10IntelligentMode;
  }
  else {
    gpsStatus = currentGPSPowerMode;
  }

  uint16_t gpsNmeaMsgWaitTime = 1200;

  if(m10PerformanceImprovements && rsm4x4) {
    gpsNmeaMsgWaitTime = 1000;
  }
  else {
    gpsNmeaMsgWaitTime = 1200;
  }

  if (gpsOperationMode != 0) {  //if gps disabled then don't unnecesarly try to read it
    unsigned long start = millis();
    do {
      while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
        
        if(xdataPortMode == 5) {
          xdataSerial.print(c);
        }
      }
        
    } while (millis() - start < gpsNmeaMsgWaitTime);


    gpsTime = gps.time.value() / 100;
    gpsHours = gps.time.hour();
    gpsMinutes = gps.time.minute();
    gpsSeconds = gps.time.second();
    gpsLat = gps.location.lat();
    gpsLong = gps.location.lng();
    gpsAlt = gps.altitude.meters();
    gpsSpeed = gps.speed.mps();
    gpsSpeedKph = gps.speed.kmph();
    gpsSats = gps.satellites.value();
    gpsHdop = gps.hdop.hdop();

    verticalVelocityCalculationHandler();

    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: GPS data obtained and updated");
    }


    if (gpsTimeoutWatchdog > 0) {
      // Check if GPS timer counter is inactive and no fix
      if (!gpsTimeoutCounterActive && gpsSats < 3) {
        gpsTimerBegin = millis();        // Start the timer
        gpsTimeoutCounterActive = true;  // Mark timer as active
      }

      // Check if timeout has occurred
      if (gpsTimeoutCounterActive && millis() - gpsTimerBegin > gpsTimeoutWatchdog) {
        if (gpsSats < 3) {                  //If GPS still has no fix after timeout, restart it
          gpsTimeoutCounterActive = false;  // Reset the timer state
          gpsTimerBegin = 0;                // Clear the timer
          restartGPS();                     // Handle GPS timeout recovery
          delay(1000);
          initGPS();
        } else {                            // Reset all timer variables and don't do anything, cause the GPS works ok
          gpsTimeoutCounterActive = false;  // Reset the timer state
          gpsTimerBegin = 0;                // Clear the timer
        }
      }
    }


    if (beganFlying && (gpsHdop > 15 || abs(vVCalc) > 300)) {
      gpsJamWarning = true;

      if (xdataPortMode == 1) {
        xdataSerial.println("[WARN]: GPS Jam warning is active!");
      }
    } else {
      gpsJamWarning = false;
    }
  }
}

void shutdownGPS() {
  digitalWrite(GPS_RESET_PIN, LOW);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS shutdown");
  }
}

void startGPS() {
  digitalWrite(GPS_RESET_PIN, HIGH);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS is ON");
  }
}

void restartGPS() {
  digitalWrite(GPS_RESET_PIN, LOW);
  delay(3000);
  digitalWrite(GPS_RESET_PIN, HIGH);

  gpsResetCounter++;

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS restart has been issued");
  }
}

void initGPS() {
  if (xdataPortMode == 1) {
    xdataSerial.println(F("[info]: GPS settings are being initialized..."));
  }

  if (ubloxGpsAirborneMode) {
    if (xdataPortMode == 1) {
      xdataSerial.println(F("[info] Setting the Airborne 1G (6) GPS dynamic model..."));
    }

    if (rsm4x4) {
#ifdef RSM4x4
      sendUblox(sizeof(ubxCfgValSet_dynmodel6), ubxCfgValSet_dynmodel6);
      delay(1000);
      sendUblox(sizeof(ubxCfgValSet_dynmodel6), ubxCfgValSet_dynmodel6);
#endif
    } else if (rsm4x2) {
      // Assuming these are defined globally for RSM4x2
      sendUblox(sizeof(ubxCfgNav5_dynmodel6), ubxCfgNav5_dynmodel6);
      delay(1000);
      sendUblox(sizeof(ubxCfgNav5_dynmodel6), ubxCfgNav5_dynmodel6);
    }
  }

  if (xdataPortMode == 1) {
    xdataSerial.println(F("[info]: GPS settings done"));
  }

#ifdef RSM4x4
  if (rsm4x4 && m10PerformanceImprovements) {
    sendUblox(sizeof(ubxCfgValSet_msgRate4Hz), ubxCfgValSet_msgRate4Hz);
    delay(100);
    sendUblox(sizeof(ubxCfgValSet_navRate2500), ubxCfgValSet_navRate2500);
    delay(100);
    sendUblox(sizeof(ubxCfgValSet_enableGns), ubxCfgValSet_enableGns);
    delay(100);
    sendUblox(sizeof(ubxCfgValSet_disableGga), ubxCfgValSet_disableGga);
    delay(100);
    sendUblox(sizeof(ubxCfgValSet_maxSvs64), ubxCfgValSet_maxSvs64);
    delay(100); 
    sendUblox(sizeof(ubxCfgElev3), ubxCfgElev3);
    delay(100);
    sendUblox(sizeof(ubxCfgSig10), ubxCfgSig10);
  }

  // Wrap constellation enables as they use the M10 ValSet arrays
  sendUblox(sizeof(ubxEnableGps), ubxEnableGps);
  delay(100);
  sendUblox(sizeof(ubxEnableGlo), ubxEnableGlo);
  delay(100);
  sendUblox(sizeof(ubxEnableGal), ubxEnableGal);
  delay(100);
  sendUblox(sizeof(ubxEnableBds), ubxEnableBds);

  if(rsm4x4 && m10SuperS) {
    delay(100);
    sendUblox(sizeof(ubxCfgValSet_enableSuperS), ubxCfgValSet_enableSuperS);
  }
#endif
}


// Function to select heater and change its state (on/off)
void selectReferencesHeater(int heatingMode) {
  referenceHeaterStatus = heatingMode;

  if (xdataPortMode == 1) {
    xdataSerial.print("[info]: References heater power has been set to ");
    xdataSerial.println(heatingMode);
  }

  switch (heatingMode) {
    case 0:                            //all OFF
      digitalWrite(PULLUP_TM, HIGH);   //disable temperature ring oscillator power
      digitalWrite(PULLUP_HYG, HIGH);  //disable wxHumidity ring oscillator power
      if (rsm4x4) {
        digitalWrite(HEAT_REF, LOW);  //disable reference heater
      } else if (rsm4x2) {
        writeRegister(0x0C, 0x01);  //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
      }
      break;

    case 1:  //hyg reference heater on
      digitalWrite(PULLUP_TM, HIGH);
      digitalWrite(PULLUP_HYG, LOW);
      if (rsm4x4) {
        digitalWrite(HEAT_REF, HIGH);
      } else if (rsm4x2) {
        writeRegister(0x0C, 0x00);  //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
      }
      break;

    case 2:  //temp reference heater on
      digitalWrite(PULLUP_TM, LOW);
      digitalWrite(PULLUP_HYG, HIGH);
      if (rsm4x4) {
        digitalWrite(HEAT_REF, HIGH);
      } else if (rsm4x2) {
        writeRegister(0x0C, 0x00);  //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
      }
      break;

    case 3:  //all heaters on
      digitalWrite(PULLUP_TM, LOW);
      digitalWrite(PULLUP_HYG, LOW);
      if (rsm4x4) {
        digitalWrite(HEAT_REF, HIGH);
      } else if (rsm4x2) {
        writeRegister(0x0C, 0x00);  //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
      }
      break;

    default:
      break;
  }
}



/*
void refHeaterHandler() {
  unsigned long currentMillis = millis();
  float currentTemp = readThermistorTemp();

  // Mode 0: Heating Off
  if (refHeatingMode == 0) {
    disableRefHeaterRing();  // Ensure the heater is off

    isReferenceHeaterOn = false;
    heaterDebugState = 5;

    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: Heating is completely OFF.");
    }

    return;  // Exit early
  }

  // Mode 1: Auto Mode
  if (refHeatingMode == 1) {
    // Disable heater if temperature exceeds the critical disable threshold
    if (currentTemp > refHeaterCriticalDisableTemp) {
      disableRefHeaterRing();  // Turn off the heater

      if (xdataPortMode == 1) {
        xdataSerial.print("[ERR]: Heater has been turned OFF because temp exceeds threshold of (*C): ");
        xdataSerial.println(refHeaterCriticalDisableTemp);
      }

      heaterOffTime = currentMillis;  // Record the time the heater was turned off

      isReferenceHeaterOn = false;         // Update the heater state
      isHeaterPausedOvht = true;  // Mark that the heater was paused due to overheating
      heaterDebugState = 19;

      return;  // Exit early
    }

    // Re-enable the heater if it was paused due to overheating and the temperature drops below the re-enable threshold
    if (isHeaterPausedOvht && currentTemp <= refHeaterCriticalReenableTemp) {
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Temperature dropped below re-enable threshold of (*C): ");
        xdataSerial.println(refHeaterCriticalReenableTemp);
      }

      isHeaterPausedOvht = false;  // Clear the overheating flag

      enableRefHeaterRing();  // Turn on the heater

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HEATER RE-ENABLED!");
      }

      isReferenceHeaterOn = true;  // Update the heater state
      heaterDebugState = 11;

      return;  // Exit early after re-enabling
    }

    // Disable heater if the timer condition is met
    if (isReferenceHeaterOn && currentMillis - heaterOnTime >= heaterWorkingTimeSec * 1000) {
      disableRefHeaterRing();  // Turn off the heater
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Heater OFF due to timer, cooldown for (seconds): ");
        xdataSerial.println(heaterCooldownTimeSec);
      }

      heaterOffTime = currentMillis;  // Record the time the heater was turned off
      isReferenceHeaterOn = false;             // Update the heater state
      heaterDebugState = 10;
    } else if (isReferenceHeaterOn) {
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Heater is currently ON, at (*C): ");
        xdataSerial.println(currentTemp);
      }

      heaterDebugState = 11;
    } else {
      // If the heater is off, check if cooldown has passed and if it should be re-enabled
      if (currentMillis - heaterOffTime >= heaterCooldownTimeSec * 1000) {
        if (currentTemp < autoHeaterThreshold) {
          if (xdataPortMode == 1) {
            xdataSerial.print("[info]: Cooldown time passed, thermistor temp is under threshold of (*C): ");
            xdataSerial.println(autoHeaterThreshold);
            xdataSerial.println("[info]: HEATER ON!");
          }

          enableRefHeaterRing();  // Turn on the heater

          heaterOnTime = currentMillis;  // Record the time the heater was turned on
          isReferenceHeaterOn = true;             // Update the heater state
          heaterDebugState = 11;
        } else {
          if (xdataPortMode == 1) {
            xdataSerial.println("[info]: autoRefHeating is enabled, but the temperature is higher than the threshold.");
          }

          heaterDebugState = 10;
        }
      }
    }
    return;  // Exit early
  }

  // Mode 2: Heater Always On with Safety Control
  if (refHeatingMode == 2) {
    // Check if the temperature exceeds the critical disable threshold
    if (currentTemp > refHeaterCriticalDisableTemp) {
      disableRefHeaterRing();  // Turn off the heater

      if (xdataPortMode == 1) {
        xdataSerial.print("[ERR]: Heater has been turned OFF because temp exceeds threshold of (*C): ");
        xdataSerial.println(refHeaterCriticalDisableTemp);
      }

      heaterOffTime = currentMillis;  // Record the time the heater was turned off
      isReferenceHeaterOn = false;             // Update the heater state
      isHeaterPausedOvht = true;      // Mark that the heater was paused due to overheating
      heaterDebugState = 29;

      return;  // Exit early
    }

    // Re-enable the heater immediately after temperature drops below re-enable threshold
    if (isHeaterPausedOvht && currentTemp <= refHeaterCriticalReenableTemp) {
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Temperature dropped below re-enable threshold of (*C): ");
        xdataSerial.println(refHeaterCriticalReenableTemp);
      }

      isHeaterPausedOvht = false;  // Clear the overheating flag

      enableRefHeaterRing();  // Turn on the heater

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HEATER RE-ENABLED!");
      }

      isReferenceHeaterOn = true;             // Update the heater state
      heaterOnTime = currentMillis;  // Reset the heater's working time
      heaterDebugState = 21;
    } else if (!isReferenceHeaterOn) {
      // If the heater is not on, ensure it is turned on
      enableRefHeaterRing();  // Turn on the heater

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Heating is always ON with safety control.");
      }

      isReferenceHeaterOn = true;             // Update the heater state
      heaterOnTime = currentMillis;  // Reset the heater's working time
      heaterDebugState = 21;
    }
    return;  // Exit early
  }
}

void refHeaterHeightActivator() {
  if (refHeaterAutoActivationHeight != 0) {
    if (gpsAlt > refHeaterAutoActivationHeight) {
      refHeatingMode = 2;
    } else if (gpsAlt < refHeaterAutoActivationHeight) {
      refHeatingMode = 1;
    } else {
      refHeatingMode = 1;
    }
  }
}
*/


void powerHandler() {
  if (readBatteryVoltage() < batteryCutOffVoltage && batteryCutOffVoltage != 0) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[ERR] BATTERY CUT-OFF VOLTAGE, SYSTEM WILL POWER OFF");
    }

    radioDisableTx();

    if (xdataPortMode == 1) {
      xdataSerial.println("\n PSU_SHUTDOWN_PIN set HIGH, bye!");
    }

    hardwarePowerShutdown();
  }
}

/* TO BE CONTINUED... will implement better in the future
void xdataInstrumentHandler() {
  unsigned long start = millis();
  String serialData = "";

  // Read data until timeout
  while (millis() - start < oif411MsgWaitTime) {
    while (xdataSerial.available()) {
      serialData += (char)xdataSerial.read();
    }
  }

  // Process each frame separately
  int startIndex = 0;
  while ((startIndex = serialData.indexOf("xdata=", startIndex)) != -1) {
    int endIndex = serialData.indexOf("xdata=", startIndex + 1);
    if (endIndex == -1) {
      endIndex = serialData.length();
    }
    String frame = serialData.substring(startIndex, endIndex);
    decodeXdataOif411(frame);
    startIndex = endIndex;
  }
}


void decodeXdataOif411(String xdataString) {
  // Check if the input string starts with "xdata="
  if (xdataString.startsWith("xdata=")) {
    // Extract the relevant substring containing the data
    String data = xdataString.substring(6);  // Remove "xdata=" from the input

    // Extract and decode instrument type
    xdataInstrumentType = strtol(data.substring(0, 2).c_str(), NULL, 16);

    // Extract and decode instrument number
    xdataInstrumentNumber = strtol(data.substring(2, 4).c_str(), NULL, 16);

    // Extract and decode ozone pump temperature
    int rawPumpTemp = strtol(data.substring(4, 8).c_str(), NULL, 16);
    xdataOzonePumpTemperature = rawPumpTemp / 100.0;  // Convert to Celsius

    // Extract and decode ozone current
    int rawOzoneCurrent = strtol(data.substring(8, 13).c_str(), NULL, 16);
    xdataOzoneCurrent = rawOzoneCurrent / 10000;  //  /10000

    // Extract and decode ozone battery voltage
    int rawwxVoltage = strtol(data.substring(13, 15).c_str(), NULL, 16);
    xdataOzonewxVoltage = rawwxVoltage / 10.0;  // Convert to Volts (V)

    // Extract and decode ozone pump current
    xdataOzonePumpCurrent = strtol(data.substring(15, 18).c_str(), NULL, 16);
  }
}*/


// Function to select reading of a sensor and set its state (on/off)
void selectSensorBoom(int sensorNum, int state) {
  // Ensure state is either 0 (off) or 1 (on), return if invalid
  if (state != 0 && state != 1) {
    return;  // Invalid state, do nothing
  }

  // Set powerState based on the valid state
  int powerState = (state == 1) ? HIGH : LOW;

  switch (sensorNum) {
    case 0:                             // All sensors
      digitalWrite(SPST1, powerState);  // Reference 1
      digitalWrite(SPST2, powerState);  // Reference 2
      digitalWrite(SPST3, powerState);  // External Heater Temp
      digitalWrite(SPST4, powerState);  // Main
      digitalWrite(SPDT1, powerState);
      digitalWrite(SPDT2, powerState);
      digitalWrite(SPDT3, powerState);
      digitalWrite(PULLUP_TM, powerState);  // Common PULLUP for all
      digitalWrite(PULLUP_HYG, powerState);
      break;

    case 1:  // Reference 1 92khz (SPST1 and PULLUP_TM)
      digitalWrite(SPST1, powerState);
      digitalWrite(PULLUP_TM, powerState);
      digitalWrite(PULLUP_HYG, !powerState);
      break;

    case 2:  // Reference 2 62khz (SPST2 and PULLUP_TM)
      digitalWrite(SPST2, powerState);
      digitalWrite(PULLUP_TM, powerState);
      digitalWrite(PULLUP_HYG, !powerState);
      break;

    case 3:  // External Heater Temperature (SPST3 and PULLUP_TM)
      digitalWrite(SPST3, powerState);
      digitalWrite(PULLUP_TM, powerState);
      digitalWrite(PULLUP_HYG, !powerState);
      break;

    case 4:  // Main Temperature (SPST4 and PULLUP_TM)
      digitalWrite(SPST4, powerState);
      digitalWrite(PULLUP_TM, powerState);
      digitalWrite(PULLUP_HYG, !powerState);
      break;

    case 5:
      digitalWrite(SPDT1, powerState);
      digitalWrite(PULLUP_TM, !powerState);
      digitalWrite(PULLUP_HYG, powerState);

    case 6:
      digitalWrite(SPDT2, powerState);
      digitalWrite(PULLUP_TM, !powerState);
      digitalWrite(PULLUP_HYG, powerState);

    case 7:
      digitalWrite(SPDT3, powerState);
      digitalWrite(PULLUP_TM, !powerState);
      digitalWrite(PULLUP_HYG, powerState);

    default:
      // Invalid sensor number, do nothing
      break;
  }
}


float getSensorBoomFreq(int sensorNum) {
  static uint32_t firstEdge, lastEdge, prevCapture, currentCapture;
  static uint64_t totalTicks;
  static uint32_t timFreq, timeoutCounter;
  static float freq;
  
  static uint32_t original_gpio_cfg;
  static uint32_t original_uart_cr1;

  const uint32_t targetSamples = 2400; // Samples count
  const uint32_t CYCLE_TIMEOUT = 1000000; 
  
  totalTicks = 0;
  freq = 0.0f;

  selectSensorBoom(sensorNum, 1);
  delay(18); 

  // --- 1. Clock Enable & GPIO Setup ---
#if defined(RSM4x4) // L412
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE1) | GPIO_MODER_MODE1_1; 
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xF << 4)) | (0x1 << 4); // AF1 = TIM2
#elif defined(RSM4x2) // F100
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  GPIOA->CRL = (GPIOA->CRL & ~(0xF << 4)) | (0x4 << 4); // Input Floating
#endif

  // --- 2. Timer Setup ---
  TIM2->CR1 = 0;
  TIM2->PSC = 0; 
  TIM2->ARR = 0xFFFFFFFF; // L4 is 32-bit native, F1 uses 16-bit
  TIM2->CCMR1 = TIM_CCMR1_CC2S_0 | (0x4 << 12); 
  TIM2->CCER = TIM_CCER_CC2E; 
  TIM2->SR = 0;                
  TIM2->EGR = TIM_EGR_UG;      
  TIM2->CR1 |= TIM_CR1_CEN;    

  // --- 3. TOTAL SYSTEM LOCKDOWN () ---
  // GPS UART seems to cause jitter
#if defined(RSM4x2)
  original_gpio_cfg = GPIOB->CRH;
  GPIOB->CRH &= ~(0xF << 12); // Disconnect PB11 RX
  original_uart_cr1 = USART3->CR1;
  USART3->CR1 &= ~USART_CR1_UE; 
#endif

  __disable_irq();

  // Wait for FIRST edge
  TIM2->SR = ~TIM_SR_CC2IF;
  timeoutCounter = 0;
  while (!(TIM2->SR & TIM_SR_CC2IF)) {
    if (++timeoutCounter > CYCLE_TIMEOUT) goto capture_error;
  }
  firstEdge = TIM2->CCR2;
  prevCapture = firstEdge;

  // 4. Capture loop
  for (uint32_t i = 0; i < targetSamples; i++) {
    timeoutCounter = 0;
    while (!(TIM2->SR & TIM_SR_CC2IF)) {
      if (++timeoutCounter > CYCLE_TIMEOUT) goto capture_error;
    }
    
    currentCapture = TIM2->CCR2;
    TIM2->SR = ~TIM_SR_CC2IF; 
    
#if defined(RSM4x2)
    totalTicks += (uint16_t)(currentCapture - prevCapture); // Accumulate 16-bit
#else
    // On L412 (32-bit), we can just subtract at the end, 
    // but accumulation is safer for extremely long samples.
    totalTicks += (uint32_t)(currentCapture - prevCapture); 
#endif
    prevCapture = currentCapture;
  }
  lastEdge = currentCapture;

  __enable_irq();
  goto cleanup;

capture_error: 
  __enable_irq();
  totalTicks = 0; 

cleanup:
#if defined(RSM4x2)
  USART3->CR1 = original_uart_cr1;
  GPIOB->CRH = original_gpio_cfg; 
#endif

  TIM2->CR1 &= ~TIM_CR1_CEN;
  selectSensorBoom(sensorNum, 0);

  // --- 5. Final Calculation ---
  if (totalTicks > 0) {
    timFreq = HAL_RCC_GetPCLK1Freq();
    // Timer clock is 2x PCLK if APB1 divider > 1
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) timFreq *= 2;
    
    freq = (float)((double)timFreq * (double)targetSamples / (double)totalTicks);
    
  } else {
    freq = 0.0f;
  }

  return freq;
}

// Function to calibrate the sensor using the two calibration resistors
float calibrateTempSensorBoom() {
  // Get the frequencies for calibration resistors
  float freq750 = getSensorBoomFreq(1);  // Get frequency for 750Ω resistor
  selectSensorBoom(0, 0);
  float freq1100 = getSensorBoomFreq(2);  // Get frequency for 1100Ω resistor
  selectSensorBoom(0, 0);

  // Calibration constant k: R * f (frequency-to-resistance ratio)
  // We use an average to balance between the two calibration resistors.
  float k = (750.0 * freq750 + 1100.0 * freq1100) / 2.0;

  return k;  // Return the calibration constant
}

float calculateSensorBoomResistance(float freq, float k) {
  return k / freq;
}

// Function to convert PT1000 resistance to temperature in Celsius
float convertPt1000ResToTemp(float resistance) {
  const float R0 = 1000.0;      // Resistance at 0°C
  const float alpha = 0.00385;  // Temperature coefficient of resistance

  // Calculate temperature using the formula
  float temperature = (resistance * 0.945 - R0) / (R0 * alpha); // Vaisala uses sensors that have a slight offset from original PT1000 sensors, thats why *0.945 - thanks for this observation Petya!

  return temperature;
}

void sensorBoomHandler() {
  double tempCorrectionFactor = 1.0;

  if (sensorBoomEnable) {

    int lastReferenceHeaterStatus = referenceHeaterStatus;
    selectReferencesHeater(0);

    // Calibrate the sensor and get the calibration factor
    tempSensorBoomCalibrationFactor = calibrateTempSensorBoom();

    // Get main temperature frequency
    mainTemperatureFrequency = getSensorBoomFreq(4);
    if (mainTemperatureFrequency <= 0) {
      sensorBoomMainTempError = true;  // Error if frequency is invalid

      if (xdataPortMode == 1) {
        xdataSerial.println("[WARN]: Main temperature hook sensor boom error! MEAS frequency invalid. The hook may have snapped.");
      }

    } else {
      sensorBoomMainTempError = false;  // No error
      mainTemperatureResistance = calculateSensorBoomResistance(mainTemperatureFrequency, tempSensorBoomCalibrationFactor);
      mainTemperatureValue = convertPt1000ResToTemp(mainTemperatureResistance) + mainTemperatureCorrectionC;

    }

    selectSensorBoom(0, 0);

    // Get external heater temperature frequency
    extHeaterTemperatureFrequency = getSensorBoomFreq(3);
    if (extHeaterTemperatureFrequency <= 0) {
      sensorBoomHumidityModuleError = true;  // Error if frequency is invalid

      if (xdataPortMode == 1) {
        xdataSerial.println("[WARN]: External humidity heater temperature sensor error! MEAS frequency invalid.");
      }

    } else {
      sensorBoomHumidityModuleError = false;  // No error
      extHeaterTemperatureResistance = calculateSensorBoomResistance(extHeaterTemperatureFrequency, tempSensorBoomCalibrationFactor);
      extHeaterTemperatureValue = convertPt1000ResToTemp(extHeaterTemperatureResistance) + extHeaterTemperatureCorrectionC;
    }

    selectSensorBoom(0, 0);



  humidityFrequency = getSensorBoomFreq(5);
  refCapHighFrequency = getSensorBoomFreq(6); // 47pF Ref (Lower Frequency)
  refCapLowFrequency = getSensorBoomFreq(7);  // 0pF Ref (Higher Frequency)

  // 1. Calculate raw capacitance
  humidityCapacitance = 47.0 * (float)(refCapLowFrequency - humidityFrequency) / (refCapLowFrequency - refCapHighFrequency);

  // 2. Initial % calculation
  humidityValue = ((humidityCapacitance - zeroHumidityCapacitance) / (maxHumidityCapacitance - zeroHumidityCapacitance)) * 100.0;

  // FIX: Clamp rawRH here before the complex math transformations
  // This prevents the exponential and polynomial corrections from exploding
  if (humidityValue > 115.0) humidityValue = 115.0; 
  if (humidityValue < 0) humidityValue = 0;

  // Module heating RH correction
  if (extHeaterPwmStatus > 0) {
    humidityValue *= expf(0.045f * (extHeaterTemperatureValue - mainTemperatureValue));
  }
  
  // Low-end capacitive sensor correction
  humidityValue += (100 - humidityValue) * humidityValue * 75 / 10000;

  // Base reduction at temperatures < 0°C
  humidityValue += (0 - extHeaterTemperatureValue) / 11.1;

  // Very low temperature corrections
  if (extHeaterTemperatureValue < -20) {
      humidityValue = humidityValue * (100 + (-20 - extHeaterTemperatureValue)) / 140;
  }
  if (extHeaterTemperatureValue < -40) {
      humidityValue = humidityValue * (150 + (-40 - extHeaterTemperatureValue)) / 280;
  }

  humidityValue = kalmanFilter(humidityValue, humidityKalmanEst, humidityKalmanErrorEst, humidityKalmanError, humidityKalmanQ); 

  if (humidityValue < 0.0) {
    humidityValue = 0.0;
  }
  else if (humidityValue > 100.0 && humidityValue <= 105.0) {
    humidityValue = 100.0;
  }
  else if (humidityValue > 105.0 && humidityValue <= 110) {
    humidityValue -= 5.0;
  }
  else if(humidityValue > 110.0) {
    humidityValue = 110;
  }



    // Check overall sensor status
    sensorBoomFault = sensorBoomMainTempError || sensorBoomHumidityModuleError;

    if (sensorBoomMainTempError && sensorBoomHumidityModuleError) {
      if (xdataPortMode == 1) {
        xdataSerial.println("[ERR]: The sensor boom seems disconnected!");
      }
    }

    selectSensorBoom(0, 0);
    selectReferencesHeater(lastReferenceHeaterStatus);
  }
  else {
    selectSensorBoom(0, 0);
  }
}



void verticalVelocityCalculationHandler() {
  if (gpsSats > 3) {  //calculate only if fix, else set to 0 to indicate fault
    // Time difference in seconds
    float timeDifference = (millis() - lastGpsAltMillisTime) / 1000.0;

    // Ensure timeDifference is not zero to avoid division by zero
    if (timeDifference > 0) {
      // Vertical velocity Vv = (altNow - altBefore) / (timeNow - timeBefore)
      vVCalc = (gpsAlt - lastGpsAlt) / timeDifference;
    }

    // Update last known values
    lastGpsAltMillisTime = millis();
    lastGpsAlt = gpsAlt;
  } else {
    vVCalc = 0;
  }
}

// Function to send the space tone (2200 Hz)
void aprsSendSpace() {
  writeRegister(0x73, 0x0A);
  delayMicroseconds(aprsSpaceTime);  // Half cycle of space (2200Hz)
  writeRegister(0x73, 0x00);
  delayMicroseconds(aprsSpaceTime);  // Half cycle of silence

  // Repeat to match 833 microseconds for 1 full bit duration at 1200 baud
  writeRegister(0x73, 0x0A);
  delayMicroseconds(aprsSpaceTime);  // Half cycle of space (2200Hz)
  writeRegister(0x73, 0x00);
  delayMicroseconds(aprsSpaceTime);  // Half cycle of silence
}

// Function to send the mark tone (1200 Hz)
void aprsSendMark() {
  writeRegister(0x73, 0x0A);
  delayMicroseconds(aprsMarkTime);  // Half cycle of mark (1200Hz)
  writeRegister(0x73, 0x00);
  delayMicroseconds(aprsMarkTime);  // Half cycle of silence
}

void aprsSetTone(bool currentTone) {
  if (currentTone)
    aprsSendMark();
  else
    aprsSendSpace();
}

/*
 * This function will calculate CRC-16 CCITT for the FCS (Frame Check Sequence)
 * as required for the HDLC frame validity check.
 * 
 * Using 0x1021 as polynomial generator. The CRC registers are initialized with
 * 0xFFFF
 */
void calcAprsCrc(bool in_bit) {
  unsigned short xor_in;

  xor_in = aprsCrc ^ in_bit;
  aprsCrc >>= 1;

  if (xor_in & 0x01)
    aprsCrc ^= 0x8408;
}

void sendAprsCrc(void) {
  unsigned char crc_lo = aprsCrc ^ 0xff;
  unsigned char crc_hi = (aprsCrc >> 8) ^ 0xff;

  sendAprsChar_NRZI(crc_lo, HIGH);
  sendAprsChar_NRZI(crc_hi, HIGH);
}

void sendAprsHeader(void) {
  char temp;

  /*
   * APRS AX.25 Header 
   * ........................................................
   * |   DEST   |  SOURCE  |   DIGI   | CTRL FLD |    PID   |
   * --------------------------------------------------------
   * |  7 bytes |  7 bytes |  7 bytes |   0x03   |   0xf0   |
   * --------------------------------------------------------
   * 
   * DEST   : 6 byte "callsign" + 1 byte ssid
   * SOURCE : 6 byte Your callsign + 1 byte ssid
   * DIGI   : 6 byte "digi callsign" + 1 byte ssid
   * 
   * ALL DEST, SOURCE, & DIGI are left shifted 1 bit, ASCII format.
   * DIGI ssid is left shifted 1 bit + 1
   * 
   * CTRL FLD is 0x03 and not shifted.
   * PID is 0xf0 and not shifted.
   */

  /********* DEST ***********/
  temp = strlen(aprsDest);
  for (int j = 0; j < temp; j++)
    sendAprsChar_NRZI(aprsDest[j] << 1, HIGH);
  if (temp < 6) {
    for (int j = 0; j < (6 - temp); j++)
      sendAprsChar_NRZI(' ' << 1, HIGH);
  }
  sendAprsChar_NRZI('0' << 1, HIGH);


  /********* SOURCE *********/
  temp = strlen(aprsCall);
  for (int j = 0; j < temp; j++)
    sendAprsChar_NRZI(aprsCall[j] << 1, HIGH);
  if (temp < 6) {
    for (int j = 0; j < (6 - temp); j++)
      sendAprsChar_NRZI(' ' << 1, HIGH);
  }
  sendAprsChar_NRZI((aprsSsid + '0') << 1, HIGH);


  /********* DIGI ***********/
  temp = strlen(aprsDigi);
  for (int j = 0; j < temp; j++)
    sendAprsChar_NRZI(aprsDigi[j] << 1, HIGH);
  if (temp < 6) {
    for (int j = 0; j < (6 - temp); j++)
      sendAprsChar_NRZI(' ' << 1, HIGH);
  }
  sendAprsChar_NRZI(((aprsDigiSsid + '0') << 1) + 1, HIGH);

  /***** CTRL FLD & PID *****/
  sendAprsChar_NRZI(0x03, HIGH);
  sendAprsChar_NRZI(0xf0, HIGH);
}

void sendAprsPayload(char type) {
  /*
   * APRS AX.25 Payloads
   * 
   * TYPE : POSITION
   * ........................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|
   * --------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |
   * --------------------------------------------------------
   * 
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * 
   * 
   * TYPE : STATUS
   * ..................................
   * |DATA TYPE |    STATUS TEXT      |
   * ----------------------------------
   * |  1 byte  |       N bytes       |
   * ----------------------------------
   * 
   * DATA TYPE  : >
   * STATUS TEXT: Free form text
   * 
   * 
   * TYPE : POSITION & STATUS
   * ..............................................................................
   * |DATA TYPE |    LAT   |SYMB. OVL.|    LON   |SYMB. TBL.|    STATUS TEXT      |
   * ------------------------------------------------------------------------------
   * |  1 byte  |  8 bytes |  1 byte  |  9 bytes |  1 byte  |       N bytes       |
   * ------------------------------------------------------------------------------
   * 
   * DATA TYPE  : !
   * LAT        : ddmm.ssN or ddmm.ssS
   * LON        : dddmm.ssE or dddmm.ssW
   * STATUS TEXT: Free form text
   * 
   * 
   * All of the data are sent in the form of ASCII Text, not shifted.
   * 
   
  if(type == _FIXPOS)
  {
    sendAprsChar_NRZI('!', HIGH);
    sendAprsStringLen(lat, strlen(lat));
    sendAprsChar_NRZI(aprsSymbolOverlay, HIGH);
    sendAprsStringLen(lon, strlen(lon));
    sendAprsChar_NRZI(aprsSymTable, HIGH);
  }
  else if(type == _STATUS)
  {
    sendAprsChar_NRZI('>', HIGH);
    sendAprsStringLen(statusMessage, strlen(statusMessage));
  }
  else if(type == _FIXPOS_STATUS)
  {
    sendAprsChar_NRZI('!', HIGH);
    sendAprsStringLen(lat, strlen(lat));
    sendAprsChar_NRZI(aprsSymbolOverlay, HIGH);
    sendAprsStringLen(lon, strlen(lon));
    sendAprsChar_NRZI(aprsSymTable, HIGH);

    sendAprsChar_NRZI(' ', HIGH);
    sendAprsStringLen(statusMessage, strlen(statusMessage));
  }
  else 
   */


  if (type == 1) {  //HAB format (compatible with RS41NG APRS packet format)
    sendAprsStringLen(aprsLocationMsg, strlen(aprsLocationMsg));
    sendAprsChar_NRZI(aprsSymbolOverlay, HIGH);
    sendAprsStringLen(aprsOthersMsg, strlen(aprsOthersMsg));
  } else if (type == 2) {  //WX format
    sendAprsStringLen(aprsWxMsg, strlen(aprsWxMsg));
  } else if (type == 11) {  //HAB format + flight recorder message
    sendAprsStringLen(aprsLocationMsg, strlen(aprsLocationMsg));
    sendAprsChar_NRZI(aprsSymbolOverlay, HIGH);
    sendAprsStringLen(aprsOthersMsg, strlen(aprsOthersMsg));
  }
}

/*
 * This function will send one byte input and convert it
 * into AFSK signal one bit at a time LSB first.
 * 
 * The encode which used is NRZI (Non Return to Zero, Inverted)
 * bit 1 : transmitted as no change in tone
 * bit 0 : transmitted as change in tone
 */
void sendAprsChar_NRZI(unsigned char in_byte, bool enaprsBitStuffingCounter) {
  bool bits;

  for (int i = 0; i < 8; i++) {
    bits = in_byte & 0x01;

    calcAprsCrc(bits);

    if (bits) {
      aprsSetTone(aprsTone);
      aprsBitStuffingCounter++;

      if ((enaprsBitStuffingCounter) && (aprsBitStuffingCounter == 5)) {
        aprsTone ^= 1;
        aprsSetTone(aprsTone);

        aprsBitStuffingCounter = 0;
      }
    } else {
      aprsTone ^= 1;
      aprsSetTone(aprsTone);

      aprsBitStuffingCounter = 0;
    }

    in_byte >>= 1;
  }
}

void sendAprsStringLen(const char* in_string, int len) {
  for (int j = 0; j < len; j++)
    sendAprsChar_NRZI(in_string[j], HIGH);
}

void sendAprsFlag(unsigned char flag_len) {
  for (int j = 0; j < flag_len; j++)
    sendAprsChar_NRZI(0x7e, LOW);
}

/*
 * In this preliminary test, a packet is consists of FLAG(s) and PAYLOAD(s).
 * Standard APRS FLAG is 0x7e character sent over and over again as a packet
 * delimiter. In this example, 100 flags is used the preamble and 3 flags as
 * the postamble.
 */
void sendAprsPacket(char packet_type) {
  /*
   * AX25 FRAME
   * 
   * ........................................................
   * |  FLAG(s) |  HEADER  | PAYLOAD  | FCS(CRC) |  FLAG(s) |
   * --------------------------------------------------------
   * |  N bytes | 22 bytes |  N bytes | 2 bytes  |  N bytes |
   * --------------------------------------------------------
   * 
   * FLAG(s)  : 0x7e
   * HEADER   : see header
   * PAYLOAD  : 1 byte data type + N byte info
   * FCS      : 2 bytes calculated from HEADER + PAYLOAD
   */
  writeRegister(0x72, 0x00);

  aprsTone = 0;
  sendAprsFlag(100);
  aprsCrc = 0xffff;
  aprsBitStuffingCounter = 0;  // Reset aprsBitStuffingCounter counter
  sendAprsHeader();
  sendAprsPayload(packet_type);
  sendAprsCrc();
  sendAprsFlag(3);
}


void convert_degrees_to_dmh(long x, int16_t* degrees, uint8_t* minutes, uint8_t* h_minutes) {
  uint8_t sign = (uint8_t)(x > 0 ? 1 : 0);
  if (!sign) {
    x = -(x);
  }
  *degrees = (int16_t)(x / 1000000);
  x = x - (*degrees * 1000000);
  x = (x)*60 / 10000;
  *minutes = (uint8_t)(x / 100);
  *h_minutes = (uint8_t)(x - (*minutes * 100));
  if (!sign) {
    *degrees = (int16_t) - *degrees;
  }
}

void aprsLocationFormat(float latitude, float longitude, char* aprsMessage) {
  // Latitude buffer (8 characters: DDMM.ssN/S)
  char latBuffer[9];
  // Longitude buffer (9 characters: DDDMM.ssE/W)
  char lonBuffer[10];

  // Convert Latitude to DMH format
  int16_t latDegrees;
  uint8_t latMinutes, latHMinutes;
  convert_degrees_to_dmh((long)(latitude * 1000000), &latDegrees, &latMinutes, &latHMinutes);

  // Determine Hemisphere for Latitude
  char latHemisphere = (latitude >= 0) ? 'N' : 'S';

  // Format Latitude: ddmm.ssN or ddmm.ssS
  sprintf(latBuffer, "%02d%02d.%02d%c", abs(latDegrees), latMinutes, latHMinutes, latHemisphere);

  // Convert Longitude to DMH format
  int16_t lonDegrees;
  uint8_t lonMinutes, lonHMinutes;
  convert_degrees_to_dmh((long)(longitude * 1000000), &lonDegrees, &lonMinutes, &lonHMinutes);

  // Determine Hemisphere for Longitude
  char lonHemisphere = (longitude >= 0) ? 'E' : 'W';

  // Format Longitude: dddmm.ssE or dddmm.ssW
  sprintf(lonBuffer, "%03d%02d.%02d%c", abs(lonDegrees), lonMinutes, lonHMinutes, lonHemisphere);

  // Combine the latitude and longitude into the aprsMessage
  // Format: !ddmm.ssN/dddmm.ssE
  sprintf(aprsMessage, "!%s/%s", latBuffer, lonBuffer);
}


void aprsHabFormat(char* aprsMessage) {
  // Convert gpsAlt from meters to feet
  int gpsAltFeet = static_cast<int>(gpsAlt * 3.28084);

  int aprsBoardRev = 0;
  if(rsm4x4) {
    aprsBoardRev = 4;
  }
  else if(rsm4x2) {
    aprsBoardRev = 2;
  }

  /* OLD RS41ng APRS FORMAT Format the string into the provided aprsMessage buffer
  snprintf(aprsMessage, 256,
           "/A=%06d/P%dS%dT%dV%04dC%d %s",
           gpsAltFeet, aprsPacketNum, gpsSats, aprsTemperature, static_cast<int>(readBatteryVoltage() * 1000), aprsClimb, aprsComment.c_str());*/

  // New RS41-NFW format
  snprintf(aprsMessage, 320,
           "/A=%06d/F%dS%dV%04dC%dI%dT%dH%dP%dJ%dR%d %s",
           gpsAltFeet, aprsPacketNum, gpsSats, static_cast<int>(readBatteryVoltage() * 1000), static_cast<int>(vVCalc * 100), readAvgIntTemp(), static_cast<int>(mainTemperatureValue), static_cast<int>(humidityValue), static_cast<int>(pressureValue * 10), gpsJamWarning, aprsBoardRev, aprsComment.c_str());
}

void aprsWxFormat(float latitude, float longitude, char* aprsMessage) {
  // Latitude buffer (8 characters: DDMM.ssN/S)
  char latBuffer[9];
  // Longitude buffer (9 characters: DDDMM.ssE/W)
  char lonBuffer[10];

  // Convert Latitude to DMH format using convert_degrees_to_dmh
  int16_t latDegrees;
  uint8_t latMinutes, latHMinutes;
  convert_degrees_to_dmh((long)(latitude * 1000000), &latDegrees, &latMinutes, &latHMinutes);

  // Determine Hemisphere for Latitude
  char latHemisphere = (latitude >= 0) ? 'N' : 'S';

  // Format Latitude: ddmm.ssN or ddmm.ssS
  sprintf(latBuffer, "%02d%02d.%02d%c", abs(latDegrees), latMinutes, latHMinutes, latHemisphere);

  // Convert Longitude to DMH format using convert_degrees_to_dmh
  int16_t lonDegrees;
  uint8_t lonMinutes, lonHMinutes;
  convert_degrees_to_dmh((long)(longitude * 1000000), &lonDegrees, &lonMinutes, &lonHMinutes);

  // Determine Hemisphere for Longitude
  char lonHemisphere = (longitude >= 0) ? 'E' : 'W';

  // Format Longitude: dddmm.ssE or dddmm.ssW
  sprintf(lonBuffer, "%03d%02d.%02d%c", abs(lonDegrees), lonMinutes, lonHMinutes, lonHemisphere);

  // Convert Celsius to Fahrenheit
  int wxTemperatureF = static_cast<int>(mainTemperatureValue * 9.0 / 5.0 + 32);

  // Additional WX tags (assuming these functions are defined and return valid data)
  int wxHumidity = 0;

  if (humidityValue >= 100) {
    wxHumidity = 0;
  } else if (humidityValue == 0) {
    wxHumidity = 1;
  } else {
    wxHumidity = humidityValue;  // wxHumidity as percentage
  }

  
  int wxPressure = pressureValue * 10;  // Barometric pressure (hPa * 10)

  // Read additional values
  int wxPacketNumber = 0;      // Packet number, here as 0, because it could potentially overflow when used as a weather station
  int wxSatellites = gpsSats;  // GPS satellites

  if(pressureMode == 1) {
    snprintf(aprsMessage, 320,
           "!%s/%s_.../...g...t%03dh%02db%05d U%dmV %s",   
           latBuffer,                                      // Formatted Latitude buffer
           lonBuffer,                                      // Formatted Longitude buffer
           wxTemperatureF,                                 // Temperature in Fahrenheit
           wxHumidity,                                     // Humidity
           wxPressure,                                     // Pressure
           static_cast<int>(readBatteryVoltage() * 1000),  // Battery voltage (mV)
           aprsComment.c_str()                             // APRS comment
    );    
  }
  else {
    snprintf(aprsMessage, 320,
           "!%s/%s_.../...g...t%03dh%02d U%dmV %s",   //NOTE!: if You want to report pressure, add 'b%05d' after humidity tag and a wxPressure variable after wxHumidity variable
           latBuffer,                                      // Formatted Latitude buffer
           lonBuffer,                                      // Formatted Longitude buffer
           wxTemperatureF,                                 // Temperature in Fahrenheit
           wxHumidity,                                     // Humidity
           static_cast<int>(readBatteryVoltage() * 1000),  // Battery voltage (mV)
           aprsComment.c_str()                             // APRS comment
  );
  }

}


void aprsRecorderFormat(char* aprsMessage) {
  // Convert gpsAlt from meters to feet
  int gpsAltFeet = static_cast<int>(gpsAlt * 3.28084);

  // Convert battery voltage to integer (e.g., 3.75V -> 375)
  int wxVoltageFormatted = static_cast<int>(readBatteryVoltage() * 100);

  int aprsTemperature = static_cast<int>(
    (!sensorBoomEnable || sensorBoomFault) ? readAvgIntTemp() : mainTemperatureValue);

  int healthStatus = 0;
  if (err) {
    healthStatus = 2;
  } else if (warn) {
    healthStatus = 1;
  } else {
    healthStatus = 0;
  }

  int ledsEnableInt = static_cast<int>(ledsEnable ? 1 : 0);
  int beganFlyingInt = static_cast<int>(beganFlying ? 1 : 0);
  int burstDetectedInt = static_cast<int>(burstDetected ? 1 : 0);
  int radioTemp = static_cast<int>(readRadioTemp());
  int mvBatUInt = static_cast<int>(readBatteryVoltage() * 1000);
  int thermistorTempInt = static_cast<int>(readThermistorTemp());
  int gpsJamWarningInt = static_cast<int>(gpsJamWarning ? 1 : 0);
  int sensorBoomFaultInt = static_cast<int>(sensorBoomFault ? 1 : 0);
  int gpsHdopInt = static_cast<int>(gpsHdop);

  int gpsCurrentModeInt = 0;
  if (gpsOperationMode == 0) {
    gpsCurrentModeInt = 0;
  } else {
    gpsCurrentModeInt = currentGPSPowerMode;
  }

  // Format the string into the provided aprsMessage buffer (with a 55 character comment)
  snprintf(
    aprsMessage, 320,
    " NFW;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d %s",
    maxAlt,
    maxSpeed,
    maxAscentRate,
    maxDescentRate,
    maxMainTemperature,
    minMainTemperature,
    maxInternalTemp,
    minInternalTemp,
    ledsEnableInt,
    healthStatus,
    gpsResetCounter,
    beganFlyingInt,
    burstDetectedInt,
    currentRadioPwrSetting,
    gpsCurrentModeInt,
    radioTemp,
    sensorBoomFaultInt,
    zeroHumidityCapacitance,
    humidityCapacitanceRangeDelta,
    extHeaterPwmStatus,
    referenceHeaterStatus,
    mvBatUInt,
    thermistorTempInt,
    gpsJamWarningInt,
    gpsHdopInt,
    aprsComment.c_str());
}


void redLed() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
}

void greenLed() {
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
}

void orangeLed() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
}

void bothLedOff() {
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
}

void flightComputing() {
  if (dataRecorderFlightNoiseFiltering && beganFlying) {
    if (static_cast<int>(gpsAlt) > maxAlt) {
      maxAlt = static_cast<int>(gpsAlt);
    }

    if (static_cast<int>(gpsSpeedKph) > maxSpeed) {
      maxSpeed = static_cast<int>(gpsSpeedKph);
    }

    if (vVCalc > 0 && vVCalc > maxAscentRate) {
      maxAscentRate = vVCalc;
    }

    if (vVCalc < 0 && vVCalc < maxDescentRate) {
      maxDescentRate = vVCalc;
    }
  }


  if (mainTemperatureValue > maxMainTemperature) {
    maxMainTemperature = static_cast<int>(mainTemperatureValue);
  }
  if (mainTemperatureValue < minMainTemperature) {
    minMainTemperature = static_cast<int>(mainTemperatureValue);
  }

  int tempInternal = static_cast<int>(readAvgIntTemp());
  if (tempInternal > maxInternalTemp) {
    maxInternalTemp = tempInternal;
  }
  if (tempInternal < minInternalTemp) {
    minInternalTemp = tempInternal;
  }


  if (gpsAlt > flightDetectionAltitude && !beganFlying) {
    beganFlying = true;
  }

  if (gpsAlt + burstDetectionThreshold < maxAlt && !burstDetected) {
    burstDetected = true;
  }

  if (gpsAlt < flightDetectionAltitude && !hasLanded) {
    if (beganFlying && burstDetected) {
      hasLanded = true;
      landingTimeMillis = millis();
    }
  }

  if (disableGpsImprovementInFlight && beganFlying) {
    cancelGpsImprovement = true;
  }

  if (beganFlying && burstDetected) {
    if (lowAltitudeFastTxThreshold != 0 && gpsAlt < lowAltitudeFastTxThreshold) {
      if (horusEnable) {
        lowAltitudeFastTxModeBeginTime = millis();
        gpsOperationMode = 1;
        lowAltitudeFastTxMode();
        lowAltitudeFastTxModeEnd = true;
      }
    }
  }
}

void lowAltitudeFastTxMode() {
  setRadioPower(7);
  while (millis() - lowAltitudeFastTxModeBeginTime < lowAltitudeFastTxDuration && !lowAltitudeFastTxModeEnd) {
    gpsHandler();
    sensorBoomHandler();
    pressureHandler();

    if (horusEnable) {
      int pkt_len = build_horus_binary_packet_v2(rawbuffer);
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      setRadioModulation(0);  // CW modulation
      setRadioFrequency(horusFreqTable[0]);

      radioEnableTx();

      fsk4_preamble(horusPreambleLength);
      fsk4_write(codedbuffer, coded_len);
      radioDisableTx();
    }
    
    if (horusV3Enable) {
      int pkt_len = buildHorusV3Packet(rawbuffer);

      // Bomb out if we can't encode
      if (pkt_len == 0){
        return;
      }
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      setRadioModulation(0);  // CW modulation
      setRadioFrequency(horusV3FreqTable[0]);

      radioEnableTx();

      fsk4_preamble(horusPreambleLength);
      fsk4_write(codedbuffer, coded_len);
      radioDisableTx();
    }

    if (aprsEnable) {
      setRadioModulation(2);
      setRadioFrequency((aprsFreqTable[0] - 0.002));  //its lower due to the deviation in FSK adding 0.002MHz when the signal is in total 10kHz wide

      aprsLocationFormat(gpsLat, gpsLong, aprsLocationMsg);
      aprsHabFormat(aprsOthersMsg);

      aprsPacketNum++;

      radioEnableTx();
      for (int i = 0; i < 128; i++) {
        aprsSendMark();
      }
      sendAprsPacket(aprsOperationMode);  //send HAB APRS format packet
      radioDisableTx();
    }

    delay(lowAltitudeFastTxInterval);
  }
}


void autoResetHandler() {
  if (autoResetEnable && millis() >= SYSTEM_RESET_PERIOD) {
    NVIC_SystemReset();
  }
}

void initRecorderData() {
  if (!recorderInitialized) {
    maxAlt = gpsAlt;
    maxSpeed = gpsSpeedKph;

    maxAscentRate = vVCalc;
    maxDescentRate = vVCalc;

    maxMainTemperature = static_cast<int>(mainTemperatureValue);
    minMainTemperature = static_cast<int>(mainTemperatureValue);

    int tempInternal = static_cast<int>(readAvgIntTemp());
    maxInternalTemp = tempInternal;
    minInternalTemp = tempInternal;

    recorderInitialized = true;
  }
}


void pipTx() {
  if (pipEnable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: PIP mode enabled");
    }

    if (radioEnablePA) {
      setRadioPower(pipRadioPower);
      setRadioModulation(0);
      setRadioFrequency(pipFrequencyMhz);

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: PIP on (MHz): ");
        xdataSerial.println(pipFrequencyMhz);
        xdataSerial.println("[info]: Transmitting PIP...");
      }

      for (txRepeatCounter; txRepeatCounter < pipRepeat; txRepeatCounter++) {
        radioEnableTx();

        if (xdataPortMode == 1) {
          xdataSerial.print("pip ");
        }

        delay(pipLengthMs);
        radioDisableTx();
        delay(pipLengthMs);
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: PIP TX done");
      }

      txRepeatCounter = 0;
    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}

void morseTx() {
  if (morseEnable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: Morse mode enabled");
    }

    if (radioEnablePA) {
      morseMsg = createRttyMorsePayload();
      const char* morseMsgCstr = morseMsg.c_str();
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Morse payload created: ");
        xdataSerial.println(morseMsg);
        xdataSerial.println();
      }

      setRadioPower(morseRadioPower);
      setRadioModulation(0);  // CW modulation
      setRadioFrequency(morseFrequencyMhz);

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Morse transmitting on (MHz): ");
        xdataSerial.println(morseFrequencyMhz);
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Transmitting morse...");
      }

      transmitMorseString(morseMsgCstr, morseUnitTime);
      radioDisableTx();

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Morse TX done");
      }

    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}

void rttyTx() {
  if (rttyEnable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: RTTY mode enabled");
    }

    if (radioEnablePA) {
      rttyMsg = createRttyMorsePayload();

      const char* rttyMsgCstr = rttyMsg.c_str();
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: RTTY payload created: ");
        xdataSerial.println(rttyMsg);
        xdataSerial.println("");
      }

      setRadioPower(rttyRadioPower);
      setRadioModulation(0);  // CW modulation
      setRadioFrequency(rttyFrequencyMhz);
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: RTTY frequency set to (MHz): ");
        xdataSerial.println(rttyFrequencyMhz);
      }

      radioEnableTx();

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Transmitting RTTY");
      }

      sendRTTYPacket(rttyMsgCstr);
      radioDisableTx();

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: RTTY TX done");
      }

    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won' transmit");
      }
    }
  }
}


void horusTx() {
  // Calculate the number of frequencies in the table automatically
  int freqTableSize = sizeof(horusFreqTable) / sizeof(horusFreqTable[0]);

  if (horusEnable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: HORUS V2 mode enabled");
    }

    if (radioEnablePA) {
      // 1. Prepare the payload once
      int pkt_len = build_horus_binary_packet_v2(rawbuffer);
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HORUS V2 payload created.");
      }

      // 2. Loop through every frequency in the table
      for (int i = 0; i < freqTableSize; i++) {
        float currentFreq = horusFreqTable[i];

        // Configure Radio for this specific hop
        setRadioPower(horusRadioPower);
        setRadioModulation(0);  // CW modulation
        setRadioFrequency(currentFreq);

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: Transmitting on (MHz): ");
          xdataSerial.println(currentFreq);
        }

        // 3. Physical Transmission
        radioEnableTx();

        fsk4_preamble(horusPreambleLength);
        fsk4_write(codedbuffer, coded_len);

        radioDisableTx();

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: Done on ");
          xdataSerial.println(currentFreq);
        }
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: All HORUS V2 frequencies transmitted");
      }

    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}

void horusV3Tx() {
  // Calculate the number of frequencies in the table automatically
  int freqTableSize = sizeof(horusFreqTable) / sizeof(horusFreqTable[0]);

  if (horusV3Enable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: HORUS V3 mode enabled");
    }

    if (radioEnablePA) {
      // 1. Prepare the payload once
      int pkt_len = buildHorusV3Packet(rawbuffer);
      // Bomb out if we can't encode
      if (pkt_len == 0){
        return;
      }

      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HORUS V3 payload created.");
      }

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: HORUS V3 payload created.");

        xdataSerial.print(F("Uncoded Length (bytes): "));
        xdataSerial.println(pkt_len);
        xdataSerial.print("Uncoded: ");
        PrintHex(rawbuffer, pkt_len, debugbuffer);
        xdataSerial.println(debugbuffer);
        xdataSerial.print(F("Encoded Length (bytes): "));
        xdataSerial.println(coded_len);
        xdataSerial.print("Coded: ");
        PrintHex(codedbuffer, coded_len, debugbuffer);
        xdataSerial.println(debugbuffer);
      }

      // 2. Loop through every frequency in the table
      for (int i = 0; i < freqTableSize; i++) {
        float currentFreq = horusV3FreqTable[i];

        // Configure Radio for this specific hop
        setRadioPower(horusV3RadioPower);
        setRadioModulation(0);  // CW modulation
        setRadioFrequency(currentFreq);

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: Transmitting on (MHz): ");
          xdataSerial.println(currentFreq);
        }

        // 3. Physical Transmission
        radioEnableTx();

        fsk4_preamble(horusPreambleLength);
        fsk4_write(codedbuffer, coded_len);

        radioDisableTx();

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: Done on ");
          xdataSerial.println(currentFreq);
        }
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: All HORUS V3 frequencies transmitted");
      }

    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}


void aprsTx() {
  if (aprsEnable) {
    // Calculate size locally from the global array
    int tableSize = sizeof(aprsFreqTable) / sizeof(aprsFreqTable[0]);

    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: APRS mode enabled");
    }

    if (radioEnablePA) {
      for (int f = 0; f < tableSize; f++) {
        float currentFreq = aprsFreqTable[f];
        float adjustedFreq = currentFreq - 0.002;

        setRadioPower(aprsRadioPower);
        setRadioModulation(2);
        setRadioFrequency(adjustedFreq);

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: APRS frequency set to (MHz): ");
          xdataSerial.println(adjustedFreq, 3);
          xdataSerial.println("[info]: Transmitting APRS");
        }

        if (aprsOperationMode == 2) {
          aprsWxFormat(gpsLat, gpsLong, aprsWxMsg);
        } else {
          aprsLocationFormat(gpsLat, gpsLong, aprsLocationMsg);
          aprsHabFormat(aprsOthersMsg);
        }

        aprsPacketNum++;

        radioEnableTx();
        for (int i = 0; i < 128; i++) {
          aprsSendMark();
        }
        sendAprsPacket(aprsOperationMode);
        radioDisableTx();

        // Calibration tones only on the first frequency
        if (aprsToneCalibrationMode && f == 0) {
          radioEnableTx();
          for (int i = 0; i < 5000; i++) { aprsSendSpace(); }
          for (int i = 0; i < 5000; i++) { aprsSendMark(); }
          radioDisableTx();
        }

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: APRS TX done on index: ");
          xdataSerial.println(f);
        }

        if (f < (tableSize - 1)) delay(200);
      }
    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}

void dataRecorderTx() {
  if (dataRecorderEnable && millis() - lastDataRecorderTransmission > dataRecorderInterval) {    
    if (horusV3Enable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: HORUS V3 mode enabled");
    }

    if (radioEnablePA) {
      // 1. Prepare the payload once
      int pkt_len = buildHorusV3PacketDataRecorder(rawbuffer);
      // Bomb out if we can't encode
      if (pkt_len == 0){
        return;
      }

      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HORUS V3 payload created.");
      }

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: HORUS V3 payload created.");

        xdataSerial.print(F("Uncoded Length (bytes): "));
        xdataSerial.println(pkt_len);
        xdataSerial.print("Uncoded: ");
        PrintHex(rawbuffer, pkt_len, debugbuffer);
        xdataSerial.println(debugbuffer);
        xdataSerial.print(F("Encoded Length (bytes): "));
        xdataSerial.println(coded_len);
        xdataSerial.print("Coded: ");
        PrintHex(codedbuffer, coded_len, debugbuffer);
        xdataSerial.println(debugbuffer);
      }


        // Configure Radio for this specific hop
        setRadioPower(horusV3RadioPower);
        setRadioModulation(0);  // CW modulation
        setRadioFrequency(horusV3FreqTable[0]);


        // 3. Physical Transmission
        radioEnableTx();

        fsk4_preamble(horusPreambleLength);
        fsk4_write(codedbuffer, coded_len);

        radioDisableTx();

        delay(500);

        radioEnableTx();

        fsk4_preamble(horusPreambleLength);
        fsk4_write(codedbuffer, coded_len);

        radioDisableTx();

      

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: All HORUS V3 frequencies transmitted");
      }

    } 
  }


    lastDataRecorderTransmission = millis();
  }
}

void ultraPowerSaveHandler() {
  if (ultraPowerSaveAfterLanding) {
    if (hasLanded && millis() - landingTimeMillis > 1200000) {  //20 minutes after landing
      gpsOperationMode = 0;
      sensorBoomEnable = false;
      ledStatusEnable = false;
      selectSensorBoom(0, 0);
      setRadioPower(6);  //NOTE: power save mode changes the power to 50mW, which may not be what a powersave is meant to be. However, sonde laying on the ground has a very poor radio propagation and range, therefore a couple second long transmission won't impact it much
      for (;;) {
        if (horusEnable) {
          int pkt_len = build_horus_binary_packet_v2(rawbuffer);
          int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

          setRadioModulation(0);  // CW modulation
          setRadioFrequency(horusFreqTable[0]);

          radioEnableTx();

          fsk4_preamble(horusPreambleLength);
          fsk4_write(codedbuffer, coded_len);
          radioDisableTx();
        }

        if (horusV3Enable) {
          int pkt_len = buildHorusV3Packet(rawbuffer);

          // Bomb out if we can't encode
          if (pkt_len == 0){
            return;
          }
          int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

          setRadioModulation(0);  // CW modulation
          setRadioFrequency(horusV3FreqTable[0]);

          radioEnableTx();

          fsk4_preamble(horusPreambleLength);
          fsk4_write(codedbuffer, coded_len);
          radioDisableTx();
        }


        if (aprsEnable) {
          setRadioModulation(2);
          setRadioFrequency((aprsFreqTable[0] - 0.002));  //its lower due to the deviation in FSK adding 0.002MHz when the signal is in total 10kHz wide

          aprsLocationFormat(gpsLat, gpsLong, aprsLocationMsg);
          aprsHabFormat(aprsOthersMsg);

          aprsPacketNum++;

          radioEnableTx();
          for (int i = 0; i < 128; i++) {
            aprsSendMark();
          }
          sendAprsPacket(aprsOperationMode);  //send HAB APRS format packet
          radioDisableTx();


          if (dataRecorderEnable) {
            aprsLocationFormat(gpsLat, gpsLong, aprsLocationMsg);
            aprsRecorderFormat(aprsOthersMsg);  //dataRecorder format

            aprsPacketNum++;

            radioEnableTx();
            for (int i = 0; i < 128; i++) {
              aprsSendMark();
            }

            sendAprsPacket(1);
            radioDisableTx();
          }
        }
        unsigned long modeChangeDelayCallbackTimer = millis() + 300000;

        while (millis() < modeChangeDelayCallbackTimer) {
          buttonHandler();
          deviceStatusHandler();
          gpsHandler();
          powerHandler();
          redLed();
          delay(50);
          bothLedOff();
          delay(950);
        }

        autoResetHandler();
      }
    }
  }
}

void temperatureCalibration() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Automatic temperature calibration - performing readings...");
  }

  if (autoTemperatureCalibration) {

    if (autoTemperatureCalibrationMethod == 1) {  //using constant start environment temperature
      if (xdataPortMode == 4) {
        xdataSerial.println("stage: 11");
      }
    } else if (autoTemperatureCalibrationMethod == 2) {  //based on the PCB temperature
      if (xdataPortMode == 4) {
        xdataSerial.println("stage: 12");
      }
    }

    mainTemperatureCorrectionC = 0;
    sensorBoomHandler();

    if (autoTemperatureCalibrationMethod == 1) {  //using constant start environment temperature
      mainTemperatureCorrectionC = environmentStartupAirTemperature - mainTemperatureValue;

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Method 1 - via environment constant");
      }
    } else if (autoTemperatureCalibrationMethod == 2) {  //based on the PCB temperature
      float internalTemperature = readAvgIntTemp();
      float selfHeatingCorrectedInternalTemperature = -8.5 + 1.307 * internalTemperature - 0.001461 * pow(internalTemperature, 2) - 0.000082 * pow(internalTemperature, 3);
      mainTemperatureCorrectionC = selfHeatingCorrectedInternalTemperature - mainTemperatureValue;

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Method 2 - via PCB temperature.");
        xdataSerial.print("[info]: Average PCB temperature = ");
        xdataSerial.print(internalTemperature);
        xdataSerial.print("*C, estimated air temperature (empirical, polynomial self-heating and heat-capacity -assumed correction) = ");
        xdataSerial.print(selfHeatingCorrectedInternalTemperature);
        xdataSerial.println("*C");
      }
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("mainTemperatureCorrectionC = ");
      xdataSerial.print(mainTemperatureCorrectionC);
      xdataSerial.println("*C");
    }
  }
  if (autoHumidityModuleTemperatureCorrection) {  //automatically correct the humidity module readings - simple calibration
    if (xdataPortMode == 4) {
      xdataSerial.println("stage: 15");
    }


    extHeaterTemperatureCorrectionC = 0;
    sensorBoomHandler();
    extHeaterTemperatureCorrectionC = mainTemperatureValue - extHeaterTemperatureValue;


    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: Automatic humidity module temperature calibration - extHeaterTemperatureCorrectionC = ");
      xdataSerial.print(extHeaterTemperatureCorrectionC);
      xdataSerial.println("*C");
    }
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Automatic temperature calibration and correction complete");
  }
}

void reconditioningPhase() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Entering reconditioning phase...");
    xdataSerial.println("[WARN]: The humidity module WILL GET HOT SOON! Don't touch it with anything.");
  }

  if (xdataPortMode == 4) {
    xdataSerial.println("stage: 20");
  }

  unsigned long reconBeginMillis = millis();
  delay(500);

  while (millis() - reconBeginMillis < 60000) {  //1 minute
    orangeLed();
    delay(100);
    bothLedOff();
    sensorBoomHandler();
    buttonHandlerSimplified();
    interfaceHandler();

    if (sensorBoomHumidityModuleError) {
      extHeaterHandler(false, 0, 0);

      for (int i = 0; i < 5; i++) {
        redLed();
        delay(250);
        bothLedOff();
        delay(300);
      }

      calibrationError = true;

      if (xdataPortMode == 1) {
        xdataSerial.println("[ERR]: Sensor boom measurement error (check boom and connection) - exiting reconditioning...");
      }

      if (xdataPortMode == 4) {
        xdataSerial.println("e: 220");
      }

      return;
    }


    extHeaterHandler(true, reconditioningTemperature, extHeaterTemperatureValue);

    if (xdataPortMode == 1) {
      xdataSerial.print("[WARN]: Current humidity module temperature = ");
      xdataSerial.print(extHeaterTemperatureValue);
      xdataSerial.println("*C");
    }
  }

  extHeaterHandler(false, 0, 0);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Reconditioning phase completed. Heating OFF.");
  }
}

void zeroHumidityCheck() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Entering zero-humidity calibration...");
    xdataSerial.println("[WARN]: The humidity module WILL GET HOT! During 0-humidity check, please keep the device in a stable environment, with little to no wind, RH < 70%, temperature > 0*C and don't touch, submerge, blow, lick or do anything with the sensor.");
  }

  if (xdataPortMode == 4) {
    xdataSerial.println("stage: 21");
  }


  unsigned long measurement = 0;
  float capMeasurement = 0;
  unsigned int measurementCount = 0;
  unsigned long measurementBeginMillis;

  buttonHandlerSimplified();

  sensorBoomHandler();

  if (sensorBoomHumidityModuleError) {  //sensor boom error
    for (int i = 0; i < 5; i++) {
      redLed();
      delay(250);
      bothLedOff();
      delay(300);
      buttonHandlerSimplified();
    }

    calibrationError = true;
    extHeaterHandler(false, 0, 0);

    if (xdataPortMode == 1) {
      xdataSerial.println("[ERR]: Sensor boom measurement error (check boom and connection) - exiting calibration...");
    }

    if (xdataPortMode == 4) {
      xdataSerial.println("e: 221");
    }

    return;
  } else if (extHeaterTemperatureValue > 50 && extHeaterTemperatureValue < -10) {  //wrong measurement conditions (heater temperature sensor)
    for (int i = 0; i < 4; i++) {
      redLed();
      delay(250);
      bothLedOff();
      delay(300);
      buttonHandlerSimplified();
    }

    calibrationError = true;
    extHeaterHandler(0, 0, 0);

    if (xdataPortMode == 1) {
      xdataSerial.println("[ERR]: Wrong measurement conditions read from humidity temperature sensor - verify the envorionment, temperature calibration and settings. Exiting calibration...");
    }

    if (xdataPortMode == 4) {
      xdataSerial.println("e: 222");
    }

    return;
  } else {
    for (int i = 0; i < 7; i++) {
      sensorBoomHandler();
      buttonHandlerSimplified();
      interfaceHandler();
      extHeaterHandler(true, 160, extHeaterTemperatureValue);

      orangeLed();
      delay(100);
      bothLedOff();

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Preheating sensor = ");
        xdataSerial.print(extHeaterTemperatureValue);
        xdataSerial.println("*C");
      }
    }
  }

  buttonHandlerSimplified();
  bothLedOff();


  measurementBeginMillis = millis();

  while (measurementCount < 8) {
    buttonHandlerSimplified();
    sensorBoomHandler();
    extHeaterHandler(true, humidityCalibrationMeasurementTemperature + 12, extHeaterTemperatureValue);
    interfaceHandler();

    if (sensorBoomHumidityModuleError) {
      extHeaterHandler(false, 0, 0);

      for (int i = 0; i < 5; i++) {
        redLed();
        delay(250);
        bothLedOff();
        delay(300);
      }

      calibrationError = true;

      if (xdataPortMode == 1) {
        xdataSerial.println("[ERR]: Sensor boom measurement error (check boom and connection) - exiting calibration...");
      }

      if (xdataPortMode == 4) {
        xdataSerial.println("e: 221");
      }

      return;
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: Humidity module temperature = ");
      xdataSerial.print(extHeaterTemperatureValue);
      xdataSerial.println("*C");
    }

    if (extHeaterTemperatureValue > humidityCalibrationMeasurementTemperature - 10 && extHeaterTemperatureValue < humidityCalibrationMeasurementTemperature + 20 && !sensorBoomHumidityModuleError) {
      orangeLed();
      delay(50);
      bothLedOff();

      measurement += humidityFrequency;
      capMeasurement += humidityCapacitance;
      measurementCount++;

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Taking measurement ");
        xdataSerial.print(measurementCount);
        xdataSerial.println("/8.");
      }
    } else {
      orangeLed();
      delay(100);
      bothLedOff();
    }

    if (millis() - measurementBeginMillis > humidityCalibrationTimeout) {
      calibrationError = true;
      extHeaterHandler(false, 0, 0);

      for (int i = 0; i < 5; i++) {
        redLed();
        delay(250);
        bothLedOff();
        delay(300);
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[WARN]: Calibration timeout. This could be due to unstable environment parameters, bad settings, hardware or user error.");
      }

      if (xdataPortMode == 4) {
        xdataSerial.println("e: 221");
      }

      return;
    }
  }

  extHeaterHandler(false, 0, 0);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Zero humidity calibration complete.");
  }

  zeroHumidityFrequency = (measurement / measurementCount);
  zeroHumidityCapacitance = (capMeasurement / measurementCount);
}

void flightHeatingHandler() {
  if (referenceHeating) {
    float cutOutTemp = readThermistorTemp();  //maintaining reference area temperature of ~20*C

    if (cutOutTemp >= referenceAreaTargetTemperature + 3) {
      selectReferencesHeater(0);  // Heating off when target+3 < temperature
    } else if (cutOutTemp > referenceAreaTargetTemperature + 1 && cutOutTemp < referenceAreaTargetTemperature + 3) {
      selectReferencesHeater(1);  // Low power when target+3 > temperature > target+1
    } else if (cutOutTemp <= referenceAreaTargetTemperature + 1 && cutOutTemp > referenceAreaTargetTemperature - 1) {
      selectReferencesHeater(2);  // Medium power when target+1 >= temperature > target -1
    } else if (cutOutTemp <= referenceAreaTargetTemperature - 1 && cutOutTemp > referenceAreaTargetTemperature - 3.5) {
      // If at low power (1), only increase to medium (2), not high (3)
      if (referenceHeaterStatus < 2) {
        selectReferencesHeater(2);  // Gradual increase from 1 to 2
      } else {
        selectReferencesHeater(3);  // If already at 2, allow 3
      }
    } else if (cutOutTemp <= referenceAreaTargetTemperature - 3.5) {
      selectReferencesHeater(3);  // Ensure high power if temp drops too much
    } else {
      selectReferencesHeater(0);
    }


    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: Reference area heating handler, cut-out temperature = ");
      xdataSerial.println(cutOutTemp);
    }
  }

  //humidity module heating algorithm - above -40C target is equal to air_temp+offset, below -40C ambient temp the module maintains -40C
  if (humidityModuleHeating && !sensorBoomFault) {

    if (extHeaterTemperatureValue < humidityModuleHeatingTemperatureThreshold) {
      extHeaterHandler(true, max((float)humicapMinimumTemperature, mainTemperatureValue + defrostingOffset), extHeaterTemperatureValue);
    } else {
      extHeaterHandler(false, 0, 0);
    }
  }
}


int readAvgIntTemp() {
  int radioTemp = static_cast<int>(readRadioTemp());
  int thermistorTemp = static_cast<int>(readThermistorTemp());

  if (abs(radioTemp) > 120) {  //in case of error
    radioTemp = thermistorTemp;
  } else if (abs(thermistorTemp) > 150) {
    thermistorTemp = radioTemp;
  }

  if(pressureMode == 1 && !rpm411Error) {
    return static_cast<int>( (radioTemp + thermistorTemp + static_cast<int>(rpm411InternalTemperature) ) / 3);
  }
  else {
    return static_cast<int>( (radioTemp + thermistorTemp) / 2);
  }
}

void generateSi4032FmTone(unsigned int toneFrequency, unsigned int lengthMs) {
  // Calculate the tone period in microseconds
  unsigned int tonePeriodMicroseconds = 1000000 / toneFrequency;

  // Get the start time in milliseconds
  unsigned long toneBeginTime = millis();

  // Generate the tone for the specified duration
  while (millis() - toneBeginTime < lengthMs) {
    writeRegister(0x72, 0x00);
    writeRegister(0x73, 0x0F);
    delayMicroseconds(tonePeriodMicroseconds / 2);  //these frequencies will differ slightly according to CPU speed et.c (APRS tone generation has different delays between rsm4x2 and rsm4x4 but here it isn't needed that much)
    writeRegister(0x72, 0x00);
    writeRegister(0x73, 0x00);
    delayMicroseconds(tonePeriodMicroseconds / 2);
  }
}

void foxHuntMiscHandler() {
  buttonHandlerSimplified();
  float vBat = readBatteryVoltage();
  if (vBat < vBatWarnValue) {
    orangeLed();
    delay(1000);
    bothLedOff();
  } else {
    greenLed();
    delay(50);
    bothLedOff();
  }

  if (vBat < batteryCutOffVoltage && batteryCutOffVoltage != 0) {
    hardwarePowerShutdown();
  }
}


void foxHuntModeLoop() {
  setRadioPower(foxHuntRadioPower);
  for (;;) {
    foxHuntMiscHandler();

    if (foxHuntFmMelody) {
      setRadioModulation(2);                          //FSK modulation
      setRadioFrequency((foxHuntFrequency - 0.003));  //its lower due to the deviation in FSK adding 0.002MHz when the signal is in total 10kHz wide
      radioEnableTx();
      for (int i = 0; i < 7; i++) {
        generateSi4032FmTone(330, 750);
        generateSi4032FmTone(392, 750);
        generateSi4032FmTone(523, 750);
        generateSi4032FmTone(784, 1500);

        buttonHandlerSimplified();
      }
      radioDisableTx();
    }

    foxHuntMiscHandler();
    delay(foxHuntTransmissionDelay);  //blocking delay, it doesn't have to be advanced, it justs plays melodies to find it :)
    foxHuntMiscHandler();

    if (foxHuntCwTone) {
      setRadioModulation(0);  // CW modulation
      setRadioFrequency(foxHuntFrequency);
      radioEnableTx();
      delay(10000);
      radioDisableTx();
    }

    foxHuntMiscHandler();
    delay(foxHuntTransmissionDelay);
    foxHuntMiscHandler();


    if (foxHuntMorseMarker) {
      morseMsg = String(foxMorseMsg) + String(" Vb=") + String(readBatteryVoltage());
      const char* morseMsgCstr = morseMsg.c_str();

      setRadioModulation(0);
      setRadioFrequency(foxHuntFrequency);
      transmitMorseString(morseMsgCstr, morseUnitTime);
      radioDisableTx();
    }

    foxHuntMiscHandler();
    delay(foxHuntTransmissionDelay);
    foxHuntMiscHandler();

    if (foxHuntLowVoltageAdditionalMarker && readBatteryVoltage() < vBatWarnValue) {
      morseMsg = String(foxMorseMsgVbat) + String(" L_Vb=") + String(readBatteryVoltage());  //L_Vb = Low Voltage. Battery voltage =
      const char* morseMsgCstr = morseMsg.c_str();

      setRadioModulation(0);
      setRadioFrequency(foxHuntFrequency);
      transmitMorseString(morseMsgCstr, morseUnitTime);
      radioDisableTx();
    }

    foxHuntMiscHandler();
    delay(foxHuntTransmissionDelay);
    foxHuntMiscHandler();
  }
}


void humidityModuleHeaterPowerControl(unsigned int heaterPower) {  //0 - OFF, 1-255 - only low power heater PWM, 256-500 - low power heater at max and high power heater PWM-controlled
  extHeaterPwmStatus = heaterPower;

  if (xdataPortMode == 1) {
    xdataSerial.print("[info]: Humidity module heating power is ");
    xdataSerial.print(heaterPower);
    xdataSerial.println("/500 .");
  }

  if (heaterPower == 0) {
    analogWrite(HEAT_HUM1, 0);
    analogWrite(HEAT_HUM2, 0);
    digitalWrite(HEAT_HUM1, LOW);
    digitalWrite(HEAT_HUM2, LOW);
  } else if (heaterPower >= 1 && heaterPower <= 255) {
    analogWrite(HEAT_HUM1, 0);
    digitalWrite(HEAT_HUM1, LOW);

    analogWrite(HEAT_HUM2, heaterPower);
  } else if (heaterPower >= 256 && heaterPower < 500) {
    analogWrite(HEAT_HUM1, (heaterPower - 255));

    analogWrite(HEAT_HUM2, 255);
    digitalWrite(HEAT_HUM2, HIGH);
  } else if (heaterPower == 500) {
    analogWrite(HEAT_HUM1, 255);
    analogWrite(HEAT_HUM2, 255);
    digitalWrite(HEAT_HUM1, HIGH);
    digitalWrite(HEAT_HUM2, HIGH);
  } else {
    analogWrite(HEAT_HUM1, 0);
    analogWrite(HEAT_HUM2, 0);
    digitalWrite(HEAT_HUM1, LOW);
    digitalWrite(HEAT_HUM2, LOW);
  }
}

void gpsQuietMode() {
    if (xdataPortMode == 1) xdataSerial.println(F("[info]: Entering GPS Quiet Mode"));

    unsigned long startQuietMillis = millis();
    unsigned long lastUpdateMillis = startQuietMillis;

    // Block execution while radioSilenceDuration not elapsed or cancelled
    while ((millis() - startQuietMillis < radioSilenceDuration) && !cancelGpsImprovement) {

        unsigned long now = millis();
        unsigned long elapsed = now - lastUpdateMillis;

        // Update monotonic system time
        if (elapsed > 0) {
            systemTimeMillis += elapsed;
            lastUpdateMillis = now;
        }

        if (gpsSats < 5) {
          bothLedOff(); delay(100);
          orangeLed();  delay(100);
          bothLedOff(); delay(100);
          orangeLed();  delay(100);
        }
        else if (gpsSats >= 5) {
          greenLed();
          delay(80);
          bothLedOff();
        }

        // Run handlers to keep system responsive
        gpsHandler();
        buttonHandler();
        interfaceHandler();
        sensorBoomHandler();
        flightHeatingHandler();
        pressureHandler();
    }

    if (xdataPortMode == 1) xdataSerial.println(F("[info]: Exiting GPS Quiet Mode"));

    // Resync reference after blocking loop
    lastMillisUpdate = millis();
}


void schedulerInit() {
  lastMillisUpdate = millis();
  systemTimeMillis = 0;
  gpsTimeSynced = false;
  
  // Initialize all next transmission times to 0 (will transmit immediately on first run)
  nextPipTxTime = 0;
  nextHorusV3TxTime = 0;
  nextHorusTxTime = 0;
  nextAprsTxTime = 0;
  nextRttyTxTime = 0;
  nextMorseTxTime = 0;
}

void schedulerLoop() {

  static unsigned long lastSensorUpdate = 0;
  static unsigned long lastSyncPrint = 0;
  static unsigned long previousSeconds = 0;

  const unsigned long SENSOR_UPDATE_INTERVAL = 30000UL;

  unsigned long nowMillis = millis();
  unsigned long elapsed = nowMillis - lastMillisUpdate;

  // millis() rollover safe
  if (nowMillis < lastMillisUpdate) {
    elapsed = nowMillis;
  }

  if (elapsed > 0) {
    systemTimeMillis += elapsed;
    lastMillisUpdate = nowMillis;
  }

  unsigned long currentSeconds = systemTimeMillis / 1000UL;

  if (currentSeconds < previousSeconds) {
    nextPipTxTime      = 0;
    nextHorusV3TxTime  = 0;
    nextHorusTxTime    = 0;
    nextAprsTxTime     = 0;
    nextRttyTxTime     = 0;
    nextMorseTxTime    = 0;

    if (xdataPortMode == 1) {
      xdataSerial.println(F("[warn]: Time anomaly detected, schedules reset"));
    }
  }
  previousSeconds = currentSeconds;

  if (gpsSats >= 4 && gps.time.isValid() && gps.time.age() < 1000) {

    unsigned long gpsSeconds =
        (unsigned long)gps.time.hour()   * 3600UL +
        (unsigned long)gps.time.minute() * 60UL +
        (unsigned long)gps.time.second();

    // Compare only within 24h window
    long localDaySeconds = (long)(currentSeconds % 86400UL);
    long diff = (long)gpsSeconds - localDaySeconds;

    // Wrap correction
    if (diff > 43200L)  diff -= 86400L;
    if (diff < -43200L) diff += 86400L;

    if (labs(diff) >= 2) {
      systemTimeMillis += diff * 1000L;
      lastMillisUpdate = millis();
      currentSeconds = systemTimeMillis / 1000UL;

      if (xdataPortMode == 1) {
        xdataSerial.print(F("[info]: GPS clock adjust "));
        xdataSerial.print(diff);
        xdataSerial.println(F("s"));
      }
    }

    if (!gpsTimeSynced || millis() - lastSyncPrint >= 5000) {
      if (xdataPortMode == 1 && !gpsTimeSynced) {
        xdataSerial.print(F("[info]: GPS synced "));
        if (gps.time.hour() < 10) xdataSerial.print('0');
        xdataSerial.print(gps.time.hour());
        xdataSerial.print(':');
        if (gps.time.minute() < 10) xdataSerial.print('0');
        xdataSerial.print(gps.time.minute());
        xdataSerial.print(':');
        if (gps.time.second() < 10) xdataSerial.print('0');
        xdataSerial.println(gps.time.second());
      }
      lastSyncPrint = millis();
    }

    gpsTimeSynced = true;

  } else {
    if (gpsTimeSynced && xdataPortMode == 1) {
      xdataSerial.println(F("[info]: GPS sync lost"));
    }
    gpsTimeSynced = false;
  }


  if (improvedGpsPerformance && !cancelGpsImprovement && gpsSats < 4) {
    gpsQuietMode();
  }


  unsigned long minTimeUntilTx = 0xFFFFFFFFUL;
  unsigned long targetTxTime = 0;

  #define CHECK_NEXT_TX(t) \
    if ((t) > currentSeconds) { \
      unsigned long d = (t) - currentSeconds; \
      if (d < minTimeUntilTx) { minTimeUntilTx = d; targetTxTime = (t); } \
    }

  CHECK_NEXT_TX(nextPipTxTime);
  CHECK_NEXT_TX(nextHorusV3TxTime);
  CHECK_NEXT_TX(nextHorusTxTime);
  CHECK_NEXT_TX(nextAprsTxTime);
  CHECK_NEXT_TX(nextRttyTxTime);
  CHECK_NEXT_TX(nextMorseTxTime);
  
  if (minTimeUntilTx > 0 && minTimeUntilTx <= 3) {

    unsigned long waitStart = millis();

    if (xdataPortMode == 1) {
      xdataSerial.println(F("[info]: Syncing TX"));
    }

    while (currentSeconds < targetTxTime) {

      if (millis() - waitStart > 4000UL) break;

      unsigned long lNow = millis();
      unsigned long lElapsed = lNow - lastMillisUpdate;
      systemTimeMillis += lElapsed;
      lastMillisUpdate = lNow;
      currentSeconds = systemTimeMillis / 1000UL;

      buttonHandler();
      delay((targetTxTime - currentSeconds > 1) ? 50 : 5);
    }
  }


  #define SCHEDULE_TX(enable, nextVar, offset, period, txFunc) \
    if (enable) { \
      if ((nextVar) == 0) { \
        unsigned long p = (currentSeconds - (offset)) % (period); \
        (nextVar) = currentSeconds - p + (period); \
      } \
      if (currentSeconds >= (nextVar)) { \
        txFunc(); \
        unsigned long p = (currentSeconds - (offset)) % (period); \
        (nextVar) = currentSeconds - p + (period); \
      } \
    }

  SCHEDULE_TX(pipEnable,     nextPipTxTime,     pipTimeSyncOffsetSeconds,     pipTimeSyncSeconds,     pipTx);
  SCHEDULE_TX(horusV3Enable, nextHorusV3TxTime, horusV3TimeSyncOffsetSeconds, horusV3TimeSyncSeconds, horusV3Tx);
  SCHEDULE_TX(horusEnable,   nextHorusTxTime,   horusTimeSyncOffsetSeconds,   horusTimeSyncSeconds,   horusTx);
  SCHEDULE_TX(aprsEnable,    nextAprsTxTime,    aprsTimeSyncOffsetSeconds,    aprsTimeSyncSeconds,    aprsTx);
  SCHEDULE_TX(rttyEnable,    nextRttyTxTime,    rttyTimeSyncOffsetSeconds,    rttyTimeSyncSeconds,    rttyTx);
  SCHEDULE_TX(morseEnable,   nextMorseTxTime,   morseTimeSyncOffsetSeconds,   morseTimeSyncSeconds,   morseTx);

  dataRecorderTx();
  gpsHandler();
  deviceStatusHandler();
  flightComputing();
  buttonHandler();
  powerHandler();
  initRecorderData();
  flightHeatingHandler();
  ultraPowerSaveHandler();
  autoResetHandler();


  unsigned long cm = millis();
  bool forceUpdate = (cm - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL);

  if (minTimeUntilTx > 3 || forceUpdate) {
    if (sensorBoomPowerSaving && (cm - lastSensorUpdate >= 30000UL)) {
      sensorBoomHandler();
      lastSensorUpdate = cm;
    }
    else if (!sensorBoomPowerSaving) {
      sensorBoomHandler();
      lastSensorUpdate = cm;
    }

    pressureHandler();
    interfaceHandler();

  }

}








void interfaceHandler() {
  #ifndef RSM4x2
  if (xdataPortMode != 4 || gpsAlt > flightDetectionAltitude) return;

  // --- 1. SYSTEM & HARDWARE ---
  xdataSerial.print("isRSM4x2: ");
  xdataSerial.println(rsm4x2);
  xdataSerial.print("isRSM4x4: ");
  xdataSerial.println(rsm4x4);
  xdataSerial.print("fwVer: ");
  xdataSerial.println(NFW_VERSION);
  xdataSerial.print("millis: ");
  xdataSerial.println(millis());
  xdataSerial.print("autoResetEnable: ");
  xdataSerial.println(autoResetEnable);
  xdataSerial.print("buttonMode: ");
  xdataSerial.println(buttonMode);
  xdataSerial.print("ledStatusEnable: ");
  xdataSerial.println(ledStatusEnable);
  xdataSerial.print("ledAutoDisableHeight: ");
  xdataSerial.println(ledAutoDisableHeight);

  // --- 2. GPS CONFIG & DATA ---
  xdataSerial.print("ubloxGpsAirborneMode: ");
  xdataSerial.println(ubloxGpsAirborneMode);
  xdataSerial.print("gpsTimeoutWatchdog: ");
  xdataSerial.println(gpsTimeoutWatchdog);
  xdataSerial.print("improvedGpsPerformance: ");
  xdataSerial.println(improvedGpsPerformance);
  xdataSerial.print("disableGpsImprovementInFlight: ");
  xdataSerial.println(disableGpsImprovementInFlight);
  xdataSerial.print("gpsOperationMode: ");
  xdataSerial.println(gpsOperationMode);
  xdataSerial.print("gpsLat: ");
  xdataSerial.print(gpsLat, 6);
  xdataSerial.println();
  xdataSerial.print("gpsLong: ");
  xdataSerial.print(gpsLong, 6);
  xdataSerial.println();
  xdataSerial.print("gpsAlt: ");
  xdataSerial.println(gpsAlt);
  xdataSerial.print("gpsSats: ");
  xdataSerial.println(gpsSats);
  xdataSerial.print("gpsHours: ");
  xdataSerial.println(gpsHours);
  xdataSerial.print("gpsMinutes: ");
  xdataSerial.println(gpsMinutes);
  xdataSerial.print("gpsSeconds: ");
  xdataSerial.println(gpsSeconds);
  xdataSerial.print("gpsSpeedKph: ");
  xdataSerial.println(gpsSpeedKph);
  xdataSerial.print("gpsHdop: ");
  xdataSerial.println(gpsHdop);
  xdataSerial.print("vVCalc: ");
  xdataSerial.println(vVCalc);
  xdataSerial.print("currentGPSPowerMode: ");
  xdataSerial.println(gpsStatus);
  xdataSerial.print("gpsJamWarning: ");
  xdataSerial.println(gpsJamWarning);

  // --- 3. RADIO GENERAL ---
  xdataSerial.print("radioEnablePA: ");
  xdataSerial.println(radioEnablePA);
  xdataSerial.print("callsign: ");
  xdataSerial.println(CALLSIGN);
  xdataSerial.print("radioTemp: ");
  xdataSerial.println(readRadioTemp());

  // --- 4. PIP MODE ---
  xdataSerial.print("pipEnable: ");
  xdataSerial.println(pipEnable);
  xdataSerial.print("pipFrequencyMhz: ");
  xdataSerial.println(pipFrequencyMhz, 3);
  xdataSerial.print("pipTimeSyncSeconds: ");
  xdataSerial.println(pipTimeSyncSeconds);
  xdataSerial.print("pipTimeSyncOffsetSeconds: ");
  xdataSerial.println(pipTimeSyncOffsetSeconds);
  xdataSerial.print("pipRadioPower: ");
  xdataSerial.println(pipRadioPower);

  // --- 5. HORUS MODE ---
  xdataSerial.print("horusEnable: ");
  xdataSerial.println(horusEnable);
  xdataSerial.print("horusFrequencyMhz: ");
  xdataSerial.println(horusFreqTable[0], 3);
  xdataSerial.print("horusTimeSyncSeconds: ");
  xdataSerial.println(horusTimeSyncSeconds);
  xdataSerial.print("horusTimeSyncOffsetSeconds: ");
  xdataSerial.println(horusTimeSyncOffsetSeconds);
  xdataSerial.print("horusPayloadId: ");
  xdataSerial.println(horusPayloadId);
  xdataSerial.print("horusRadioPower: ");
  xdataSerial.println(horusRadioPower);

  // --- 5.1. HORUS V3 MODE ---
  xdataSerial.print("horusV3Enable: ");
  xdataSerial.println(horusV3Enable);
  xdataSerial.print("horusV3FrequencyMhz: ");
  xdataSerial.println(horusFreqTable[0], 3);
  xdataSerial.print("horusV3TimeSyncSeconds: ");
  xdataSerial.println(horusTimeSyncSeconds);
  xdataSerial.print("horusV3TimeSyncOffsetSeconds: ");
  xdataSerial.println(horusTimeSyncOffsetSeconds);
  xdataSerial.print("horus_v3_callsign: ");
  xdataSerial.println(HORUS_V3_CALLSIGN);
  xdataSerial.print("horusV3RadioPower: ");
  xdataSerial.println(horusV3RadioPower);

  // --- 6. APRS MODE ---
  xdataSerial.print("aprsEnable: ");
  xdataSerial.println(aprsEnable);
  xdataSerial.print("aprsFrequencyMhz: ");
  xdataSerial.println(aprsFreqTable[0], 3);
  xdataSerial.print("aprsTimeSyncSeconds: ");
  xdataSerial.println(aprsTimeSyncSeconds);
  xdataSerial.print("aprsTimeSyncOffsetSeconds: ");
  xdataSerial.println(aprsTimeSyncOffsetSeconds);
  xdataSerial.print("aprsCall: ");
  xdataSerial.println(aprsCall);
  xdataSerial.print("aprsComment: ");
  xdataSerial.println(aprsComment);
  xdataSerial.print("aprsSsid: ");
  xdataSerial.println(aprsSsid);
  xdataSerial.print("aprsOperationMode: ");
  xdataSerial.println(aprsOperationMode);
  xdataSerial.print("aprsRadioPower: ");
  xdataSerial.println(aprsRadioPower);
  xdataSerial.print("aprsToneCalibrationMode: ");
  xdataSerial.println(aprsToneCalibrationMode);

  // --- 7. RTTY & MORSE ---
  xdataSerial.print("rttyEnable: ");
  xdataSerial.println(rttyEnable);
  xdataSerial.print("rttyFrequencyMhz: ");
  xdataSerial.println(rttyFrequencyMhz, 3);
  xdataSerial.print("rttyTimeSyncSeconds: ");
  xdataSerial.println(rttyTimeSyncSeconds);
  xdataSerial.print("rttyTimeSyncOffsetSeconds: ");
  xdataSerial.println(rttyTimeSyncOffsetSeconds);
  xdataSerial.print("rttyRadioPower: ");
  xdataSerial.println(rttyRadioPower);
  xdataSerial.print("morseEnable: ");
  xdataSerial.println(morseEnable);
  xdataSerial.print("morseFrequencyMhz: ");
  xdataSerial.println(morseFrequencyMhz, 3);
  xdataSerial.print("morseTimeSyncSeconds: ");
  xdataSerial.println(morseTimeSyncSeconds);
  xdataSerial.print("morseTimeSyncOffsetSeconds: ");
  xdataSerial.println(morseTimeSyncOffsetSeconds);
  xdataSerial.print("morseRadioPower: ");
  xdataSerial.println(morseRadioPower);

  // --- 8. BATTERY & POWER ---
  xdataSerial.print("batV: ");
  xdataSerial.println(readBatteryVoltage());
  xdataSerial.print("vBatWarnValue: ");
  xdataSerial.println(vBatWarnValue);
  xdataSerial.print("batteryCutOffVoltage: ");
  xdataSerial.println(batteryCutOffVoltage);
  xdataSerial.print("ultraPowerSaveAfterLanding: ");
  xdataSerial.println(ultraPowerSaveAfterLanding);

  // --- 9. FLIGHT STATISTICS ---
  xdataSerial.print("maxAlt: ");
  xdataSerial.println(maxAlt);
  xdataSerial.print("maxSpeed: ");
  xdataSerial.println(maxSpeed);
  xdataSerial.print("maxAscentRate: ");
  xdataSerial.println(maxAscentRate);
  xdataSerial.print("maxDescentRate: ");
  xdataSerial.println(maxDescentRate);
  xdataSerial.print("maxMainTemperature: ");
  xdataSerial.println(maxMainTemperature);
  xdataSerial.print("minMainTemperature: ");
  xdataSerial.println(minMainTemperature);
  xdataSerial.print("maxInternalTemp: ");
  xdataSerial.println(maxInternalTemp);
  xdataSerial.print("minInternalTemp: ");
  xdataSerial.println(minInternalTemp);
  xdataSerial.print("beganFlying: ");
  xdataSerial.println(beganFlying);
  xdataSerial.print("burstDetected: ");
  xdataSerial.println(burstDetected);
  xdataSerial.print("hasLanded: ");
  xdataSerial.println(hasLanded);
  xdataSerial.print("flightDetectionAltitude: ");
  xdataSerial.println(flightDetectionAltitude);
  xdataSerial.print("burstDetectionThreshold: ");
  xdataSerial.println(burstDetectionThreshold);
  xdataSerial.print("lowAltitudeFastTxThreshold: ");
  xdataSerial.println(lowAltitudeFastTxThreshold);

  // --- 10. SENSOR BOOM & HEATING ---
  xdataSerial.print("sensorBoomEnable: ");
  xdataSerial.println(sensorBoomEnable);
  xdataSerial.print("thermistorTemp: ");
  xdataSerial.println(readThermistorTemp());
  xdataSerial.print("mainTemperatureValue: ");
  xdataSerial.println(mainTemperatureValue);
  xdataSerial.print("humidityValue: ");
  xdataSerial.println(humidityValue);
  xdataSerial.print("pressureValue: ");
  xdataSerial.println(pressureValue);
  xdataSerial.print("mainTemperatureCorrectionC: ");
  xdataSerial.println(mainTemperatureCorrectionC);
  xdataSerial.print("extHeaterTemperatureCorrectionC: ");
  xdataSerial.println(extHeaterTemperatureCorrectionC);
  xdataSerial.print("autoTemperatureCalibration: ");
  xdataSerial.println(autoTemperatureCalibration);
  xdataSerial.print("autoTemperatureCalibrationMethod: ");
  xdataSerial.println(autoTemperatureCalibrationMethod);
  xdataSerial.print("environmentStartupAirTemperature: ");
  xdataSerial.println(environmentStartupAirTemperature);
  xdataSerial.print("autoHumidityModuleTemperatureCorrection: ");
  xdataSerial.println(autoHumidityModuleTemperatureCorrection);
  xdataSerial.print("humidityModuleEnable: ");
  xdataSerial.println(humidityModuleEnable);
  xdataSerial.print("zeroHumidityCalibration: ");
  xdataSerial.println(zeroHumidityCalibration);
  xdataSerial.print("humidityCapacitanceRangeDelta: ");
  xdataSerial.println(humidityCapacitanceRangeDelta);
  xdataSerial.print("zeroHumidityCapacitance: ");
  xdataSerial.println(zeroHumidityCapacitance);
  xdataSerial.print("tempSensorBoomCalibrationFactor: ");
  xdataSerial.println(tempSensorBoomCalibrationFactor);
  xdataSerial.print("calibrationError: ");
  xdataSerial.println(calibrationError);
  xdataSerial.print("extHeaterPwmStatus: ");
  xdataSerial.println(extHeaterPwmStatus);
  xdataSerial.print("referenceHeaterStatus: ");
  xdataSerial.println(referenceHeaterStatus);
  xdataSerial.print("referenceAreaTargetTemperature: ");
  xdataSerial.println(referenceAreaTargetTemperature);
  xdataSerial.print("humidityModuleHeating: ");
  xdataSerial.println(humidityModuleHeating);
  xdataSerial.print("sensorBoomMainTempError: ");
  xdataSerial.println(sensorBoomMainTempError);
  xdataSerial.print("sensorBoomHumidityModuleError: ");
  xdataSerial.println(sensorBoomHumidityModuleError);
  xdataSerial.print("pressureMode: ");
  xdataSerial.println(pressureMode);
  xdataSerial.print("pressureValue: ");
  xdataSerial.println(pressureValue);
  xdataSerial.print("rpm411InternalTemperature: ");
  xdataSerial.println(rpm411InternalTemperature);
  xdataSerial.print("RPM411SerialNumber: ");
  xdataSerial.println(RPM411SerialNumber);
  xdataSerial.print("rpm411Error: ");
  xdataSerial.println(rpm411Error);
  xdataSerial.print("seaLevelPressure: ");
  xdataSerial.println(seaLevelPressure);

  // --- 11. RAW SENSOR DATA & CONSTANTS ---
  xdataSerial.print("mainTemperatureFrequency: ");
  xdataSerial.println(mainTemperatureFrequency);
  xdataSerial.print("mainTemperatureResistance: ");
  xdataSerial.println(mainTemperatureResistance);
  xdataSerial.print("extHeaterTemperatureFrequency: ");
  xdataSerial.println(extHeaterTemperatureFrequency);
  xdataSerial.print("extHeaterTemperatureResistance: ");
  xdataSerial.println(extHeaterTemperatureResistance);
  xdataSerial.print("extHeaterTemperatureValue: ");
  xdataSerial.println(extHeaterTemperatureValue);
  xdataSerial.print("humidityCapacitance: ");
  xdataSerial.println(humidityCapacitance);
  xdataSerial.print("maxHumidityCapacitance: ");
  xdataSerial.println(maxHumidityCapacitance);
  xdataSerial.print("THERMISTOR_R25: ");
  xdataSerial.println(THERMISTOR_R25);
  xdataSerial.print("THERMISTOR_B: ");
  xdataSerial.println(THERMISTOR_B);

  // --- 12. RECORDER & LOGIC FLAGS ---
  xdataSerial.print("dataRecorderEnable: ");
  xdataSerial.println(dataRecorderEnable);
  xdataSerial.print("dataRecorderInterval: ");
  xdataSerial.println(dataRecorderInterval);
  xdataSerial.print("dataRecorderFlightNoiseFiltering: ");
  xdataSerial.println(dataRecorderFlightNoiseFiltering);
  xdataSerial.print("recorderInitialized: ");
  xdataSerial.println(recorderInitialized);

  // --- 13. SUMMARY FLAGS ---
  xdataSerial.print("err: ");
  xdataSerial.println(err);
  xdataSerial.print("warn: ");
  xdataSerial.println(warn);
  xdataSerial.print("ok: ");
  xdataSerial.println(ok);
  #endif
}

void extHeaterHandler(bool enable, float targetTemp, float currentTemp) {
  static float heaterPower = 0;
  static float integral = 0;
  static float previousError = 0;
  static unsigned long lastTime = 0;

  if (!enable || sensorBoomFault) {
    humidityModuleHeaterPowerControl(0);
    integral = 0;
    previousError = 0;
    lastTime = 0;
    return;
  }

  unsigned long now = millis();
  float dt = (lastTime == 0) ? 5.0 : (now - lastTime) / 1000.0;
  lastTime = now;

  float error = targetTemp - currentTemp;

  // Integral (more headroom!)
  integral += error * dt;
  if (integral > 1050.0) integral = 1050.0;
  if (integral < 0) integral = 0;

  // Derivative only when far
  float derivative = 0;
  if (abs(error) > 15) {
    derivative = (error - previousError) / dt;
  }

  float output = extHeaterProportionalK * error + extHeaterIntegralK * integral + extHeaterDerivativeK * derivative;
  output = constrain(output, 0, 500);

  humidityModuleHeaterPowerControl((int)output);
  previousError = error;
}

void humidityDeltaCalibrationDebug() {
  if (humidityCalibrationDebug) {
    if (xdataPortMode == 4) {
      xdataSerial.println("stage: 25");
    }

    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: Entering calibration adjustment mode...");
    }

    while (extHeaterTemperatureValue > 40) {
      if (xdataPortMode == 1) {
        xdataSerial.print("Waiting for the humidity module to cool down (<40C) after calibration - T=");
        xdataSerial.print(extHeaterTemperatureValue);
        xdataSerial.println(" *C");
      }

      orangeLed();
      delay(50);
      bothLedOff();

      sensorBoomHandler();
    }

    if (xdataPortMode == 1) {
      xdataSerial.println("Hardware ready - place the sensor in 100%RH environment and observe the suggested humidityRangeDelta value.");
    }

    if (xdataPortMode == 4) {
      xdataSerial.println("stage: 26");
    }

    for (;;) {
      greenLed();
      delay(50);
      bothLedOff();

      sensorBoomHandler();

      humidityCapacitanceRangeDelta = (humidityCapacitance - zeroHumidityCapacitance) * 0.9;

      if (xdataPortMode == 1) {
        xdataSerial.print("humidityCapacitance = ");
        xdataSerial.print(humidityCapacitance);
        xdataSerial.print("uF,  humidityCapacitanceRangeDelta = ");
        xdataSerial.print(humidityCapacitanceRangeDelta);
        xdataSerial.println("Hz");
      }

      interfaceHandler();

      if (analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN) && analogRead(VBAT_PIN) > 100) {
        if (xdataPortMode == 1) {
          xdataSerial.print("Turning the sonde OFF. Reprogram it with an average humidityRangeDelta calculated for this sensor boom and disable the humidityCalibrationDebug.");
        }

        if (xdataPortMode == 4) {
          xdataSerial.print("stage: 27");
        }

        hardwarePowerShutdown();
      }
    }
  }
}



void initRPM411() {
  bool isDataReceived = false;
  delay(50);

  for (int i = 0; i < 21; i++) {

    for (int j = 0; j < 7; j++) {
      digitalWrite(CS_SPI, LOW);
      delayMicroseconds(70);

      RPM411ConfigData[i][j] = SPI_2.transfer(RPM411InitFrame[i][j]);

      if (RPM411ConfigData[i][j] != 0xFF) {
        isDataReceived = true;
      }

      digitalWrite(CS_SPI, HIGH);
      delayMicroseconds(90);
    }

    delayMicroseconds(450);

    for (int k = 7; k < 33; k++) {
      digitalWrite(CS_SPI, LOW);
      delayMicroseconds(70);

      RPM411ConfigData[i][k] = SPI_2.transfer(0x00);

      if (RPM411ConfigData[i][k] != 0xFF) {
        isDataReceived = true;
      }

      digitalWrite(CS_SPI, HIGH);
      delayMicroseconds(90);
    }

    delay(10);
  }


  if (!isDataReceived) {
    rpm411Error = true;
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: RPM411 connection error");
    }
  }
  else {
    rpm411Error = false;
    RPM411ParseConfigData();
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: RPM411 OK");
    }
  }
}

void readRPM411() {
  bool isDataReceived = false;

  for (int j = 0; j < 5; j++) {
    digitalWrite(CS_SPI, LOW);
    delayMicroseconds(100);

    SPI_2.transfer(RPM411PreReadoutFrame[j]);

    digitalWrite(CS_SPI, HIGH);
    delayMicroseconds(100);
  }

  delay(250);


  for (int j = 0; j < 5; j++) {
    digitalWrite(CS_SPI, LOW);
    delayMicroseconds(100);

    RPM411ReadingsData[j] = SPI_2.transfer(RPM411TriggerReadoutFrame[j]);

    if (RPM411ReadingsData[j] != 0xFF) {
      isDataReceived = true;
    }

    digitalWrite(CS_SPI, HIGH);
    delayMicroseconds(100);
  }

  delayMicroseconds(450);

  for (int k = 5; k < 33; k++) {
    digitalWrite(CS_SPI, LOW);
    delayMicroseconds(100);

    RPM411ReadingsData[k] = SPI_2.transfer(0x00);

    if (RPM411ReadingsData[k] != 0xFF) {
      isDataReceived = true;
    }

    digitalWrite(CS_SPI, HIGH);
    delayMicroseconds(100);
  }


  if (!isDataReceived) {
    rpm411Error = true;
    
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: RPM411 connection error");
    }

  }
  else {
    rpm411Error = false;
    RPM411ParseReadings();
    
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: RPM411 OK");
    }

  }
}


void RPM411ParseConfigData() {
  for (int i = 0; i < 8; i++) {
    RPM411SerialNumber[i] = (char)RPM411ConfigData[1][(41 + i) % 33];
  }

  RPM411SerialNumber[8] = '\0';
}


void RPM411ParseReadings() {
  if(!rpm411Error) {
    int totalLength = 33; 
  
    int tempStart = totalLength - 13;     // Index 20
    int pressureStart = totalLength - 9;  // Index 24

    uint8_t tempBytes[4];
    tempBytes[0] = RPM411ReadingsData[tempStart];
    tempBytes[1] = RPM411ReadingsData[tempStart + 1];
    tempBytes[2] = RPM411ReadingsData[tempStart + 2];
    tempBytes[3] = RPM411ReadingsData[tempStart + 3];
    memcpy(&rpm411InternalTemperature, tempBytes, 4);

    uint8_t pressureBytes[4];
    pressureBytes[0] = RPM411ReadingsData[pressureStart];
    pressureBytes[1] = RPM411ReadingsData[pressureStart + 1];
    pressureBytes[2] = RPM411ReadingsData[pressureStart + 2];
    pressureBytes[3] = RPM411ReadingsData[pressureStart + 3];
    memcpy(&rpm411Pressure, pressureBytes, 4);
  }
}


void pressureHandler() {
  if(pressureMode == 1) {
    if (rpm411Error) {
      initRPM411();
      initRPM411();
      
      if(!rpm411Error) {
        readRPM411();
      }

    }
    else {
      readRPM411();

      if(rpm411Pressure < 1200 && rpm411Pressure > 0) {
        pressureValue = kalmanFilter(rpm411Pressure, pressureKalmanEst, pressureKalmanErrorEst, pressureKalmanError, pressureKalmanQ);
      }

    }

  }
  else if (pressureMode == 2) {

    // --- Constants ---
    const double gMR = 0.0341632;   // g0*M/R (1/m)

    double Pb, Tb, Lb, hb;

    // --- 1. Use measured sea-level pressure directly ---
    // seaLevelPressure in hPa
    double P0 = seaLevelPressure;

    // --- 2. Precompute base pressures for each ISA layer ---
    // Layer 0–11 km (lapse)
    double P11 = P0 * pow(1.0 + (-0.0065 * 11000.0) / 288.15, -gMR / -0.0065);

    // Layer 11–20 km (isothermal)
    double P20 = P11 * exp(-gMR * (20000.0 - 11000.0) / 216.65);

    // Layer 20–32 km (lapse)
    double P32 = P20 * pow(1.0 + (0.0010 * (32000.0 - 20000.0)) / 216.65, -gMR / 0.0010);

    // --- 3. Select layer ---
    if (gpsAlt < 11000.0) {
      Pb = P0;
      Tb = 288.15;
      Lb = -0.0065;
      hb = 0.0;
    }
    else if (gpsAlt < 20000.0) {
      Pb = P11;
      Tb = 216.65;
      Lb = 0.0;
      hb = 11000.0;
    }
    else if (gpsAlt < 32000.0) {
      Pb = P20;
      Tb = 216.65;
      Lb = 0.0010;
      hb = 20000.0;
    }
    else {
      Pb = P32;
      Tb = 228.65;
      Lb = 0.0028;
      hb = 32000.0;
    }

    // --- 4. Final pressure calculation ---
    if (Lb == 0.0) {
      pressureValue = Pb * exp(-gMR * (gpsAlt - hb) / Tb);
    }
    else {
      double term = 1.0 + Lb * (gpsAlt - hb) / Tb;
      if (term > 0.0) {
        pressureValue = Pb * pow(term, -gMR / Lb);
      } else {
        pressureValue = 0.0;  // Safety clamp
      }
    }
  }
  else {
    pressureValue = 0;
  }

}

float kalmanFilter(float measurement, float &est, float &err_est, float err_meas, float q) {
  // 1. Prediction Step
  err_est = err_est + q;

  // 2. Kalman Gain Calculation
  float kalman_gain = err_est / (err_est + err_meas);

  // 3. Update Step
  est = est + kalman_gain * (measurement - est);
  err_est = (1.0 - kalman_gain) * err_est;

  return est;
}



void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(PSU_SHUTDOWN_PIN, OUTPUT);
  pinMode(CS_RADIO_SPI, OUTPUT);
  pinMode(CS_SPI, OUTPUT);
  if (heaterPinControlAvail) {
    pinMode(HEAT_REF, OUTPUT);
  }
  pinMode(PULLUP_TM, OUTPUT);
  pinMode(PULLUP_HYG, OUTPUT);
  pinMode(SPST1, OUTPUT);
  pinMode(SPST2, OUTPUT);
  pinMode(SPST3, OUTPUT);
  pinMode(SPST4, OUTPUT);
  pinMode(SPDT1, OUTPUT);
  pinMode(SPDT2, OUTPUT);
  pinMode(SPDT3, OUTPUT);

  pinMode(HEAT_HUM1, OUTPUT);
  pinMode(HEAT_HUM2, OUTPUT);

  pinMode(GPS_RESET_PIN, OUTPUT);

  digitalWrite(PULLUP_TM, LOW);
  digitalWrite(PULLUP_HYG, LOW);
  digitalWrite(SPST1, LOW);
  digitalWrite(SPST2, LOW);
  digitalWrite(SPST3, LOW);
  digitalWrite(SPST4, LOW);
  digitalWrite(CS_SPI, HIGH);
  digitalWrite(CS_RADIO_SPI, HIGH);

  redLed();

  if (xdataPortMode == 1) {
    xdataSerial.begin(115200);
  } else if (xdataPortMode == 3) {
    xdataSerial.begin(9600);
  } else if (xdataPortMode == 4 || xdataPortMode == 5) {
    xdataSerial.begin(115200);
  }

  if (xdataPortMode == 4) {
    xdataSerial.println("stage: 01");
  }

  if (rsm4x4) {
    gpsSerial.begin(gpsBaudRate);
  } else if (rsm4x2) {
    gpsSerial.begin(gpsBaudRate);
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Serial interfaces initialized");
  }

  if (rsm4x4) {
    analogReadResolution(12);
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: ADC resolution set to 12 bits");
    }
  }

  analogWriteResolution(8);    // Set PWM resolution
  analogWriteFrequency(1000);  // Set PWM frequency
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: PWM timer initialized - 8bit, 1kHz");
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Enabling clock output pins...");
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); // MCO1 with divider 1 from HSI clock source, output on PA8

  SPI_2.begin();
  digitalWrite(CS_RADIO_SPI, HIGH);  // Deselect the SI4432 CS pin
  digitalWrite(CS_SPI, HIGH);


  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: SPI_2 interface initialized, clock ok");
  }

  if (xdataPortMode == 4) {
    xdataSerial.println("stage: 02");
  }

  if (improvedGpsPerformance && gpsOperationMode != 0) {
    shutdownGPS();
  } else {
    startGPS();
    delay(1000);
    initGPS();
  }

  digitalWrite(CS_RADIO_SPI, LOW);
  initSi4032();
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 radio register initialization complete");
  }

  setRadioPower(6);
  if (xdataPortMode == 1) {
    xdataSerial.print("[info]: Si4032 PA power set to default 6 (50mW): ");
    xdataSerial.println(6);
  }

  writeRegister(0x72, 0x05);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 deviation set to 0x07, not used for now?...");
  }

  fsk4_bitDuration = (uint32_t)1000000 / horusBdr;  //horus 100baud delay calculation
  
  if (xdataPortMode == 4) {
    xdataSerial.println("stage: 03");
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Sensor boom, heaters, RPM411 initialization...");
  }

  selectReferencesHeater(0);  //turn off reference heating
  extHeaterHandler(false, 0, 0);
  selectSensorBoom(0, 0);  //turn off all sensor boom measurement circuits

  if(pressureMode == 1) {
    initRPM411();
    readRPM411();
    pressureKalmanEst = pressureValue;

  }
  pressureHandler();

  orangeLed();

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Hardware init seems done");
  }

  if (foxHuntMode) {
    shutdownGPS();
    foxHuntModeLoop();
  }

  if (sensorBoomEnable) {
    temperatureCalibration();
  }

  if (sensorBoomEnable && reconditioningEnabled) {
    reconditioningPhase();
  }

  if (sensorBoomEnable && humidityModuleEnable && zeroHumidityCalibration) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: Starting humidity calibration...");
    }
    zeroHumidityCheck();
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: Exiting calibration.");
    }
  }

  maxHumidityFrequency = zeroHumidityFrequency - humidityRangeDelta;
  maxHumidityCapacitance = zeroHumidityCapacitance + humidityCapacitanceRangeDelta;

  if (xdataPortMode == 1) {
    xdataSerial.print("0RH_cap | 100RH_cap => ");
    xdataSerial.print(zeroHumidityCapacitance);
    xdataSerial.print(" | ");
    xdataSerial.println(maxHumidityCapacitance);
  }

  humidityDeltaCalibrationDebug();

  if (improvedGpsPerformance && gpsOperationMode != 0) {
    startGPS();
    delay(1000);
    initGPS();
  }


  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Exiting setup and entering main program loop...");
  }

  for (int i = 0; i < 5; i++) {
    greenLed();
    delay(50);
    bothLedOff();
    delay(50);
  }

  gpsHandler();

  /*
  if(xdataSerial.available() > 0) {
    xdataSerial.read(); 
    interfaceHandler(); 
    delay(5); 
  }*/

  if (xdataPortMode == 4) {
    xdataSerial.println("stage: 04");
  }

  interfaceHandler();
  schedulerInit();

  sensorBoomHandler();
  humidityKalmanEst = humidityValue;

  bothLedOff();
}


void loop() {
  schedulerLoop();
}
