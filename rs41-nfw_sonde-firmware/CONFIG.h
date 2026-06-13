/*
 * ============================================================
 *  RS41-NFW  -  FIRMWARE CONFIGURATION FILE
 * ============================================================
 *
 *  All user-configurable settings for RS41-NFW are in this
 *  file. Edit the values below to configure the firmware.
 *  Do NOT edit settings inside the main .ino source file.
 *
 *  Please read every comment carefully before changing values.
 *  Up-to-date documentation:  https://github.com/Nevvman18/rs41-nfw
 *
 *  Version 66  |  GPL-3.0  |  Franek Łada (nevvman, SP5FRA)
 * ============================================================
 */

#ifndef NFW_CONFIG_H
#define NFW_CONFIG_H


/* ============================================================
   SECTION 1 - BOARD REVISION
   Uncomment exactly ONE line below to match your PCB.
   The PCB revision is printed at the bottom of the board.
   ============================================================ */

// #define RSM4x4   // RSM4x4 or RSM4x5  (STM32L412RBT6,  LQFP64) - newer boards
// #define RSM4x2   // RSM4x2 or RSM4x1  (STM32F100C8T6B, LQFP48) - older boards



/* ============================================================
   SECTION 2 - TX TIMING CONFIGURATION

   GPS-clock synchronised transmission intervals.

   [mode]TimeSyncSeconds
       Transmit every N seconds, aligned to the top of the hour.
       Example: 15  →  transmit at :00, :15, :30, :45 of each minute.
       Setting to 0 disables that mode's scheduled transmissions.
       Intervals shorter than ~7 s leave very little CPU time for
       sensors, GPS and other tasks - test carefully.

   [mode]TimeSyncOffsetSeconds
       Additional delay after each interval slot before transmitting.
       Example: period=15, offset=2  →  transmits at :02, :17, :32, :47.
       Useful for separating multiple sondes in the air simultaneously.

   NOTE: if you want tight synchronisation together with fast transmissions,
   consider disabling the Data Recorder (Section 24). One data-recorder packet
   takes a long time to send and can overrun its slot, briefly drifting onto the
   transmissions of other sondes nearby.
   ============================================================ */

// Pip (carrier pulse):
constexpr uint16_t pipTimeSyncSeconds        = 15;
constexpr uint16_t pipTimeSyncOffsetSeconds  = 0;

// Horus Binary V3:
constexpr uint16_t horusV3TimeSyncSeconds        = 15;
constexpr uint16_t horusV3TimeSyncOffsetSeconds  = 0;

// Horus Binary V2:
constexpr uint16_t horusTimeSyncSeconds        = 15;
constexpr uint16_t horusTimeSyncOffsetSeconds  = 0;

// APRS:
constexpr uint16_t aprsTimeSyncSeconds        = 30;
constexpr uint16_t aprsTimeSyncOffsetSeconds  = 0;

// RTTY:
constexpr uint16_t rttyTimeSyncSeconds        = 15;
constexpr uint16_t rttyTimeSyncOffsetSeconds  = 0;

// Morse:
constexpr uint16_t morseTimeSyncSeconds        = 15;
constexpr uint16_t morseTimeSyncOffsetSeconds  = 0;


/* ============================================================
   SECTION 3 - RADIO GENERAL
   ============================================================ */

bool radioEnablePA = true;   // Master enable for the radio power amplifier


/* ============================================================
   SECTION 4 - PIP MODE
   Simple unmodulated carrier burst - a beacon that marks the transmitter's
   presence (e.g. for direction finding).
   ============================================================ */

constexpr bool pipEnable       = false;  // Enable Pip TX
constexpr float  pipFrequencyMhz = 432.7; // TX frequency (MHz)
constexpr uint16_t pipLengthMs  = 100;    // Carrier pulse length (ms)
constexpr uint16_t pipRepeat    = 3;      // Pulses per TX window
constexpr int8_t   pipRadioPower = 7;
// TX power:  0=-1 dBm (~0.8 mW)  1=2 dBm (~1.6 mW)  2=5 dBm (~3 mW)
//            3=8 dBm (~6 mW)     4=11 dBm (~12 mW)   5=14 dBm (25 mW)
//            6=17 dBm (50 mW)    7=20 dBm (100 mW)


/* ============================================================
   SECTION 5 - HORUS BINARY V3 MODE
   3rd-generation 4FSK protocol from Project Horus.
   Encoder by Mark Jessop VK5QI.
   No external payload ID needed - callsign is in the packet.
   See: https://github.com/xssfox/horusbinaryv3

   horusV3ExtraSensorsEnable adds extra telemetry fields to each standard packet:
     temperatures.custom1   - humidity-module heater temperature (extHeaterTemperatureValue)
     temperatures.custom2   - cut-out thermistor temperature (readThermistorTemp)
     extraSensors "gpspwr"  - GPS power/operation mode (gpsStatus). Its meaning
                              depends on the board revision:
       RSM4x4 / RSM4x5 (M10) - the NFW Intelligent GPS tier (active in GPS mode 3):
         1 = weak fix (<=10 sats): all/optimized constellations, continuous tracking (fastest acquisition)
         2 = moderate fix (11-15 sats): continuous or cyclic power-save tracking
         3 = strong fix (>=15 sats): cyclic power-save tracking (lowest power)
         (in GPS mode 1 it stays 1 - always max performance)
       RSM4x2 / RSM4x1 (older u-blox) - the GPS power state:
         0 = not set yet (e.g. GPS disabled, mode 0)
         1 = max-performance / continuous tracking
         2 = power-save tracking
         (in GPS mode 2 it alternates between 1 and 2 with the satellite count)
   Leaving it false gives the same payload fields as Horus V2.
   ============================================================ */

constexpr bool horusV3Enable = true;   // Enable Horus V3 TX

constexpr float horusV3FreqTable[] = {437.6};
// Frequency table (MHz). Sonde cycles through all entries each TX window.
// Multi-frequency example: {437.6, 434.714, 433.8}
// lowAltitudeFastTxMode and dataRecorder always use the first entry.

#define HORUS_V3_CALLSIGN "4FSKTEST-V3"
// Payload callsign - each character adds 6 bits to the packet.

constexpr uint16_t horusV3Bdr        = 100;  // Baudrate (bps), default 100
constexpr int8_t   horusV3RadioPower  = 5;    // TX power (see Pip section for key)
constexpr bool     horusV3ExtraSensorsEnable = true; // Include extended telemetry fields (see above)

// Shared Horus V2/V3 settings:
constexpr uint16_t horusBdr          = 100;   // Baudrate (bps)
constexpr int8_t   horusPreambleLength = 8;   // Preamble length (bits); default 16


/* ============================================================
   SECTION 6 - HORUS BINARY V2 MODE
   NOTE: Horus V3 is strongly preferred - V2 is included for
   compatibility with older receiver installations only.
   Obtain your payload ID:
   https://github.com/projecthorus/horusdemodlib/issues
   ============================================================ */

constexpr bool horusEnable = false;   // Enable Horus V2 TX

constexpr float horusFreqTable[] = {437.6};
// Same format as horusV3FreqTable.

constexpr uint16_t horusPayloadId  = 256;  // Horus V2 payload ID (get yours at link above)
constexpr int8_t   horusRadioPower = 5;    // TX power (see Pip section for key)


/* ============================================================
   SECTION 7 - APRS MODE
   ============================================================ */

constexpr bool aprsEnable = true;   // Enable APRS TX

constexpr float aprsFreqTable[] = {432.5};
// Same format as horusV3FreqTable.
// dataRecorder and lowAltitudeFastTxMode use the first entry only.

char aprsCall[]   = "N0CALL";       // Your amateur radio callsign
String aprsComment = " NFWv66";     // Comment appended to every APRS packet

constexpr char aprsSsid          = 11;       // Callsign SSID
constexpr char aprsDest[]        = "APRNFW"; // Destination address
constexpr char aprsDigi[]        = "WIDE2";  // Digipeater callsign
constexpr char aprsDigiSsid      = 1;        // Digipeater SSID
constexpr char aprsSymbolOverlay = 'O';      // 'O' = balloon icon, '_' = WX station
constexpr char aprsSymTable      = 'a';      // Symbol table

/* APRS operation mode:
   1 - Standard NFW HAB tracker format. Fields in comment:
         F=frame  S=sats  V=batt(V)  C=ascent_rate(m/s)  I=temp
         T=ext_temp  H=humidity  P=pressure(hPa)
         J=jam_warning  R=PCB_revision
   2 - Weather station format (APRS WX report) */
constexpr int8_t aprsOperationMode = 1;
constexpr int8_t aprsRadioPower    = 7;   // TX power (see Pip section for key)


/* ============================================================
   SECTION 8 - RTTY MODE
   UKHAS-compliant RTTY. Improvements by OM3BC - thank you!
   ============================================================ */

#define CALLSIGN "N0CALL"   // Callsign for RTTY and Morse

constexpr bool rttyEnable        = false;  // Enable RTTY TX
constexpr float  rttyFrequencyMhz  = 434.6; // TX frequency (MHz)
constexpr uint16_t rttyBitDelay   = 10000;  // Bit period (µs): 22000≈45 Bd, 13333≈75 Bd, 10000≈100 Bd
constexpr uint8_t  rttyBits       = 7;      // Character bit width (7 or 8)
constexpr uint8_t  rttyStopBits   = 2;      // Stop bits (1, 1.5, or 2)
#define RTTY_RADIO_MARK_OFFSET  0x03        // Radio register value for mark tone
#define RTTY_RADIO_SPACE_OFFSET 0x01        // Radio register value for space tone
constexpr int8_t   rttyRadioPower = 7;      // TX power (see Pip section for key)


/* ============================================================
   SECTION 9 - MORSE MODE
   ============================================================ */

constexpr bool morseEnable        = false;  // Enable Morse TX
constexpr float  morseFrequencyMhz = 434.6; // TX frequency (MHz)
constexpr uint16_t morseUnitTime  = 40;     // Unit time (ms); dot = 1 unit, dash = 3 units
constexpr int8_t   morseRadioPower = 7;     // TX power (see Pip section for key)

// Beacon mode: instead of live telemetry, transmit a fixed custom text.
constexpr bool morseBeaconMode    = false;  // false = telemetry Morse, true = fixed-text beacon
String         morseBeaconText    = "N0CALL BEACON"; // Text sent each TX window in beacon mode
constexpr uint8_t morseBeaconRepeat = 1;    // Times the text is repeated per TX window (min 1)


/* ============================================================
   SECTION 10 - FOX-HUNT MODE
   Separate from HAB operation. Transmits FM melody and/or a
   Morse marker on a fixed frequency. Minimises power consumption
   between transmission bursts.

   LED behaviour in fox-hunt mode:
     Green blinking - OK
     Orange         - warning (e.g. low battery)

   NOTE: The Morse marker is transmitted in CW (not FM).
         Use a SSB/CW receiver to decode it.
   ============================================================ */

constexpr bool foxHuntMode                       = false; // Enable fox-hunt mode
constexpr bool foxHuntFmMelody                   = true;  // Transmit FM melody
constexpr bool foxHuntCwTone                     = false; // Transmit a 10 s CW tone instead of melody
constexpr bool foxHuntMorseMarker                = true;  // Transmit Morse marker after melody (CW)
String foxMorseMsg                     = "N0CALL N0CALL FOX";
constexpr bool foxHuntLowVoltageAdditionalMarker = true;  // Send extra marker when battery is low
String foxMorseMsgVbat                 = "N0CALL N0CALL FOX 11.123456 12.456789";
constexpr float    foxHuntFrequency         = 434.5; // TX frequency (MHz)
constexpr uint16_t foxHuntTransmissionDelay = 0;     // Delay between TX cycles (ms)
constexpr int8_t   foxHuntRadioPower        = 7;     // TX power (see Pip section for key)


/* ============================================================
   SECTION 11 - STATUS LEDs

   LED meanings during normal operation:
     Solid Green          - all OK, GPS fix held, no warnings. Ready / flying normally.
     Blinking Orange      - running but no GPS fix yet, or a non-critical warning
                            (searching for satellites, low-ish battery).
     Blinking / Solid Red - an error is active (sensor-boom fault, RPM411 fault,
                            calibration error or battery warning).
     Off                  - LEDs disabled, or above ledAutoDisableHeight.

   LED meanings during startup and calibration:
     Solid Red            - hardware initialisation right after power-on (stages 01-04).
     Blinking Orange      - calibration in progress (temperature / reconditioning /
                            zero-humidity). The module is hot (~140 C) - keep clear of the boom.
     Red blinks           - an error during start-up or calibration (e.g. sensor-boom
                            fault or calibration error).
     Green ×5             - boot complete, system started, entering normal operation.
   ============================================================ */

bool           ledStatusEnable      = true;   // Enable status LEDs
constexpr int16_t ledAutoDisableHeight = 1000; // Altitude (m) above which LEDs turn off


/* ============================================================
   SECTION 12 - XDATA PORT / INSTRUMENTS

   Mode 0 - disabled
   Mode 1 - combined log + telemetry (recommended): human-readable [info]/[warn]/[err] lines
            AND periodic compact $NFW telemetry frames readable by Ground Control Software
   Mode 2 - GPS bridge: raw NMEA echoed to the xdata port at 115200 bps
   Mode 3 - OIF411 ozone sonde (9600 bps) - see Section 12b for OIF411 settings
   ============================================================ */

constexpr uint8_t  xdataPortMode     = 1;    // XDATA port mode (see above)
constexpr uint16_t oif411MsgWaitTime = 1100; // OIF411 response timeout (ms)


/* ============================================================
   SECTION 12b - OIF411 OZONE SONDE SETTINGS  (active when xdataPortMode = 3)

   The OIF411 measures atmospheric ozone by electro-chemical cell (ECC).
   The firmware decodes the cell current and computes ozone partial pressure
   and ppbv internally.

   For accurate results:
   • Use RPM411 for pressure (pressureMode 1) - required for Cef and Ibg correction.
   • Fill the cathode with exactly 3.0 cm³ of 1 % KI solution.
   • Fill the anode with exactly 3.0 cm³ of half-saturated KBr + KHCO₃ solution.
   • Measure your specific pump time (T100) before launch and set ozonePumpTime.
   • Let I0 stabilise through the ozone-destruction filter and record it as
     ozoneBackgroundCurrent before launch.
   • The Data Recorder is strongly recommended to log full OIF411 telemetry.
   ============================================================ */

constexpr float ozonePumpTime          = 28.5f;   // Pump time for 100 ml of air [s]; measure with filter before launch
constexpr float ozoneBackgroundCurrent = 0.015f;  // I0 background current [μA]; measure through ozone-destruction filter


/* ============================================================
   SECTION 13 - POWER MANAGEMENT
   ============================================================ */

float vBatWarnValue        = 0;     // Battery warning voltage (V); 0 = disabled
float batteryCutOffVoltage = 0;
// Auto power-off below this voltage (V); 0 = disabled.
// Useful for NiMH (damage below 0.8 V/cell). Set 0 for Li or maximum runtime.

constexpr bool ultraPowerSaveAfterLanding = false;
// 20 min after landing: GPS off, sensors off, TX every 5 min (Horus+APRS only).
// Useful for maximising battery life before recovery.


/* ============================================================
   SECTION 14 - GPS CONFIGURATION

   gpsOperationMode:
     0 - GPS fully OFF (stationary WX station; set coordinates below)
     1 - Always ON / max performance (default)
     2 - Standard power-save when stable fix (RSM4x2 only)
     3 - Intelligent GPS management with M10 (RSM4x4 only - recommended)
   ============================================================ */

uint8_t gpsOperationMode = 3;

/* RSM4x4 / M10 intelligent GPS options (only when gpsOperationMode = 3):
   m10ConstellationOptimization - auto-select constellations for best perf/power balance
   m10AggressiveOpt             - maximise power saving (may reduce tracking quality!)
   m10CyclicTracking            - cyclic tracking: fix every 10 s then sleep (big power saving)
   m10PerformanceImprovements   - enable all M10 performance enhancements (always recommended)
   m10SuperS                    - u-blox Super-S adaptive power management */
constexpr bool m10ConstellationOptimization = true;
constexpr bool m10AggressiveOpt             = false;
constexpr bool m10CyclicTracking            = true;
constexpr bool m10PerformanceImprovements   = true;
constexpr bool m10SuperS                    = true;

/* Airborne 1G dynamic model - required for tracking above 18 km altitude.
   Set false only for stationary use below 18 km (marginally better accuracy). */
constexpr bool ubloxGpsAirborneMode = true;

/* GPS timeout watchdog - resets the GPS module if no valid fix for this many ms.
   0 = disabled. Recommended: 900000 ms (15 min). */
unsigned long gpsTimeoutWatchdog = 900000;

/* Improved GPS fix performance:
   While GPS has fewer than 4 satellites, the Si4032 radio is silenced for
   radioSilenceDuration ms. Si4032 produces wideband spurious emissions that
   desensitise the GPS receiver; silencing it dramatically speeds up cold starts.
   disableGpsImprovementInFlight stops the silence during flight (avoids 2 min
   data gaps). If flying in high-interference areas, set this false instead. */
constexpr bool improvedGpsPerformance        = true;
constexpr bool disableGpsImprovementInFlight = true;
unsigned long radioSilenceDuration          = 120000; // Radio-silence window (ms)

/* Static coordinates - used when gpsOperationMode = 0, overwritten once GPS active. */
float gpsLat  = 0;   // Latitude  (decimal degrees)
float gpsLong = 0;   // Longitude (decimal degrees)
float gpsAlt  = 0;   // Altitude  (m)

constexpr uint8_t gpsSatsWarnValue     = 4;      // Satellite count below which a warning is shown
unsigned long     gpsPowerSaveDebounce = 300000; // Min interval between GPS power-mode changes (ms)


/* ============================================================
   SECTION 15 - SENSOR BOOM
   ============================================================ */

bool sensorBoomEnable      = true;   // Enable sensor boom measurements and diagnostics
constexpr bool sensorBoomPowerSaving = false;  // Power saving: read the boom only every sensorBoomPowerSavingInterval
unsigned long sensorBoomPowerSavingInterval = 30000; // Boom read interval in power-saving mode (ms), default 30000 = 30 s


/* ============================================================
   SECTION 16 - TEMPERATURE CALIBRATION

   Both correction values below work the same way. All sensor booms behave
   similarly but each has its own unique offset, so switching to a different
   boom requires recalibration. Two options for each (choose one):

     1. Automatic calibration - enable the matching auto-cal flag; it runs
        at each startup.
          - The computed offset can be saved permanently into the firmware:
            read it via serial / Ground Software, enter it as the value here,
            then disable auto-cal so future flights skip the calibration step.
     2. Manual - start at 0, compare the reading against a reference,
        and enter offset = actualTemp - sondeReading by hand.

   mainTemperatureCorrectionC
       Offset (°C) for the main PT1000 temperature sensor (silver hook).
       Auto-cal flag: autoTemperatureCalibration
       (method selected by autoTemperatureCalibrationMethod below).

   extHeaterTemperatureCorrectionC
       Offset (°C) for the humidity-module heater temperature sensor.
       Auto-cal flag: autoHumidityModuleTemperatureCorrection
       (recommended - corrects it against the main hook automatically).
   ============================================================ */

float mainTemperatureCorrectionC              = 0;
float extHeaterTemperatureCorrectionC         = 35;
constexpr bool autoHumidityModuleTemperatureCorrection = true;
constexpr bool autoTemperatureCalibration              = true;

/* autoTemperatureCalibrationMethod:
   1 - Use a known ambient temperature (environmentStartupAirTemperature).
       Most accurate. You MUST power on the sonde in that exact environment.
   2 - Use average PCB temperature. Less accurate; works best 5-32 °C. */
constexpr uint8_t autoTemperatureCalibrationMethod = 1;
float environmentStartupAirTemperature = 24;  // Known ambient temperature (°C) for method 1


/* ============================================================
   SECTION 17 - HUMIDITY CALIBRATION
   ============================================================ */

constexpr bool humidityModuleEnable = true;   // Enable humidity module

/* Sensor reconditioning:
   Heats the sensor to reconditioningTemperature for ~1 min to remove impurities.
   Recommended together with zero-humidity calibration. Vaisala does the same
   during their ground check. */
constexpr bool reconditioningEnabled      = false;
constexpr uint8_t reconditioningTemperature = 140; // °C

/* Zero-humidity calibration:
   Heats the sensor above 100 °C to evaporate all moisture → known 0 %RH reference.
   Safe - Vaisala firmware performs the same procedure at launch.
   NFW uses PID control for stable temperature during the process.

   Methods (choose one):
     1. Enable zeroHumidityCalibration - automatic on each startup.
        If the sonde will be powered on outdoors in winter, use method 2 instead.
     2. Read the zeroHumidityCapacitance from Ground Software / serial after
        calibration, then disable zeroHumidityCalibration and enter the value here.
        The sonde will then skip calibration on subsequent startups.

   Environment: indoors, 5-40 °C, 0-60 %RH, still air. */
bool  zeroHumidityCalibration = true;
float zeroHumidityCapacitance = 0;   // 0 %RH capacitance reading from calibration (leave 0 for auto-cal)

/* humidityCapacitanceRangeDelta - delta between 0 %RH and 100 %RH capacitance.
   The default is an empirical average from many sensor booms.
   For best accuracy:
     Enable humidityCalibrationDebug, expose sensor to ~100 %RH (above boiling water,
     sensor pointing upward, clear of condensation), and record the peak delta value.
     Enter it below and disable humidityCalibrationDebug. */
float humidityCapacitanceRangeDelta  = 6;
bool  humidityCalibrationDebug       = false;
constexpr unsigned long humidityCalibrationTimeout              = 300000; // Max calibration time (ms); default 5 min
constexpr uint8_t       humidityCalibrationMeasurementTemperature = 125;  // Min temp for cal measurement (°C)


/* ============================================================
   SECTION 18 - PRESSURE

   pressureMode:
     0 - Pressure disabled (reported as 0)
     1 - Vaisala RPM411 BARO-CAP sensor (RS41-SGP required; highly recommended)
         Plug the RPM411 into the rear connector. No calibration needed.
     2 - Pressure estimation from altitude, temperature and humidity.
         Set seaLevelPressure to the current MSL pressure for your region.
         Order-of-magnitude accuracy only.
   ============================================================ */

constexpr uint8_t pressureMode     = 1;
constexpr float   seaLevelPressure = 1013.25;  // MSL pressure (hPa) - used only in mode 2


/* ============================================================
   SECTION 19 - HEATING

   Reference area heating:
     Maintains ~20 °C on the PCB cutout where reference resistors are located.
     Improves temperature reading accuracy at the cost of additional current.
     Recommended for flights ≤18 h with 2×AA batteries.
     Disable with 1×AA batteries or when the sensor boom is not installed.

     referenceAreaTargetTemperature guidelines:
       18 °C - with the original Vaisala styrofoam housing
        8 °C - with improvised wind cover (tape / foil)
        0 °C - bare PCB (very power-hungry - not recommended)

   Humidity module heating:
     Keeps the sensor above humicapMinimumTemperature and defrostingOffset K
     above the air temperature to prevent frosting and condensation.
     Recommended for all flights with the sensor boom installed (≤18 h).
   ============================================================ */

bool          referenceHeating                          = true;
constexpr int8_t referenceAreaTargetTemperature         = 18;  // Target temperature (°C)

bool          humidityModuleHeating                     = true;
constexpr int8_t defrostingOffset                       = 5;   // K above air temp to prevent frost
constexpr int8_t humicapMinimumTemperature              = -44; // °C - humicap accuracy degrades below this
constexpr int8_t humidityModuleHeatingTemperatureThreshold = 35; // Heating activates below this °C


/* ============================================================
   SECTION 20 - LOW-ALTITUDE FAST-TX MODE

   After burst detection, once the sonde descends below
   lowAltitudeFastTxThreshold, it switches to maximum-rate
   Horus + APRS transmissions for lowAltitudeFastTxDuration ms
   to maximise the number of position fixes available for recovery.

   Set lowAltitudeFastTxThreshold = 0 to disable this mode.
   ============================================================ */

constexpr uint16_t      lowAltitudeFastTxThreshold = 1000;    // Altitude threshold (m)
constexpr unsigned long lowAltitudeFastTxDuration  = 480000;  // Active duration (ms); default 8 min
constexpr uint16_t      lowAltitudeFastTxInterval  = 1;       // Inter-TX delay (ms); 1 = as fast as possible


/* ============================================================
   SECTION 21 - FLIGHT COMPUTING
   ============================================================ */

constexpr uint16_t flightStartClimbThreshold = 150; // Climb (m) above the launch baseline that marks the start of flight.
                                                    // Detection is relative to the altitude at the first GPS fix, so it
                                                    // works at any launch elevation and for any ascent rate (incl. slow
                                                    // solar / zero-pressure balloons). Replaces flightDetectionAltitude.
constexpr uint16_t burstDetectionThreshold = 800; // Altitude drop below maxAlt confirming burst (m)


/* ============================================================
   SECTION 22 - SYSTEM SETTINGS
   ============================================================ */

bool autoResetEnable = true;
// Automatically reset the MCU after SYSTEM_RESET_PERIOD.
// Recommended for long/continuous stationary deployments.

#define SYSTEM_RESET_PERIOD (7UL * 24UL * 60UL * 60UL * 1000UL)
// Auto-reset period - default 7 days in milliseconds.

bool aprsToneCalibrationMode = false;
// Transmits 1200 Hz and 2200 Hz AFSK tones for APRS delay calibration.
// DO NOT enable for actual flights - development/calibration use only.

#define THERMISTOR_R25 10000   // Onboard thermistor resistance at 25 °C (Ω)
#define THERMISTOR_B   3900    // Onboard thermistor Beta coefficient


/* ============================================================
   SECTION 23 - BUTTON

   buttonMode:
     0 - Button disabled
     1 - Brief hold = power off  (recommended for most use cases)
     2 - Cycling menu:
           1 blink  = cancel improvedGpsPerformance
           2 blinks = disable radio PA
           Long hold = power off
         (Release at the desired option - tricky to use in the field)

   NOTE: When flying on solar or 1×AA supply, consider disabling
         the button (mode 0) and shorting its pads to keep it
         always in the pressed state.
         Hold time may vary ~1.5-3 s depending on CPU load.
   ============================================================ */

constexpr int8_t buttonMode = 1;


/* ============================================================
   SECTION 24 - DATA RECORDER

   Sends 3 consecutive Horus V3 packets per interval. GPS, sensor boom
   and RPM411 are refreshed between packets so each carries fresh data.
   ASN.1 hard limit: 4 named fields per packet; 3 packets = 12 fields.
   (APRS data recorder deprecated due to memory issues - V3 only.)

   --- Packet A - GPS diagnostics ---
   hdop     (float, no unit)  - GPS Horizontal Dilution of Precision.
                                Measures fix geometry quality.
                                <1.0 = excellent, 1-2 = good,
                                2-5 = moderate, >5 = poor / unreliable.
   jam      (0 / 1)           - Jamming/spoofing warning flag.
                                0 = no interference detected.
                                1 = GPS reports possible interference;
                                    treat position data with caution.
   resets   (count)           - GPS module reset counter since power-on.
                                0 = stable. Rising value = GPS problems
                                (timeout watchdog firing repeatedly).
   gpspwr   (mode ID)         - GPS power/operation mode.
                                RSM4x2: 0=off  1=max-perf  2=powersave.
                                RSM4x4: 0=off  1=max-perf  2=max-tracking-PS
                                        3=efficient-tracking-PS.

   --- Packet B - Flight statistics ---
   flying   (0 / 1)           - In-flight flag.
                                0 = on ground or not yet determined.
                                1 = ascent detected (speed + altitude
                                    thresholds both exceeded).
   burst    (0 / 1)           - Balloon burst detection flag.
                                0 = ascending or on ground.
                                1 = altitude dropped more than
                                    burstDetectionThreshold below maxAlt.
   hmax     (meters)          - Maximum altitude recorded during flight.
                                Only updated when dataRecorderFlightNoiseFiltering
                                is false or when flying=1.
   vmax     (km/h)            - Maximum horizontal speed recorded.
                                Useful for estimating jet-stream exposure.

   --- Packet C - Thermal / heater diagnostics ---
   radiotemp  (deg C, int)    - Radio module (Si4032) die temperature.
                                Used to monitor PLL lock risk; Si4032
                                PLL can lose lock below ~-40 C.
   rpmtemp    (deg C, int)    - RPM411 pressure sensor internal temperature.
                                Only valid when RPM411 is connected and
                                initialised without error.
   extpwr     (0-255 PWM)     - External heater PWM duty cycle.
                                0 = heater off. 255 = full power.
                                Proportional to heating applied to the
                                humidity sensor boom housing.
   refpwr     (0 / 1 / 2)    - Reference area heater state.
                                0 = off. 1 = low. 2 = high.
                                Controls the reference capacitor heating
                                used for humidity sensor calibration.
   ============================================================ */

#ifdef RSM4x4
bool               dataRecorderEnable              = true;
bool               dataRecorderFlightNoiseFiltering = true;
#else
// dataRecorder is on by default everywhere and transmits on both boards. On the
// RSM4x2 it fits because that build is compiled with LTO (it rides Horus V3 pages).
constexpr bool     dataRecorderEnable              = true;
constexpr bool     dataRecorderFlightNoiseFiltering = true;
#endif
constexpr unsigned int dataRecorderInterval        = 300000; // Interval between bursts (ms); default 5 min
// When true, only data captured during actual flight is included in recordings
// (filters out noisy ground-level or pre-fix measurements).


/* ============================================================
   SECTION 25 - KALMAN FILTER & HEATER PID CONSTANTS
   Carefully tuned defaults. Modify only if you know what you
   are doing. Changing these can destabilise sensor readings
   or heater control loops.
   ============================================================ */

// Kalman filter - pressure:
float pressureKalmanError    = 3;
float pressureKalmanQ        = 2;
float pressureKalmanEst      = 1013.25;
float pressureKalmanErrorEst = 3;      // Initialised to pressureKalmanError

// Kalman filter - humidity:
float humidityKalmanError    = 2;
float humidityKalmanQ        = 5;
float humidityKalmanEst      = 5;
float humidityKalmanErrorEst = 2;      // Initialised to humidityKalmanError

// PID controller - humidity module heater:
float extHeaterProportionalK = 2.5;   // Proportional gain (slightly raised for a touch faster response)
float extHeaterIntegralK     = 0.53;   // Integral gain
float extHeaterDerivativeK   = 0.55;   // Derivative gain (slightly lowered to reduce damping/overshoot kick)


#endif  // NFW_CONFIG_H
