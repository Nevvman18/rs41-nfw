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
 *  Version 69  |  GPL-3.0 (see LICENSING.md)  |  Franek Łada (nevvman, SP5FRA)
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
       5 s and above - GPS-clock synced: transmit every N seconds, aligned to
       the top of the hour. Example: 15 → :00, :15, :30, :45 of each minute.

       0-4 - SIMPLE FAST-TX: setting a data mode (Horus V3/V2, APRS, RTTY,
       Morse) below 5 s switches the whole sonde to a plain fast loop with NO
       GPS-clock sync and no offsets. Each cycle it refreshes the GPS and the
       sensor boom, transmits every enabled mode back-to-back, then waits this
       many seconds. 0 = no wait = as fast as possible. The real rate is still
       bounded by one packet's air-time plus a GPS/sensor read (~4-5 s for a
       Horus V3 packet). The GPS operation mode you chose is not changed. The
       sensor boom is refreshed every cycle, unless its power saving (Section
       15) is on, in which case only every sensorBoomPowerSavingInterval.

       In short: below 5 s the scheduler is oriented for data DENSITY, not
       clock synchronisation. Keep the interval at 5 s or above if you want the
       GPS-clock-synced slots and offsets back (e.g. to interleave several
       sondes on one frequency, or land packets on exact times).

       To DISABLE a mode use its Enable switch above - in fast mode a 0 interval
       means quickest, not off. (A short Pip interval alone does not trigger
       fast mode, as Pip is only a beacon; Pip=0 still disables Pip. Pip is
       still sent each cycle when fast mode is on for another mode.)

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
// lowAltitudeFastTxMode uses the first entry only.

char aprsCall[]   = "N0CALL";       // Your amateur radio callsign - USE UPPERCASE LETTERS ONLY (lowercase may not be decoded correctly; the firmware does not convert case)
String aprsComment = " NFWv74";     // Comment appended to every APRS packet

constexpr char aprsSsid          = 11;       // Callsign SSID
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
     Solid Green           - all OK, GPS fix held, no warnings. Ready / flying normally.
     Solid Orange          - running OK but no GPS fix yet.
     Solid Red             - an error is active (sensor-boom fault, RPM411 fault,
                              calibration error or battery warning).
     Off                   - LEDs disabled, or above ledAutoDisableHeight.

   LED meanings during startup and calibration:
     Solid Red             - hardware initialisation right after power-on (stages 01-04).
     Blinking Orange       - calibration in progress (temperature / reconditioning /
                              zero-humidity). The module is hot (~140 C) - keep clear of the boom.
     Red blinks            - an error during start-up or calibration (e.g. sensor-boom
                              fault or calibration error).
     Green ×5              - boot complete, system started, entering normal operation.
   ============================================================ */

bool           ledStatusEnable      = true;   // Enable status LEDs
constexpr int16_t ledAutoDisableHeight = 1000; // Altitude (m) above which LEDs turn off


/* ============================================================
   SECTION 12 - XDATA PORT / INSTRUMENTS

   Mode 0 - disabled
   Mode 1 - combined log + telemetry (recommended): human-readable [info]/[warn]/[err] lines
            AND periodic compact $NFW telemetry frames readable by Ground Control Software at nfw.flada.ovh
   Mode 3 - OIF411 ozone sonde (9600 bps) - see Section 12b for OIF411 settings
   (Mode 2, the old raw-NMEA GPS bridge, was removed in v68: the GPS now runs
    the binary UBX protocol, so there is no NMEA stream to echo.)
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
   • The Data Recorder is very strongly recommended to log full OIF411 telemetry.
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
   SECTION 14 - GNSS CONFIGURATION

   uBlox receiver operation mode (gpsOperationMode):
     0 - GNSS fully OFF (stationary WX station; set coordinates below)
     1 - Always ON / max performance (highest power, fastest updates)
     2 - NFW Intelligent GNSS Algorithms (default, recommended). On RSM4x4 (M10)
         this is the full adaptive constellation + power-tier manager; on RSM4x2
         (G6010, which has only a power-save nav mode) it switches to power-save
         once satellites are comfortably above ~9, and back to max when scarce.
   ============================================================ */

uint8_t gpsOperationMode = 2;

/* NFW Intelligent GNSS options (RSM4x4 / M10, active in operation mode 2). The
   power figures are rough, relative to continuous max-performance tracking:
   m10ConstellationOptimization - auto-shed constellations by fix strength: drops only
                                  Galileo at a strong fix (keeps GPS + BeiDou/GLONASS + SBAS),
                                  a modest ~5% GNSS-power saving. Off by default.
   m10AggressiveOpt             - maximise power saving (also drops the secondary GNSS at a
                                  moderate fix and shortens duty cycles; may reduce tracking quality)
   m10CyclicTracking            - cyclic power-save tracking once the fix is strong: the
                                  receiver sleeps between fixes (~40-70% less GNSS power,
                                  scaling with the cyclic period).
                                  IMPORTANT - the M10 power-save modes will NOT run with the
                                  BeiDou B1C signal or with SBAS (u-blox M10 integration
                                  manual). So cyclic tracking REQUIRES gpsSecondaryGnss = 1
                                  (BeiDou B1I only) or 2 (GLONASS only) AND gpsSbasEnable =
                                  false. If B1C (gpsSecondaryGnss = 0) or SBAS is on, cyclic
                                  tracking is ignored and the receiver stays continuous - the
                                  firmware checks this (m10CyclicTrackingUsable) so a bad combo
                                  never sends a command the M10 would reject. The Firmware
                                  Builder enforces the same interlock with a warning.
   m10CyclicPeriodSec           - cyclic on/off period in seconds (longer = more saving,
                                  slower position updates)
   m10SuperS                    - u-blox Super-S adaptive power management (extra fine-grained
                                  saving on top of the above) */
constexpr bool     m10ConstellationOptimization = false;
constexpr bool     m10AggressiveOpt             = false;
constexpr bool     m10CyclicTracking            = false;  // OFF by default: the accuracy-focused default
                                                          // GNSS set (B1C + GLONASS + SBAS) is not PSM-legal.
                                                          // To use it, set gpsSecondaryGnss to 1 or 2 and gpsSbasEnable false.
constexpr uint8_t  m10CyclicPeriodSec           = 10;   // cyclic-tracking period (s)
constexpr bool     m10SuperS                    = true;

/* Airborne 1G dynamic model - required for tracking above 18 km altitude.
   Set false only for stationary use below 18 km (marginally better accuracy). */
constexpr bool ubloxGpsAirborneMode = true;

/* ---- Native UBX data path (v68: the GPS is driven entirely in binary UBX,
   NMEA is disabled). These control the UBX solution stream. ---- */

/* GPS solution / output rate in Hz (1, 2 or 4). The firmware reads the newest
   streamed fix the instant it arrives (no fixed ~1 s wait like the old NMEA
   path). 2 Hz is a good responsiveness/power balance; 4 Hz is heaviest and is
   near the u-blox 6 limit. */
constexpr uint8_t gpsUpdateRateHz = 2;

/* Dynamic model, applied when ubloxGpsAirborneMode is true (that switch is just
   the master "set a custom dynamic model" enable; this picks which one):
     0 = Portable      2 = Stationary    3 = Pedestrian   4 = Automotive
     5 = Sea           6 = Airborne <1g  7 = Airborne <2g 8 = Airborne <4g
   6 (Airborne <1g) is the default and the right choice for a high-altitude
   balloon; 8 (<4g) tolerates higher accelerations; 2 (Stationary) suits a fixed
   WX station. */
constexpr uint8_t gpsDynamicModel = 6;

/* Secondary GNSS (RSM4x4 / M10 only). GPS + Galileo + SBAS always run; this picks what
   else the single-band M10 tracks. gpsSecondaryGnss:
     0 - BeiDou (B1C) + GLONASS. BeiDou's modern B1C signal shares the 1575.42 MHz L1
         centre with GPS/Galileo, so the single-band receiver CAN process it together with
         GLONASS - the most satellites (GPS + Galileo + BeiDou-3 + GLONASS). Default.
     1 - BeiDou (B1I) only. The older B1I signal is on its own frequency (1561.098 MHz),
         away from the crowded 1575 MHz L1 band, so it is more resilient to interference or
         jamming centred on L1 - but it cannot run alongside GLONASS on this receiver, and
         it tracks fewer constellations overall.
     2 - GLONASS only.
   Note: B1I (mode 1) cannot coexist with GLONASS because it needs a separate RF path at
   1561 MHz; using BeiDou's B1C signal instead is what makes mode 0 possible.
   Power-save note: BeiDou B1C is NOT allowed in the M10 power-save modes, so mode 0 is
   incompatible with m10CyclicTracking - use mode 1 (B1I) or 2 (GLONASS) for cyclic tracking
   (and turn gpsSbasEnable off).
   QZSS is a separate regional system whose satellites are only visible over Japan, East/
   South-East Asia, Australia and the west Pacific - off by default, turn it on only if you
   launch inside that region. The older u-blox 6 boards ignore all of this. */
constexpr uint8_t gpsSecondaryGnss = 0;  // 0 = BeiDou B1C + GLONASS, 1 = BeiDou B1I only, 2 = GLONASS only
constexpr bool gpsQzssEnable = false;  // regional (Asia-Pacific only); no effect elsewhere

/* Extra u-blox options (v68). Each is checked against the receiver's ACK; a
   rejected one is logged (gpsCfgNakCount) but never blocks operation. */
constexpr bool gpsSbasEnable             = true;  // SBAS (EGNOS/WAAS/MSAS): differential corrections plus an extra ranging source.
                                                  // Must be false to use m10CyclicTracking (the M10 cannot process SBAS in power-save mode).
constexpr bool gpsAssistNowAutonomous    = true;  // predict orbits on-board -> much faster re-acquisition after signal loss
constexpr bool gpsSpoofingDetection      = true;  // poll UBX-NAV-STATUS spoofing flags (surfaced in telemetry / integrity warning)
constexpr bool gpsHardwareJammingMonitor = true;  // poll the 0..255 CW jamming indicator from UBX-MON-RF (M10) /
                                                  // MON-HW (u-blox 6). This is a raw interference level only; there is
                                                  // no jamming state on these modules.

/* Satellites engine profile - satellite acceptance vs power. Sets the receiver's
   minimum-elevation and C/N0 (signal-strength) masks. More sensitive settings
   keep weaker and lower satellites for better availability/accuracy at the cost
   of a little more power.
     0 = Max sensitivity     (0 deg elevation, low C/N0 - fight for every satellite) - default
     1 = Balanced            (4 deg elevation, moderate C/N0)
     2 = Ultra power saving  (8 deg elevation, high C/N0) */
constexpr uint8_t gpsTrackingProfile = 0;

/* GPS timeout watchdog - resets the GPS module if no valid fix for this many ms.
   0 = disabled. Recommended: 1200000 ms (20 min). */
unsigned long gpsTimeoutWatchdog = 1200000;

/* Improved GPS fix performance:
   While GPS has fewer than 4 satellites, the Si4032 radio is silenced for
   radioSilenceDuration ms. Si4032 produces wideband spurious emissions that
   desensitise the GPS receiver; silencing it dramatically speeds up cold starts.
   disableGpsImprovementInFlight stops the silence during flight (avoids 2 min
   data gaps). If flying in high-interference areas, set this false instead. */
constexpr bool improvedGpsPerformance        = true;
constexpr bool disableGpsImprovementInFlight = true;
unsigned long radioSilenceDuration          = 120000; // Radio-silence window (ms)
// After a fix is acquired in radio-quiet mode, stay quiet this much longer before letting
// the radio resume, so the fix can settle and gather more satellites before a transmission
// (which briefly desensitises the receiver) hits it. Reset if the fix is lost again; still
// capped by radioSilenceDuration. 0 = leave immediately on fix (old behaviour).
unsigned long improvedGpsHoldAfterFix        = 20000; // Extra quiet hold after a fix (ms)

/* Simultaneous GNSS setup:
   Controls WHEN the GPS is powered up and configured during boot, independently of the
   radio-quiet improvement above.
     true  (default) - the GPS is brought up at the very start, so it searches for
                       satellites in parallel with the hardware setup and the sensor-boom
                       calibration. With the boom fitted, calibration takes a while and
                       the receiver uses that time to get a head start on its first fix,
                       so the sonde reaches a valid position much sooner.
     false           - the GPS is held on reset through setup and calibration and only
                       powered up afterwards. Its RF then cannot disturb the sensitive
                       ring-oscillator temperature/humidity measurements, at the cost of a
                       slower first fix.
   This is the only thing that decides GPS start timing; improvedGpsPerformance no longer
   affects it. */
constexpr bool simultaneousGnssSetup         = true;

/* Static coordinates - used when gpsOperationMode = 0, overwritten once GPS active. */
float gpsLat  = 0;   // Latitude  (decimal degrees)
float gpsLong = 0;   // Longitude (decimal degrees)
float gpsAlt  = 0;   // Altitude  (m)

constexpr uint8_t gpsSatsWarnValue     = 4;      // Satellite count below which a warning is shown
unsigned long     gpsPowerSaveDebounce = 60000;  // Min interval before climbing to a lighter GPS tier (ms). Downgrades to a harder-tracking tier ignore this and apply at once.


/* ============================================================
   SECTION 15 - SENSOR BOOM
   ============================================================ */

bool sensorBoomEnable      = true;   // Enable sensor boom measurements and diagnostics
constexpr bool sensorBoomPowerSaving = false;  // Power saving: read the boom only every sensorBoomPowerSavingInterval
unsigned long sensorBoomPowerSavingInterval = 30000; // Boom read interval in power-saving mode (ms), default 30000 = 30 s


/* ============================================================
   SECTION 15b - SENSOR CALIBRATION MODE

   Selects how temperature and humidity are computed. The switch governs the
   WHOLE measurement chain (main temp, heater temp, humidity) - nothing is
   mixed between modes.

     1 - NFW calibration. The in-house calibration and compensation algorithms
         (computed from reference-resistor ratio + PT1000 curve, capacitance + zero-humidity
         range and passed through extensive correction scripts). Works on every board.

     2 - Factory (Vaisala) calibration (default). Uses the sonde's original
         factory coefficients, reproducing the Vaisala / rs1729 conversion for
         full factory accuracy. The coefficients are sonde-specific: the Sounding
         Software fetches them per serial number from SondeHub and pastes them
         in below.

   BOARD SUPPORT:
     - RSM4x4 / RSM4x5 (newer boards): both modes are available; this setting
       chooses between them (default 2 = factory / Vaisala).
     - RSM4x2 / RSM4x1 (older boards): factory (Vaisala) calibration ONLY. The NFW
       calibration path is no longer built for the F100; the factory chain now fits
       because its maths is single-precision (no double-precision libm), so this
       setting is effectively fixed at 2 there.

   NOTE: Leave the coefficients at 0 when unused - the Sounding Software fills them
   when you load a serial number.
   ============================================================ */
#if defined(RSM4x4) || defined(RSM4x2)
uint8_t sensorCalibrationMode = 2;   // 1 = NFW, 2 = factory (Vaisala). RSM4x4/4x5: selectable. RSM4x2/4x1: factory only.

// --- Factory calibration coefficients (filled per-serial from SondeHub) ---
// Reference resistors (Ohm) / capacitors (pF):
float factoryRefResistorLow   = 750;    // refResistorLow  (low ref, getSensorBoomFreq(1))
float factoryRefResistorHigh  = 1100;   // refResistorHigh (high ref, getSensorBoomFreq(2))
float factoryRefCapLow        = 0;      // refCapLow  (0 pF, getSensorBoomFreq(7))
float factoryRefCapHigh       = 47;     // refCapHigh (47 pF, getSensorBoomFreq(6))
// Main temperature polynomial: taylorT[0..2], calT, polyT[0..1]
float factoryTaylorT0 = 0;
float factoryTaylorT1 = 0;
float factoryTaylorT2 = 0;
float factoryCalT     = 0;
float factoryPolyT0   = 0;
float factoryPolyT1   = 0;
// Heater / RH-sensor temperature polynomial: taylorTU[0..2], calTU, polyTrh[0..1]
float factoryTaylorTU0 = 0;
float factoryTaylorTU1 = 0;
float factoryTaylorTU2 = 0;
float factoryCalTU     = 0;
float factoryPolyTrh0  = 0;
float factoryPolyTrh1  = 0;
// Humidity calibration: calibU[0..1] (capacitance normalisation)
float factoryCalibU0 = 0;
float factoryCalibU1 = 0;
// Humidity calibration matrix matrixU (7 rows x 6 cols, row-major). Filled per-boom serial.
const float factoryMatrixU[42] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* --- Factory-mode startup self-checks (mode 2 only) ---
   These replace the NFW startup calibrations (which are disabled in factory mode,
   because the Vaisala coefficients are absolute and need no per-flight tuning).
   They do NOT alter any reading - they are only for diagnostics and to ensure the best accuracy and repeatability. Each is simply on/off; the procedure itself is fixed (no tuning).

   factoryTemperatureCheck (runs first):
       One sensor-boom measurement; verifies the main temperature and the heater
       temperature agree within 3 °C. A larger gap flags a temperature-check error.

   factoryHumidityCheck (runs after the temperature check):
       Reconditions the humidity sensor (heats it to ~140 °C for one minute), then,
       while holding ~140 °C, reads humidity. A bone-dry sensor must read < 5 %RH; a
       higher reading, or failure to exceed 115 °C within the minute, flags an error. */
bool factoryTemperatureCheck = true;
bool factoryHumidityCheck    = true;
#endif  // RSM4x4 factory-calibration block (Section 15b)


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
     0 - Pressure disabled (the field is left out of Horus v3 frames entirely)
     1 - Vaisala RPM411 BARO-CAP sensor (RS41-SGP required; highly recommended)
         Plug the RPM411 into the rear connector. No calibration needed.
     2 - Pressure estimation from altitude (ISA barometric model). Works on both boards.
         Set seaLevelPressure to the current MSL pressure for your region.
         Order-of-magnitude accuracy only - use mode 1 (RPM411) when you have the sensor.
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

   Heaters power optimisation:
     Acts only during descent, after burst. Falling fast through thin air the
     airflow strips heat away faster than the heaters can supply it, so holding
     the full targets there only wastes battery:
       descent above 18 m/s - reference target lowered by 6 °C, humidity module heater off
       descent above 14 m/s - reference target lowered by 3 °C
     At slower descent rates heating is not altered at all, and normal heating
     resumes automatically as the fall slows down. Never changes anything on ascent.
   ============================================================ */

bool          referenceHeating                          = true;
constexpr int8_t referenceAreaTargetTemperature         = 18;  // Target temperature (°C)

bool          humidityModuleHeating                     = true;
constexpr int8_t defrostingOffset                       = 5;   // K above air temp to prevent frost
constexpr int8_t humicapMinimumTemperature              = -40; // °C - humicap accuracy degrades below this
constexpr int8_t humidityModuleHeatingTemperatureThreshold = 45; // Heating activates below this °C (upper cap on module heating)

bool          heatersPowerOptimisation                  = true; // Lower heating targets during the fast fall after burst (see above)


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
   SECTION 20b - PRIVATE LANDING MODE   (RSM4x4 / RSM4x5 only)

   NOT RECOMMENDED - please read before enabling.

   This hides the landing spot from public tracking maps by moving every
   enabled mode onto a separate, private frequency once the sonde is coming
   down near the ground. Trackers following the public frequencies simply
   lose the payload after it drops below the threshold.

   A far better approach for almost everyone: clearly label the payload as
   harmless research equipment with a short "reward if returned" note and
   your contact details. Finders then return it to you, which is easier and
   friendlier than hiding. Reserve private landing for the rare, marginal
   situations where that genuinely is not enough.

   How it works: after the sonde has flown (beganFlying), climbed above
   privateLandingAltitudeThreshold and then descended back below it, every
   enabled mode stops using its normal frequency (or frequency table) and
   transmits only on its single private frequency below. It stays that way
   until power-off. Because it must climb above the threshold first, it never
   triggers during a low-altitude ascent.

   Each mode has exactly ONE private frequency here (no frequency tables).
   ============================================================ */

#ifdef RSM4x4
constexpr bool     privateLandingModeEnable        = false;  // Master enable (NOT recommended - read above); default off
constexpr uint16_t privateLandingAltitudeThreshold = 3500;   // Switch to private freqs after descending below this altitude (m)

constexpr float pipPrivateFreq     = 432.7;  // Pip private frequency (MHz)
constexpr float horusV3PrivateFreq = 437.6;  // Horus Binary V3 private frequency (MHz)
constexpr float horusPrivateFreq   = 437.6;  // Horus Binary V2 private frequency (MHz)
constexpr float aprsPrivateFreq    = 432.5;  // APRS private frequency (MHz)
constexpr float rttyPrivateFreq    = 434.6;  // RTTY private frequency (MHz)
constexpr float morsePrivateFreq   = 434.6;  // Morse private frequency (MHz)
#endif


/* ============================================================
   SECTION 21 - FLIGHT COMPUTING
   ============================================================ */

constexpr uint16_t flightStartClimbThreshold = 300; // Climb (m) above the launch baseline that marks the start of flight.
                                                    // Detection is relative to a launch baseline captured after the GPS
                                                    // altitude has settled (see flightBaselineSettleTime), so it works at
                                                    // any launch elevation and for any ascent rate (incl. slow solar /
                                                    // zero-pressure balloons). Replaces previous flightDetectionAltitude.
constexpr uint16_t flightBaselineSettleTime = 15000; // Time (ms) to wait after the first GPS fix before latching the launch
                                                     // baseline, so a cold-start altitude (often 100-200 m off) can converge.
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

constexpr char aprsDest[] = "APRNFW";
// APRS AX.25 destination address / tocall. APRNFW is the OFFICIAL registered tocall for
// this firmware - it is recognised by the APRS network and identifies the device as
// running RS41-NFW. This is a system value: do not change it (which is why it is not
// exposed).

#define THERMISTOR_R25 10000   // Onboard thermistor resistance at 25 °C (Ω)
#define THERMISTOR_B   3900    // Onboard thermistor Beta coefficient

/* MCU internal temperature sensor (both RSM4x2 and RSM4x4).
   cpuTempSensorVoltageAt25degC is the temperature-sensor output voltage at 25 °C.
   It is fairly repeatable across units, so the default 1.424 V is fine to leave as-is.
   It can optionally be loaded per-serial from the factory subframe
   (cpuTempSensorVoltageAt25deg) for a touch more accuracy, but this is not required. */
float cpuTempSensorVoltageAt25degC = 1.424;

// Measurement-boom serial number. The Sounding Software fills this with the serial you
// enter for factory (Vaisala) mode; it is reported in the $NFW telemetry so Ground
// Control can show which boom's calibration the firmware was built with. Leave empty
// otherwise (NFW mode does not need it).
String boomSerialNumber = "";


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

constexpr int8_t buttonMode = 0;


/* ============================================================
   SECTION 24 - DATA RECORDER

   Sends 3 consecutive Horus V3 packets per interval. GPS, sensor boom
   and RPM411 are refreshed between packets so each carries fresh data.
   ASN.1 hard limit: 4 named fields per packet; 3 packets = 12 fields.
   (APRS data recorder deprecated due to memory issues - V3 only.)

   --- Packet A - GPS diagnostics ---
   hdop     (float, no unit)  - GPS Dilution of Precision. Since v68 (native UBX)
                                this field carries pDOP (position DOP) from
                                NAV-PVT/NAV-SOL - it measures overall 3D fix
                                geometry (always >= the old hDOP).
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
   are doing. Even slight changes of these can easily destabilise
   sensor readings or heater control loops.
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
float extHeaterProportionalK = 2.48;   // Proportional gain
float extHeaterIntegralK     = 0.6;  // Integral gain
float extHeaterDerivativeK   = 0.49;  // Derivative gain

// You do not edit this.
/* Helper used across the sketch: true when the factory (Vaisala) chain is active.
   In factory mode the NFW startup calibrations (temperature offset / zero-humidity)
   are skipped, because the factory polynomials are absolute - applying an NFW
   offset on top of them would corrupt the readings.*/
#if defined(RSM4x4)
// RSM4x4 / RSM4x5: the mode is user-selectable (1 = NFW, 2 = factory).
#define FACTORY_CAL_ACTIVE (sensorCalibrationMode == 2)
#elif defined(RSM4x2)
// RSM4x2 / RSM4x1: factory (Vaisala) calibration only - the NFW calibration chain is
// no longer built here, so the factory path is always active.
#define FACTORY_CAL_ACTIVE (1)
#else
#define FACTORY_CAL_ACTIVE (0)
#endif


#endif  // NFW_CONFIG_H
