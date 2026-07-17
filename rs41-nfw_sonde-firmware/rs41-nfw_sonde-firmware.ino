/*
RS41-NFW - versatile, feature-rich and user-friendly custom firmware for ALL revisions of Vaisala RS41 radiosondes
Authors: Franek Łada (nevvman, SP5FRA)

Version 69 (public, stable)

All code and dependencies used or modified here that don't origin from me are described in code comments and repo details.
https://github.com/Nevvman18/rs41-nfw

-------------------------------------------------------------------------------
 LICENSING (see LICENSING.md for the full text and the authoritative file list)

 The firmware is licensed GNU GPL-3.0, WITH one addition by the author:
 a linking / combination exception (GPL-3.0 sec. 7) that permits combining the
 GPL code with the Source-Available Modules described below and distributing the
 resulting firmware binary.

 Source-Available Modules - NOT licensed under the GPL. These cover the sensor
 boom acquisition/calibration, the sensor-boom / humidity-module heating control,
 and the GPS readout / NFW intelligent-GPS code. They are marked in-source with an
 "[RS41-NFW-SA]" banner directly above each function, and are released under the
 RS41-NFW Source-Available License (LICENSE.source-available): you may read, build
 and use them for personal/non-commercial purposes, but not sell them or
 redistribute modified closed versions.

 RS41-NFW is inspired by the RS41ng project but does not reuse code originating
 from it; the hardware sequences are implemented from the manufacturer datasheets.
 gps.h/gps.cpp is a native u-blox UBX driver written for RS41-NFW (it replaced
 the former TinyGPS++ NMEA parser). Bundled third-party code keeps its own
 license: horus_l2.* contains Golay code (R. Morelos-Zaragoza).
-------------------------------------------------------------------------------
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
#define NFW_VERSION "RS41-NFW v70, GPL-3.0 Franek Lada (nevvman, SP5FRA)"  //This is the firmware version You are running

//===== Libraries and lib-dependant definitions (nothing to modify)
/* No libraries are required to be installed, all dependencies are shipped within the project folder. */
#include "horus_l2.h"
//#include "horus_l2.cpp"
#include <SPI.h>
#include "gps.h"
#include "HorusBinaryV3.h"

UbxGnss gps;

#include <HardwareTimer.h>
#include <new>   // placement new - build the big Horus V3 ASN.1 struct directly into g_txScratch

// Shared transmit scratch RAM. On the tight 8 KB F100 there is not room for all of these at
// once, but they are never live at the same time - each belongs to a different, non-overlapping
// step: the $NFW telemetry frame (interfaceHandler), the two APRS message buffers (aprsTx, and
// only one APRS mode is built per packet), and the >1 KB Horus V3 ASN.1 packet (buildHorusV3Packet
// / the data recorder). Overlapping them in a union keeps the big Horus struct OFF the stack -
// on the stack it overflowed into the heap and corrupted the humidity-heater PWM timer handle,
// hard-faulting the sonde after the first transmission - without needing RAM the F100 lacks.
union NfwTxScratch {
  char           nfwFrame[1280];   // $NFW telemetry frame (interfaceHandler). Headroom over 1 KB for the
                                   // v68 GPS-diagnostics fields; NW_S writes without a bounds check.
  char           aprsOthers[256];  // APRS HAB comment (aprsTx, mode 1)
  char           aprsWx[256];      // APRS WX report  (aprsTx, mode 2)
  char           dbgHex[256];      // Horus debug hex dump (written only after the packet is encoded)
  horusTelemetry horusMsg;         // Horus V3 ASN.1 packet (buildHorusV3Packet / data recorder)
};
NfwTxScratch g_txScratch;

/*
 * ============================================================
 *  RS41-NFW - CONFIGURATION
 * ============================================================
 *  ALL user-configurable settings are located in CONFIG.h.
 *  Open CONFIG.h to configure the firmware:
 *    • Board revision selection (RSM4x4 or RSM4x2)
 *    • TX timing and protocol settings (Horus, APRS, RTTY, …)
 *    • GPS mode, sensor boom, heating, power management
 *    • System, button, data-recorder, and calibration options
 *
 *  DO NOT edit configuration values in this .ino file.
 *  The hardware pin and interface definitions that follow are
 *  fixed per board revision and are resolved automatically
 *  from the revision selected in CONFIG.h.
 * ============================================================
 */
#include "CONFIG.h"

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
bool err = false;   //red light, error status state
bool ok = true;     //green light, ok status state
bool vBatWarn = false;
bool gpsFixWarn = false;
int horusPacketCount;
int horusV3PacketCount;
uint8_t xdataInstrumentType = 0;
int xdataInstrumentNumber = 0;
float xdataOzonePumpTemperature = 0;  // [°C]
float xdataOzoneCurrent = 0;           // [μA]
float xdataOzoneBatteryVoltage = 0;    // [V]
float xdataOzonePumpCurrent = 0;       // [mA]
float xdataOzoneExtVoltage = 0;        // external voltage [V]
char  xdataOzoneSerial[9] = {0};       // 8-char instrument serial
uint16_t xdataOzoneDiagnostics = 0xFFFF; // 0x0000=OK, 0x0004=pump T cold, 0x0400=batt off
uint8_t  xdataOzoneFwVersion = 0;      // firmware version integer (e.g. 10 = v0.10)
float xdataOzonePartialPressure = 0;   // P3 partial pressure of ozone [mPa]
float xdataOzonePpb = 0;              // O3 volume mixing ratio [ppbv]
float ozone_P0 = 0;                   // ground-level pressure at first GPS fix [hPa]
bool  ozone_P0Set = false;
uint8_t ozone_P0SampleCount = 0;
float ozone_P0Accumulator = 0;
unsigned long ozoneLastFrameMs = 0;   // millis() of last valid OIF411 parsed frame
bool ozoneConnectionError = false;    // true when no valid frame for > 3 s (after first frame)
char    ozoneDbgRaw[64] = "";         // last raw xdata= frame content (for debug)
uint8_t ozoneDbgLen     = 0;          // length of last received raw frame
uint32_t ozoneRxByteTotal = 0;        // total bytes received on xdata RX port
uint16_t ozoneFrameTotal = 0;         // count of successfully parsed OIF411 frames
float lastGpsAlt;
unsigned long lastGpsAltMillisTime = 1;
float vVCalc;
bool gpsAltFresh = false;             // true for the one cycle a new fixed altitude was parsed
bool gpsTimeoutCounterActive = false;
unsigned long gpsTimerBegin = 0;
int currentGPSPowerMode = 0;  // RSM4x2/4x1 GPS power state (sent as Horus "gpspwr"): 0 not set, 1 max-performance/continuous, 2 power-save
unsigned long lastPowerSaveChange = 0;   // millis() of the last GPS power-mode/tier change (debounce, see gpsPowerSaveDebounce)
bool gpsPowerModeInitialized = false;    // false until the first power-mode/tier decision has been applied
unsigned int rttyFrameCounter = 0;
unsigned long lowAltitudeFastTxModeBeginTime = 0;
unsigned int gpsResetCounter = 0;
uint16_t gpsCfgNakCount = 0;   // GPS config messages that failed to ACK (used by the data recorder + diagnostics)
unsigned long lastDataRecorderTransmission = 0;
unsigned long landingTimeMillis = 0;
bool cancelGpsImprovement = false;
bool gpsJamWarning = false;
int8_t currentRadioPwrSetting = 0;
int8_t currentM10IntelligentMode = 1;  // RSM4x4/4x5 M10 intelligent tier (sent as Horus "gpspwr", mode 3): 1 weak fix (<=10 sats) continuous, 2 moderate (11-15) cyclic/continuous, 3 strong (>=15) cyclic power-save
int8_t gpsStatus = 1;  // value sent as Horus V3 extra-sensor "gpspwr": = currentM10IntelligentMode on RSM4x4/4x5, = currentGPSPowerMode on RSM4x2/4x1

uint16_t maxAlt = 0;
int16_t maxSpeed = 0;
char nfwCurrentStage[4] = "00";
int maxAscentRate = 0;
int maxDescentRate = 0;
int8_t maxMainTemperature = 0;
int8_t minMainTemperature = 0;
int8_t maxInternalTemp = 0;
int8_t minInternalTemp = 0;
bool beganFlying = false;
bool burstDetected = false;
// Flight-start detection state (cumulative climb above launch baseline).
float         flightBaseAlt = 0.0f;         // launch-baseline altitude, captured once the fix has settled
bool          flightBaselineSet = false;
float         flightPrevAlt = 0.0f;         // last fix altitude, to count only new fixes
uint8_t       flightSustainedRise = 0;      // consecutive fixes >= climb threshold (noise rejection)
unsigned long flightFixAcquiredMillis = 0;  // millis() of the first GPS fix, to time baseline settling
bool recorderInitialized = false;  //not init by default
bool hasLanded = false;
bool lowAltitudeFastTxModeEnd = false;
#ifdef RSM4x4
// Private landing mode (RSM4x4/4x5 only): latches true once the sonde has flown,
// climbed above privateLandingAltitudeThreshold and then descended back below it.
// While set, every enabled mode transmits only on its private frequency. It never
// clears until power-off. privateLandingArmed records that the sonde has been above
// the threshold in flight (independent of maxAlt / noise filtering).
bool privateLandingActive = false;
bool privateLandingArmed  = false;
#endif

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
bool _hrdStopRequested = false;
bool calibrationError = false;
int extHeaterPwmStatus = 0;
int referenceHeaterStatus = 0;


// APRS - misc.
bool aprsTone = 0;
char statusMessage[320];  // Status message
char aprsLocationMsg[32];
char* const aprsOthersMsg = g_txScratch.aprsOthers;   // shares g_txScratch (not live during Horus TX)
char* const aprsWxMsg     = g_txScratch.aprsWx;
char aprsBitStuffingCounter = 0;  // Bit stuffing counter
unsigned short aprsCrc = 0xffff;  // CRC for error checking
unsigned int aprsPacketNum = 0;

// ===== SCHEDULER STATE =====
// Managed by schedulerInit() / schedulerLoop(). Do not access directly from other code.

unsigned long sch_sysMs    = 0;   // UTC time-of-day in ms (GPS-synced when available)
unsigned long sch_lastMillis = 0; // Previous millis() snapshot used for time tracking
bool          sch_gpsSynced  = false; // True when GPS time is valid and locked
bool          sch_everSeeded = false; // True once the clock has been seeded from GPS at least once (stays set across GPS losses)

// Next scheduled TX times in sch_sysMs units (ms since UTC midnight).
// Value 0 = uninitialized → compute first slot on the next scheduler pass.
unsigned long sch_nextPipMs     = 0;
unsigned long sch_nextHorusV3Ms = 0;
unsigned long sch_nextHorusMs   = 0;
unsigned long sch_nextAprsMs    = 0;
unsigned long sch_nextRttyMs    = 0;
unsigned long sch_nextMorseMs   = 0;

// Time (millis(), NOT sch_sysMs) each mode last actually transmitted, indexed
// 0=pip 1=horusV3 2=horus 3=aprs 4=rtty 5=morse. millis() is used on purpose: it never
// jumps, so it stays valid across a GPS clock re-alignment, which is exactly the case
// that could otherwise reschedule a mode onto a slot just after it already transmitted
// and make it fire twice within a few seconds. 0 = has not transmitted yet.
unsigned long sch_lastTxHw[6] = {0, 0, 0, 0, 0, 0};

// Sensor-update freshness - millis()-based (immune to GPS clock corrections)
unsigned long sch_lastSensorBoom = 0;
unsigned long sch_lastPressure   = 0;
unsigned long sch_lastInterface  = 0;
unsigned long sch_lastGps        = 0;
unsigned long sch_lastOzone      = 0;

// Simple fast-TX mode. When any enabled DATA mode (Horus V3/V2, APRS, RTTY, Morse) has its
// interval set below FAST_TX_MIN_SYNC_SECONDS, the sonde drops GPS-clock scheduling for a
// plain loop: refresh GPS + sensor boom, transmit every enabled mode back-to-back, then
// wait fastTxDelayMs (0 = no wait = quickest). No slot alignment or per-mode offsets apply.
// The sensor boom is refreshed every cycle unless its own power saving is on (then only
// every sensorBoomPowerSavingInterval). Computed once in schedulerInit(); when no interval
// is that short the flag is false and the normal GPS-clock scheduler runs unchanged.
#define FAST_TX_MIN_SYNC_SECONDS 5   // intervals below this (0-4) select fast mode
bool          fastTxMode    = false;
unsigned long fastTxDelayMs = 0;

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

// CFG-PM power modes are now built on the fly (see m10SetContinuous / m10SetPsmct), which is
// why the old pre-baked PSMCT/continuous VALSET frames were removed.

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
#ifdef RSM4x4
// Horus v2 Mode 1 (32-byte) Binary Packet
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
  int16_t dummy1;
  int16_t dummy2;
  uint8_t dummy3;
  uint16_t dummy4;
  uint16_t dummy5;
  uint16_t Checksum;  // CRC16-CCITT Checksum.
} __attribute__((packed));
#endif

// Buffers and counters.
// QI - Horus v2 - 32 bytes uncoded -> 65 bytes coded.
// QI - Horus v3 48 bytes -> 94 bytes coded
#define HORUS_UNCODED_BUFFER_SIZE 128
#define HORUS_CODED_BUFFER_SIZE 256
char rawbuffer[HORUS_UNCODED_BUFFER_SIZE];    // Buffer to temporarily store a raw binary packet.
// QI - Expanded to 256 bytes to fit the big 128 byte (Coded) v3 packet
char codedbuffer[HORUS_CODED_BUFFER_SIZE];  // Buffer to store an encoded binary packet
char* const debugbuffer = g_txScratch.dbgHex;  // shares g_txScratch (written only after packet encode)

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
bool lastRpm411ErrorState = false;  // tracks previous RPM411 state so the status is logged only on change (avoids [info] spam)
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

void setRadioFrequency(const float frequency_mhz) {  // Si4032 PLL setup per the manufacturer datasheet (approach inspired by RS41ng, no code reused)
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

  // Clamp all fields to their ASN.1-schema limits before encoding.
  // Exceeding any limit causes the encoder to return an error and drop the packet.
  const uint8_t  asn_sats      = constrain(gpsSats,                                  0,      31);
  const int32_t  asn_alt       = constrain((int32_t)gpsAlt,                      -1000,   50000);
  const uint16_t asn_speed     = constrain((int32_t)gpsSpeedKph,                     0,     512);
  const uint16_t asn_pressure  = constrain((int32_t)(pressureValue*10),              0,   12000);
  const int16_t  asn_tempExt   = constrain((int32_t)(mainTemperatureValue*10),    -1023,    1023);
  const int16_t  asn_tempCust1 = constrain((int32_t)(extHeaterTemperatureValue*10), -1023,  1023);
  const uint8_t  asn_humidity  = constrain((int32_t)humidityValue,                   0,     100);

  horusTelemetry& asnMessage = g_txScratch.horusMsg;   // shared off-stack instance (see g_txScratch)
  new (&asnMessage) horusTelemetry{
        .payloadCallsign  = HORUS_V3_CALLSIGN,
        .sequenceNumber = horusV3PacketCount,
        .timeOfDaySeconds  = gpsHours*3600 + gpsMinutes*60 + gpsSeconds,
        .latitude = (int)(gpsLat*100000),
        .longitude = (int)(gpsLong*100000),
        .altitudeMeters = asn_alt,
        // Standard-packet extra sensors: only the ozone fields (p3, o3ppb), and
        // only in ozone mode. gpspwr and the other GPS diagnostics now live in the
        // data recorder, not here. extraSensors is omitted entirely otherwise
        // (its exist flag is set from xdataPortMode below).
        .extraSensors = {
          .nCount = 2,
          .arr = {
            { .name = "p3",     .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzonePartialPressure } } } }, .exist = { .name = true, .values = true } },
            { .name = "o3ppb",  .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)xdataOzonePpb } } } },        .exist = { .name = true, .values = true } },
          },
        },
        .velocityHorizontalKilometersPerHour = asn_speed,
        .gnssSatellitesVisible = asn_sats,
        .ascentRateCentimetersPerSecond = vVCalc * 100,
        .pressurehPa_x10 = asn_pressure,
        .temperatureCelsius_x10 = {
            .internal = readAvgIntTemp()*10,
            .external = asn_tempExt,
            .custom1 = asn_tempCust1,
            .custom2 = 0,
            .exist = {
                .internal = true,
                .external = true,
                .custom1 = true,
                .custom2 = false
            }
        },
        .humidityPercentage = asn_humidity,
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
            .extraSensors = (xdataPortMode == 3),   // ozone fields only; omitted in normal mode
            .velocityHorizontalKilometersPerHour = true,
            .gnssSatellitesVisible = true,
            .ascentRateCentimetersPerSecond = true,
            .pressurehPa_x10 = true,
            .temperatureCelsius_x10 = true,
            .humidityPercentage = true,
            .milliVolts = true
        }
    };

    if (!horusV3ExtraSensorsEnable) {
      asnMessage.exist.extraSensors = false;
      asnMessage.temperatureCelsius_x10.exist.custom1 = false;
    }

    if (sensorBoomEnable == false) {
      asnMessage.temperatureCelsius_x10.exist.external = false;
      asnMessage.exist.humidityPercentage = false;
      if (horusV3ExtraSensorsEnable) {
        asnMessage.temperatureCelsius_x10.exist.custom1 = false;
      }
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
                    HORUS_UNCODED_BUFFER_SIZE-2  // buffer starts at +2, so only 126 bytes are available
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
            xdataSerial.print("[err]: HORUS v3 Encoding Failed: ");
            xdataSerial.println(errCode);
          }
          if(assert_value != 0){
            xdataSerial.println("[err]: HORUS v3 Assert Failure, maybe hit buffer size limit");
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
        // Write CRC into bytes 0-1 of the packet
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

#if defined(RSM4x4) || defined(RSM4x2)   // dataRecorder runs on both boards
int buildHorusV3PacketDataRecorder(char* uncoded_buffer, uint8_t page){
  // Horus v3 packets are encoded using ASN1, and are encapsulated in packets
  // of sizes 32, 48, 64, 96 or 128 bytes (before coding)
  // The CRC16 for these packets is located at the *start* of the packet, still little-endian encoded

  // Erase the uncoded buffer
  // This has the effect of padding out the unused bytes in the packet with zeros
  memset(uncoded_buffer, 0, HORUS_UNCODED_BUFFER_SIZE);

  // Increment packet count
  horusV3PacketCount++;

  // Clamp all ASN.1-constrained fields
  const uint8_t  dr_sats      = constrain(gpsSats,                                  0,     31);
  const int32_t  dr_alt       = constrain((int32_t)gpsAlt,                      -1000,  50000);
  const uint16_t dr_speed     = constrain((int32_t)gpsSpeedKph,                     0,    512);
  const uint16_t dr_pressure  = constrain((int32_t)(pressureValue*10),              0,  12000);
  const int16_t  dr_tempInt   = constrain((int32_t)(readAvgIntTemp()*10),        -1023,   1023);
  const int16_t  dr_tempExt   = constrain((int32_t)(mainTemperatureValue*10),    -1023,   1023);
  const int16_t  dr_tempCust1 = constrain((int32_t)(extHeaterTemperatureValue*10), -1023, 1023);
  const uint8_t  dr_humidity  = constrain((int32_t)humidityValue,                   0,    100);

  horusTelemetry& asnMessage = g_txScratch.horusMsg;   // shared off-stack instance (see g_txScratch)
  new (&asnMessage) horusTelemetry{
      .payloadCallsign = HORUS_V3_CALLSIGN,
      .sequenceNumber = horusV3PacketCount,
      .timeOfDaySeconds = gpsHours * 3600 + gpsMinutes * 60 + gpsSeconds,
      .latitude = (int)(gpsLat * 100000),
      .longitude = (int)(gpsLong * 100000),
      .altitudeMeters = dr_alt,

      // 4 individually named sensors per packet (ASN.1 hard limit: max 4 per packet).
      // Pages: 0=A GNSS diagnostics, 1=B GNSS integrity, 2=C satellite counts,
      // 3=D flight stats, 4=E thermal/heater, 5=F ozone pump, 6=G OIF411.
      .extraSensors = [&]() -> horusAdditionalSensors {
        if (page == 0) {                          // A: GNSS diagnostics
          // Only count rejected config messages (NAKs). Frame checksum errors are not
          // counted: on the slower RSM4x2 they are mostly harmless UART-overrun noise and
          // climbed fast enough to swamp the meaningful config-NAK signal.
          const int ubxErrs = (int)gpsCfgNakCount;
          return { .nCount = 4, .arr = {
            { .name = "gpspwr", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsStatus        } } } }, .exist = { .name = true, .values = true } },
            { .name = "pdop",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsHdop          } } } }, .exist = { .name = true, .values = true } },
            { .name = "resets", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsResetCounter  } } } }, .exist = { .name = true, .values = true } },
            { .name = "ubxerrs",.values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { ubxErrs              } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 1) {                   // B: GNSS integrity (RSM4x4/M10 only; skipped on RSM4x2)
          // The 0..255 CW jam indicator, the spoofing state (NAV-STATUS) and the combined
          // integrity warning.
          return { .nCount = 3, .arr = {
            { .name = "jamlvl",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.jamIndicator } } } }, .exist = { .name = true, .values = true } },
            { .name = "spoofing", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.spoofState   } } } }, .exist = { .name = true, .values = true } },
            { .name = "integrity",.values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsJamWarning    } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 3) {                   // D: flight statistics
          return { .nCount = 4, .arr = {
            { .name = "flying", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)beganFlying      } } } }, .exist = { .name = true, .values = true } },
            { .name = "burst",  .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)burstDetected    } } } }, .exist = { .name = true, .values = true } },
            { .name = "hmax",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)maxAlt           } } } }, .exist = { .name = true, .values = true } },
            { .name = "vmax",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)maxSpeed         } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 2) {                   // C: satellites used per constellation
          // The four ranging constellations the M10 tracks: GPS, Galileo, BeiDou, GLONASS
          // (an inactive one just reads 0). SBAS is an augmentation, not counted here.
          return { .nCount = 4, .arr = {
            { .name = "gpscount", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.satGps } } } }, .exist = { .name = true, .values = true } },
            { .name = "galcount", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.satGal } } } }, .exist = { .name = true, .values = true } },
            { .name = "beicount", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.satBds } } } }, .exist = { .name = true, .values = true } },
            { .name = "glocount", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gps.satGlo } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 5) {                   // F: ozone pump
          // Physical measurements go out as floats (horusReal) so a decoder shows the
          // real value (17.1 C), not a scaled integer (171). Only flags/versions stay int.
          return { .nCount = 4, .arr = {
            { .name = "pumpt",  .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzonePumpTemperature } } } }, .exist = { .name = true, .values = true } },
            { .name = "o3c",    .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzoneCurrent         } } } }, .exist = { .name = true, .values = true } },
            { .name = "pumpu",  .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzoneBatteryVoltage  } } } }, .exist = { .name = true, .values = true } },
            { .name = "pumpc",  .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzonePumpCurrent     } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 6) {                   // G: OIF411
          return { .nCount = 4, .arr = {
            { .name = "oifu",    .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzoneExtVoltage                 } } } }, .exist = { .name = true, .values = true } },
            { .name = "oiferr",  .values = { .kind = horusInt_PRESENT,  .u = { .horusInt  = { .nCount = 1, .arr = { (xdataOzoneDiagnostics != 0) ? 1 : 0 } } } }, .exist = { .name = true, .values = true } },
            { .name = "oifver",  .values = { .kind = horusInt_PRESENT,  .u = { .horusInt  = { .nCount = 1, .arr = { (int)xdataOzoneFwVersion             } } } }, .exist = { .name = true, .values = true } },
            { .name = "o3ppb",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)xdataOzonePpb                   } } } }, .exist = { .name = true, .values = true } },
          }};
        } else {                                  // E (page 4): thermal / heater
          return { .nCount = 4, .arr = {
            { .name = "radiotemp", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)readRadioTemp()           } } } }, .exist = { .name = true, .values = true } },
            { .name = "rpmtemp",  .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)rpm411InternalTemperature  } } } }, .exist = { .name = true, .values = true } },
            { .name = "extpwr",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)extHeaterPwmStatus         } } } }, .exist = { .name = true, .values = true } },
            { .name = "refpwr",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)referenceHeaterStatus      } } } }, .exist = { .name = true, .values = true } },
          }};
        }
      }(),

      .velocityHorizontalKilometersPerHour = dr_speed,
      .gnssSatellitesVisible = dr_sats,
      .ascentRateCentimetersPerSecond = vVCalc * 100,
      .pressurehPa_x10 = dr_pressure,

      .temperatureCelsius_x10 = {
          .internal = dr_tempInt,
          .external = dr_tempExt,
          .custom1  = dr_tempCust1,
          .custom2  = 0,
          .exist = { .internal = true, .external = true, .custom1 = true, .custom2 = false }
      },

      .humidityPercentage = dr_humidity,

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
                    HORUS_UNCODED_BUFFER_SIZE-2  // buffer starts at +2, so only 126 bytes are available
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
        // Write CRC into bytes 0-1 of the packet
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
#endif  // dataRecorder (buildHorusV3PacketDataRecorder) - both boards

#ifdef RSM4x4
int build_horus_binary_packet_v2(char* buffer) {
  horusPacketCount++;

  struct HorusBinaryPacketV2 BinaryPacketV2;

  BinaryPacketV2.PayloadID = horusPayloadId;
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

  BinaryPacketV2.dummy1 = vVCalc * 100;
  BinaryPacketV2.dummy2 = mainTemperatureValue * 10;
  BinaryPacketV2.dummy3 = humidityValue;
  BinaryPacketV2.dummy4 = pressureValue * 10;
  BinaryPacketV2.dummy5 = 0;

  BinaryPacketV2.Checksum = (uint16_t)crc16((unsigned char*)&BinaryPacketV2, sizeof(BinaryPacketV2) - 2);

  memcpy(buffer, &BinaryPacketV2, sizeof(BinaryPacketV2));

  return sizeof(struct HorusBinaryPacketV2);
}
#endif

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
    xdataSerial.println("[info]: SHUTDOWN - powering off");
  }

  for (int i = 0; i < 3; i++) {
    redLed();
    delay(200);
    bothLedOff();
    delay(200);
  }
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
  // Status model is intentionally simple: only OK or ERR (no 'warn' state).
  vBatWarn = false;
  gpsFixWarn = false;

  err = false;
  ok = true;  // Default to ok until proven otherwise

  float vBat = readBatteryVoltage();
  if (vBat < vBatWarnValue) {
    vBatWarn = true;
  }

  if (gpsSats < gpsSatsWarnValue) {
    if (gpsOperationMode == 0) {
      gpsFixWarn = false;

    } else {
      gpsFixWarn = true;

      setStage("50");
    }
  } else {
    if (xdataPortMode == 1 || xdataPortMode == 3) {
      setStage("59");
    }
  }

  // OIF411 error: only internal diagnostics faults (0x0004, 0x0400) light the red LED.
  // Connection timeout is informational only - not an LED error (timeouts are normal at boot).
  bool oif411Err = (xdataPortMode == 3) &&
                   (xdataOzoneDiagnostics != 0 && xdataOzoneDiagnostics != 0xFFFF);
  if (sensorBoomFault || calibrationError || rpm411Error || vBatWarn || oif411Err) {
    err = true;
    ok = false;
  } else {
    err = false;
    ok = true;
  }

  if (ledStatusEnable) {
    if (gpsAlt > ledAutoDisableHeight) {
      ledsEnable = false;
    } else {
      ledsEnable = true;
    }

    if (ledsEnable) {
      bool noGpsFix = (gpsOperationMode != 0 && gpsSats < 5);

      if (err) {
        redLed();
      } else if (noGpsFix) {
        orangeLed();
      } else {
        greenLed();
      }
    }
  } else {
    bothLedOff();
  }
}

void serialStatusHandler() {
  if (xdataPortMode != 1) return;

  if (sensorBoomFault)  xdataSerial.println(F("[err]: sensorBoom - sensor boom fault"));
  if (calibrationError) xdataSerial.println(F("[err]: calibration - calibration error"));
  if (rpm411Error)      xdataSerial.println(F("[err]: rpm411 - RPM411 connection error"));
  if (vBatWarn)         xdataSerial.println(F("[warn]: vBat - low battery voltage"));
  if (gpsFixWarn)       xdataSerial.println(F("[warn]: gpsFix - no GPS fix, waiting..."));
  if (ok)               xdataSerial.println(F("[ok]: all systems nominal"));
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

  // logf (not log): single precision keeps the double-precision libm out of the
  // flash-tight F100 build; the Beta-equation result is unchanged at sensor accuracy.
  float temperatureK = 1.0f / (1.0f / (25.0f + 273.15f) + (1.0f / THERMISTOR_B) * logf(THERMISTOR_R25 / resistance));
  float temperatureC = temperatureK - 273.15f;

  return temperatureC;
}

float readRadioTemp() {  // Si4032 internal temperature ADC per the datasheet (approach inspired by RS41ng, no code reused)
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

// Wait for a UBX ACK/NAK. When ackCls != 0 the ACK must reference that exact
// message (its class/id sit at bytes 6-7 of the ACK-ACK/ACK-NAK payload), so a
// stray ACK for a different message is not mistaken for success.
//   returns  1 = ACK-ACK (accepted), 0 = ACK-NAK (rejected), -1 = timeout / none
int8_t ackWaitFor(uint8_t ackCls, uint8_t ackId, uint16_t timeoutMs)
{
  uint8_t buf[10];
  uint8_t idx = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (gpsSerial.available()) {
      uint8_t b = gpsSerial.read();

      if (idx == 0 && b != 0xB5) continue;          // sync char 1
      if (idx == 1 && b != 0x62) { idx = 0; continue; }  // sync char 2

      buf[idx++] = b;

      if (idx == 10) {                              // ACK/NAK packets are 10 bytes
        if (buf[2] == 0x05 && (buf[3] == 0x00 || buf[3] == 0x01)) {
          bool matches = (ackCls == 0) || (buf[6] == ackCls && buf[7] == ackId);
          if (matches) return (buf[3] == 0x01) ? 1 : 0;   // ACK-ACK : ACK-NAK
        }
        idx = 0;                                    // not our ACK - keep scanning
      }
    }
  }
  return -1;                                        // timed out
}

// Backward-compatible helper (matches any ACK).
bool ackWait(uint16_t timeoutMs = 100) { return ackWaitFor(0, 0, timeoutMs) == 1; }

// key = the CFG-VALSET config key (M10), or 0 when not applicable. Since every
// M10 config message is a CFG-VALSET (cls 0x06 / id 0x8A), the key is what tells
// you which specific setting the receiver rejected.
void gpsCfgWarn(uint8_t cls, uint8_t id, uint32_t key) {
  gpsCfgNakCount++;
  if (xdataPortMode == 1) {
    xdataSerial.print(F("[warn]: GPS config not ACKed - cls=0x"));
    xdataSerial.print(cls, HEX); xdataSerial.print(F(" id=0x")); xdataSerial.print(id, HEX);
    if (key) { xdataSerial.print(F(" key=0x")); xdataSerial.print(key, HEX); }
    xdataSerial.println();
  }
}

// Extract the CFG-VALSET config key from a UBX payload, or 0 if it is not one.
uint32_t ubxValsetKey(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  if (cls == 0x06 && id == 0x8A && len >= 8)
    return (uint32_t)payload[4] | ((uint32_t)payload[5] << 8) |
           ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24);
  return 0;
}

// Send a pre-built UBX frame (header + payload + checksum already in Buffer) and
// validate the receiver's ACK, retrying a couple of times. All these frames are
// CFG (class 0x06) messages, which the GPS always acknowledges.
void sendUblox(int Size, uint8_t* Buffer) {
  uint8_t cls = Buffer[2], id = Buffer[3];
  uint32_t key = (Size >= 8) ? ubxValsetKey(cls, id, Buffer + 6, Size - 8) : 0;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    gpsSerial.write(Buffer, Size);
    int8_t r = ackWaitFor(cls, id, 120);   // ACKs normally arrive <50 ms; keep worst-case blocking low
    if (r == 1) return;                 // accepted
    if (attempt == 2) { gpsCfgWarn(cls, id, key); return; }
    delay(20);
  }
}

// ---------------------------------------------------------------------------
// Native UBX config helpers (v68). Build a UBX frame with a runtime-computed
// Fletcher checksum, so new config messages never need a hand-typed checksum.
// On the M10 (RSM4x4) a CFG message is followed by an ACK wait; polls are not.
// ---------------------------------------------------------------------------
void writeUbxFrame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  #define _UBX_CK(b) do { ckA += (uint8_t)(b); ckB += ckA; } while (0)
  _UBX_CK(cls); _UBX_CK(id); _UBX_CK(len & 0xFF); _UBX_CK(len >> 8);
  for (uint16_t i = 0; i < len; i++) _UBX_CK(payload[i]);
  #undef _UBX_CK

  gpsSerial.write((uint8_t)0xB5); gpsSerial.write((uint8_t)0x62);
  gpsSerial.write(cls); gpsSerial.write(id);
  gpsSerial.write((uint8_t)(len & 0xFF)); gpsSerial.write((uint8_t)(len >> 8));
  for (uint16_t i = 0; i < len; i++) gpsSerial.write(payload[i]);
  gpsSerial.write(ckA); gpsSerial.write(ckB);
}

// Build + send a UBX frame. When expectAck, validate the ACK and retry a couple
// of times, logging a warning if the receiver never acknowledges it (which is
// exactly what happens if a config key/id is wrong - so a bad key is caught at
// runtime instead of silently doing nothing). Polls pass expectAck = false; the
// receiver's reply to a poll is a data message read by the parser, not an ACK.
void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, bool expectAck) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    writeUbxFrame(cls, id, payload, len);
    if (!expectAck) return;
    int8_t r = ackWaitFor(cls, id, 120);   // ACKs normally arrive <50 ms; keep worst-case blocking low
    if (r == 1) return;                 // accepted
    if (attempt == 2) { gpsCfgWarn(cls, id, ubxValsetKey(cls, id, payload, len)); return; }
    delay(20);
  }
}

// Poll a UBX message (zero-length request); the receiver replies with the
// current message, which the parser then decodes in gpsHandler().
void sendUbxPoll(uint8_t cls, uint8_t id) {
  sendUbx(cls, id, nullptr, 0, false);
}

#ifdef RSM4x4
// M10 UBX-CFG-VALSET (key/value into the RAM layer) helpers.
void m10ValSet(uint32_t key, const uint8_t* val, uint8_t valLen) {
  uint8_t p[16];
  p[0] = 0x00; p[1] = 0x01; p[2] = 0x00; p[3] = 0x00;   // version, layer=RAM, reserved
  p[4] = key & 0xFF; p[5] = (key >> 8) & 0xFF; p[6] = (key >> 16) & 0xFF; p[7] = (key >> 24) & 0xFF;
  for (uint8_t i = 0; i < valLen && i < 8; i++) p[8 + i] = val[i];
  sendUbx(0x06, 0x8A, p, 8 + valLen, true);
}
void m10ValSetU1(uint32_t key, uint8_t v)  { m10ValSet(key, &v, 1); }
void m10ValSetI1(uint32_t key, int8_t v)   { uint8_t b = (uint8_t)v; m10ValSet(key, &b, 1); }
void m10ValSetU2(uint32_t key, uint16_t v) { uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) }; m10ValSet(key, b, 2); }
void m10ValSetU4(uint32_t key, uint32_t v) { uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) }; m10ValSet(key, b, 4); }

// Secondary-GNSS mode (gpsSecondaryGnss) decoders. 0 = BeiDou B1C + GLONASS, 1 = BeiDou
// B1I only, 2 = GLONASS only.
static inline bool gpsUseBeidou()  { return gpsSecondaryGnss != 2; }  // BeiDou on in modes 0,1
static inline bool gpsUseGlonass() { return gpsSecondaryGnss != 1; }  // GLONASS on in modes 0,2
static inline bool gpsBeidouB1c()  { return gpsSecondaryGnss == 0; }  // B1C signal only when paired with GLONASS

// Signature of the constellation/signal config last applied (-1 = unknown, forces a
// resend). m10ResetConstellationCache() invalidates it after a GPS power-up / reset.
static int s_conSig = -1;
void m10ResetConstellationCache() { s_conSig = -1; }

// Apply the constellation + BeiDou-signal set in ONE atomic CFG-VALSET. This matters:
//  - the M10 validates the whole GNSS configuration together, so a valid combination
//    (e.g. GPS + Galileo + BeiDou B1C + GLONASS + SBAS) is accepted as a set, whereas
//    enabling the same keys one at a time passes through an invalid intermediate state
//    (BeiDou on its default B1I signal WHILE GLONASS is on - which the single-band
//    receiver refuses) and made the GLONASS enable NAK;
//  - it resets the GNSS subsystem only ONCE, not once per key.
// BeiDou's signal (B1C 0x1031000f, shares the 1575.42 MHz L1 centre so it can pair with
// GLONASS; vs B1I 0x1031000d at 1561 MHz, more interference-resilient but no GLONASS) is
// set in the same message so BeiDou never comes up on the wrong signal. Cached so it is
// only re-sent when the config actually changes (a resend would reset the receiver).
void m10SetConstellations(bool gps, bool glo, bool gal, bool bds, bool qzss, bool sbas, bool bdsB1c) {
  int sig = gps | (glo << 1) | (gal << 2) | (bds << 3) | (qzss << 4) | (sbas << 5) | (bdsB1c << 6);
  if (sig == s_conSig) return;                       // already applied - do not reset the receiver again

  uint8_t p[64];
  uint8_t n = 0;
  p[n++] = 0x00; p[n++] = 0x01; p[n++] = 0x00; p[n++] = 0x00;   // version, layer = RAM, reserved
  auto addKey = [&](uint32_t key, uint8_t val) {
    p[n++] = key & 0xFF; p[n++] = (key >> 8) & 0xFF; p[n++] = (key >> 16) & 0xFF; p[n++] = (key >> 24) & 0xFF;
    p[n++] = val;
  };
  addKey(0x1031001FUL, gps  ? 1 : 0);                // CFG-SIGNAL-GPS_ENA
  addKey(0x10310021UL, gal  ? 1 : 0);                // CFG-SIGNAL-GAL_ENA
  addKey(0x10310022UL, bds  ? 1 : 0);                // CFG-SIGNAL-BDS_ENA
  addKey(0x1031000fUL, (bds &&  bdsB1c) ? 1 : 0);    // CFG-SIGNAL-BDS_B1C_ENA (B1C)
  addKey(0x1031000dUL, (bds && !bdsB1c) ? 1 : 0);    // CFG-SIGNAL-BDS_B1_ENA  (B1I)
  addKey(0x10310025UL, glo  ? 1 : 0);                // CFG-SIGNAL-GLO_ENA
  addKey(0x10310024UL, qzss ? 1 : 0);                // CFG-SIGNAL-QZSS_ENA
  addKey(0x10310020UL, sbas ? 1 : 0);                // CFG-SIGNAL-SBAS_ENA

  sendUbx(0x06, 0x8A, p, n, true);                   // CFG-VALSET, ACKed (retried on NAK)
  delay(500);                                        // GNSS-subsystem reset settle (per M10 manual)
  s_conSig = sig;
}

// M10 receiver power mode (CFG-PM). The M10 has two power-save modes - ON/OFF (PSMOO) and
// cyclic tracking (PSMCT) - selected by CFG-PM-OPERATEMODE (0x20D00001: 0 = FULL/continuous,
// 1 = PSMOO, 2 = PSMCT). We use PSMCT for the intelligent tiers' cyclic power save.
//
// HARD CONSTRAINT (u-blox M10 integration manual, "Power save mode"): PSM does NOT support the
// BeiDou B1C signal, and the receiver cannot process SBAS while in PSM (u-blox recommends
// disabling SBAS). If either B1C or SBAS is enabled, the receiver NAKs
// CFG-PM-OPERATEMODE = PSMCT/PSMOO (the "20d00001 not acked" we used to see). The default
// gpsSecondaryGnss = 0 ("BeiDou B1C + GLONASS") enables B1C; only modes 1 (B1I only) and
// 2 (GLONASS only) are PSM-legal. See m10CyclicTrackingUsable(): we NEVER send PSMCT unless
// the live signal set is PSM-legal, so a mismatched config falls back to continuous instead
// of a rejected command. The Firmware Builder interlocks these so a normal build can't reach
// the bad state; this runtime check keeps a hand-edited CONFIG.h safe too.
int8_t s_pmMode = -1;                            // cache: -1 unknown, 0 FULL, 2 PSMCT
void m10ResetPmCache() { s_pmMode = -1; }

// True only when cyclic tracking is both requested AND compatible with the configured signals
// (no BeiDou B1C, no SBAS). Any incompatible config reads as "off" here - fail safe.
static inline bool m10CyclicTrackingUsable() {
  return m10CyclicTracking && !gpsBeidouB1c() && !gpsSbasEnable;
}

void m10SetContinuous() {
  if (s_pmMode == 0) return;                     // already continuous - don't re-send
  m10ValSetU1(0x20D00001UL, 0);                  // CFG-PM-OPERATEMODE = FULL
  s_pmMode = 0;
}

// Enter cyclic tracking. onTimeSec = CFG-PM-ONTIME, the time held in full tracking each cycle.
// Caller MUST have checked m10CyclicTrackingUsable() first (B1C/SBAS off), or the M10 NAKs it.
void m10SetPsmct(uint16_t onTimeSec) {
  if (s_pmMode == 2) return;                     // already in cyclic tracking
  if (onTimeSec < 1) onTimeSec = 1;
  m10ValSetU2(0x30D00005UL, onTimeSec);          // CFG-PM-ONTIME (s)
  m10ValSetU1(0x20D00001UL, 2);                  // CFG-PM-OPERATEMODE = PSMCT
  s_pmMode = 2;
}
#endif

// u-blox 6 legacy CFG-MSG: set the output rate of one message on the current port.
void m6CfgMsgRate(uint8_t cls, uint8_t id, uint8_t rate) {
  uint8_t p[3] = { cls, id, rate };
  sendUbx(0x06, 0x01, p, 3, true);   // CFG-MSG is ACKed
}

// Map gpsTrackingProfile -> (minimum elevation deg, minimum C/N0 dBHz).
// Higher sensitivity keeps weaker/lower satellites (better fix availability
// and geometry, a little more power). Verify exact effect on your hardware.
void gpsTrackingProfileValues(int8_t &minElevDeg, uint8_t &minCno) {
  switch (gpsTrackingProfile) {
    case 0:  minElevDeg = 0;  minCno = 6;  break;   // max sensitivity - fight for every sat
    case 2:  minElevDeg = 8;  minCno = 30; break;   // ultra power saving
    case 1:
    default: minElevDeg = 4;  minCno = 20; break;   // balanced (default)
  }
}

// Configure the receiver for the native UBX data path: dynamic model, disable
// NMEA, enable the UBX nav + jamming messages we parse, set the update rate and
// apply the tracking profile. Called from initGPS().
//
// NOTE: the register keys / message IDs below follow the u-blox interface
// manuals (M10 config-key DB and u-blox 6 receiver-description). If a unit
// behaves oddly, verify these against the manual for that exact module.
void gpsConfigureUbx() {
  const uint16_t measMs = 1000 / (gpsUpdateRateHz == 0 ? 1 : gpsUpdateRateHz);
  int8_t  minElevDeg; uint8_t minCno;
  gpsTrackingProfileValues(minElevDeg, minCno);

  if (rsm4x4) {
#ifdef RSM4x4
    // Dynamic model (configurable; default 6 = Airborne <1g)
    if (ubloxGpsAirborneMode) m10ValSetU1(0x20110021UL, gpsDynamicModel);  // CFG-NAVSPG-DYNMODEL

    // Output protocols: UBX on, NMEA off (UART1)
    m10ValSetU1(0x10740001UL, 1);   // CFG-UART1OUTPROT-UBX  = 1
    m10ValSetU1(0x10740002UL, 0);   // CFG-UART1OUTPROT-NMEA = 0

    // Enable the NAV-PVT solution stream. MON-RF (the 0..255 CW jam indicator) and
    // NAV-STATUS (spoofing) are POLLED each cycle in gpsHandler().
    m10ValSetU1(0x20910007UL, 1);     // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1

    // Solution / measurement rate
    m10ValSetU2(0x30210001UL, measMs);  // CFG-RATE-MEAS (ms)
    m10ValSetU2(0x30210002UL, 1);       // CFG-RATE-NAV  (cycles) = 1

    // Tracking profile: elevation + C/N0 masks. Both keys are the ones the
    // legacy (flight-tested) config already used, so they are known-valid on
    // this module: 0x..A4 = INFIL_MINELEV, 0x..A3 = INFIL_MINCNO.
    m10ValSetI1(0x201100A4UL, minElevDeg);  // CFG-NAVSPG-INFIL_MINELEV (deg)
    m10ValSetU1(0x201100A3UL, minCno);      // CFG-NAVSPG-INFIL_MINCNO  (dBHz)

    // SBAS is enabled as part of the atomic constellation set (m10SetConstellations),
    // called from initGPS and the intelligent GPS management.

    // AssistNow Autonomous: on-board orbit prediction -> faster re-acquisition
    if (gpsAssistNowAutonomous)
      m10ValSetU1(0x10230001UL, 1);   // CFG-ANA-USE_ANA = 1 (verified against M10 SPG 5.30 config DB)

    // NOTE: the old ubxCfgValSet_maxSvs64 message was dropped - its key was
    // 0x20110021 (that is DYNMODEL, not a max-SV limit) and its hardcoded
    // checksum was wrong, so the M10 always NAKed it and it never did anything.
    // The M10 has no artificial SV cap by default, so nothing is needed here.
#endif
  }
  else if (rsm4x2) {
    // Dynamic model / masks (default airborne 6 applied by initGPS via CFG-NAV5)

    // Disable NMEA output (set every standard NMEA message rate to 0)
    m6CfgMsgRate(0xF0, 0x00, 0);   // GGA
    m6CfgMsgRate(0xF0, 0x01, 0);   // GLL
    m6CfgMsgRate(0xF0, 0x02, 0);   // GSA
    m6CfgMsgRate(0xF0, 0x03, 0);   // GSV
    m6CfgMsgRate(0xF0, 0x04, 0);   // RMC
    m6CfgMsgRate(0xF0, 0x05, 0);   // VTG

    // Enable the UBX nav messages we parse
    m6CfgMsgRate(0x01, 0x02, 1);   // NAV-POSLLH  (position + altitude)
    m6CfgMsgRate(0x01, 0x12, 1);   // NAV-VELNED  (velocity, incl. velD)
    m6CfgMsgRate(0x01, 0x06, 1);   // NAV-SOL     (fix, numSV, pDOP)
    m6CfgMsgRate(0x01, 0x21, 1);   // NAV-TIMEUTC (UTC time)
    // NAV-STATUS (spoofing) and MON-HW (jamming) are polled in gpsHandler().

    // Solution / measurement rate: CFG-RATE (measMs, navRate=1, timeRef=GPS)
    uint8_t rate[6] = { (uint8_t)(measMs & 0xFF), (uint8_t)(measMs >> 8), 0x01, 0x00, 0x01, 0x00 };
    sendUbx(0x06, 0x08, rate, 6, true);

    // Dynamic model + minimum elevation via CFG-NAV5 (mask selects the fields).
    uint8_t nav5[36] = {0};
    uint16_t nav5mask = 0x0002;          // minElev
    nav5[12] = (uint8_t)minElevDeg;      // minElev (deg)
    if (ubloxGpsAirborneMode) {
      nav5mask |= 0x0001;                // dynModel
      nav5[2]   = gpsDynamicModel;
    }
    nav5[0] = nav5mask & 0xFF; nav5[1] = nav5mask >> 8;
    sendUbx(0x06, 0x24, nav5, 36, true);

    // SBAS augmentation (EGNOS/WAAS/MSAS)
    if (gpsSbasEnable) {
      uint8_t sbas[8] = { 0x01, 0x03, 0x03, 0x00, 0, 0, 0, 0 };  // mode=on, usage=range+diff, 3 SBAS
      sendUbx(0x06, 0x16, sbas, 8, true);   // CFG-SBAS
    }
    // (AssistNow Autonomous on u-blox 6 needs CFG-NAVX5; applied on the M10 path only.)
  }
}

// True once enough time has elapsed since the last GPS power-mode/tier change.
// Debounces the adaptive switching so a satellite count wobbling around a
// boundary can't fire a burst of UBX reconfigurations (wastes power/time).
bool gpsPowerChangeDue() {
  if (!gpsPowerModeInitialized) return true;                 // first decision applies at once
  return (millis() - lastPowerSaveChange) >= gpsPowerSaveDebounce;
}
void gpsPowerModeApplied() {
  lastPowerSaveChange = millis();
  gpsPowerModeInitialized = true;
}

#ifdef RSM4x4
// Desired M10 intelligent tier from the satellite count. Tiers: 1 = weak (<=10,
// continuous), 2 = moderate (11-15), 3 = strong (>=16, cyclic power-save).
//
// The response is deliberately asymmetric. A weakening fix downgrades at once, on
// the nominal boundaries with no hysteresis, so the receiver returns to a harder-
// tracking tier the moment it needs to (e.g. at 10 sats it drops straight back to
// tier 1 / continuous). A strengthening fix climbs lazily - only once the count is
// clear of the boundary by the hysteresis margin - so a count wobbling just above a
// threshold cannot keep flipping up into power-save.
uint8_t m10DesiredTier() {
  uint8_t cur = currentM10IntelligentMode;
  const uint8_t H = 1;                                       // upgrade hysteresis (1 sat past the line)
  uint8_t nominal = (gpsSats <= 10) ? 1 : (gpsSats <= 15 ? 2 : 3);
  if (cur < 1 || cur > 3) return nominal;                    // no valid tier yet
  if (nominal < cur) return nominal;                         // fix weakened -> downgrade now (strict)
  if (cur == 1 && gpsSats >= 11 + H) return (gpsSats >= 16 + H) ? 3 : 2;  // rising, past hysteresis
  if (cur == 2 && gpsSats >= 16 + H) return 3;
  return cur;                                                // hold
}

// Apply the constellation + power configuration for an M10 tier. Tiers: 1 weak,
// 2 moderate, 3 strong. Constellation optimization sheds GNSS for power once the
// fix is strong enough; aggressive mode sheds sooner and more. Higher tiers also
// move to cyclic PSM. GPS + SBAS always stay on; the secondary GNSS (BeiDou and/or
// GLONASS) follows gpsSecondaryGnss.
void applyM10Tier(uint8_t tier) {
  bool secOn = true, galOn = true;   // secondary GNSS (BeiDou/GLONASS) and Galileo
  if (m10ConstellationOptimization) {
    if (m10AggressiveOpt) {
      // Aggressive: drop Galileo AND the secondary GNSS from a moderate fix up
      // (tier >= 2), leaving GPS + SBAS at strong fixes. Kept deliberately eager.
      secOn = (tier < 2);
      galOn = (tier < 2);
    } else {
      // Balanced: only drop Galileo at a strong fix; keep the secondary GNSS
      // (BeiDou/GLONASS), GPS and SBAS. Gentle, modest saving.
      galOn = (tier < 3);
    }
  }
  bool bdsOn = gpsUseBeidou()  && secOn;
  bool gloOn = gpsUseGlonass() && secOn;
  m10SetConstellations(true, gloOn, galOn, bdsOn, gpsQzssEnable, gpsSbasEnable, gpsBeidouB1c());

  // Power mode. Cyclic tracking (PSMCT) needs a PSM-legal signal set (no B1C, no SBAS - see
  // m10CyclicTrackingUsable), so an incompatible config just stays continuous. Weak fixes
  // (tier 1) always stay continuous for fastest acquisition; aggressive optimization enters
  // PSMCT one tier earlier (from tier 2), balanced only at the strong-fix tier (3).
  bool wantPsmct = m10CyclicTrackingUsable() &&
                   (tier >= 3 || (tier == 2 && m10AggressiveOpt));
  if (wantPsmct) m10SetPsmct(m10CyclicPeriodSec);
  else           m10SetContinuous();
  currentM10IntelligentMode = tier;
}
#endif

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void GPSManagement() {
  if(gpsOperationMode == 0) { // GPS disabled
    shutdownGPS();
  }
  else if(gpsOperationMode == 1) {
    startGPS();

    if (currentGPSPowerMode != 1) {
      if(rsm4x4) {
#ifdef RSM4x4
        m10SetContinuous();
        m10SetConstellations(true, gpsUseGlonass(), true, gpsUseBeidou(), gpsQzssEnable, gpsSbasEnable, gpsBeidouB1c());
#endif
      }
      else if(rsm4x2) {
        sendUblox(sizeof(ubxCfgNav5_maxPerformance), ubxCfgNav5_maxPerformance);
      }
      currentGPSPowerMode = 1;
    }
  }
  else if (gpsOperationMode == 2) {   // NFW Intelligent GNSS Algorithms (both boards)
    startGPS();

    if (rsm4x4) {
#ifdef RSM4x4
      // M10: full intelligent management - adaptive constellation + power tiers,
      // debounced + hysteresis (see m10DesiredTier / applyM10Tier).
      uint8_t desired = m10DesiredTier();
      if (desired != currentM10IntelligentMode) {
        // A downgrade (weakening fix) is applied immediately; only upgrades into a
        // lighter-tracking tier wait out the debounce, so we never sit in power-save
        // while the fix degrades.
        bool downgrade = desired < currentM10IntelligentMode;
        if (downgrade || gpsPowerChangeDue()) {
          applyM10Tier(desired);
          gpsPowerModeApplied();
        }
      }
#endif
    }
    else if (rsm4x2) {
      // G6010: the only power lever this chip has is the power-save nav mode. Run
      // max performance while satellites are scarce (fight for the fix), and power-
      // save once there is a comfortable margin above 9 sats. Debounced + 2-sat
      // hysteresis so it can't flip every cycle. currentGPSPowerMode: 1 = max, 2 = save.
      const uint8_t H = 2, THR = 9;
      uint8_t desired = currentGPSPowerMode;
      if (currentGPSPowerMode == 0)        desired = (gpsSats < THR) ? 1 : 2;  // first decision
      else if (gpsSats <= THR - H)         desired = 1;                        // scarce -> max perf
      else if (gpsSats >= THR + H)         desired = 2;                        // plenty -> power save

      if (desired != currentGPSPowerMode && gpsPowerChangeDue()) {
        if (desired == 1) sendUblox(sizeof(ubxCfgNav5_maxPerformance), ubxCfgNav5_maxPerformance);
        else              sendUblox(sizeof(ubxCfgNav5_powerSave),      ubxCfgNav5_powerSave);
        currentGPSPowerMode = desired;
        gpsPowerModeApplied();
      }
    }
  }
  else {
    gpsOperationMode = 1;   // unknown mode -> safe max performance
  }
}


// oif411

static const float OIF411_CEF_P[]  = {2.0f,   3.0f,   5.0f,  10.0f,  20.0f,  30.0f,  50.0f, 100.0f, 200.0f, 300.0f, 500.0f, 1000.0f};
static const float OIF411_CEF_V[]  = {1.1655f, 1.1275f, 1.0895f, 1.0545f, 1.0325f, 1.0230f, 1.0150f, 1.0105f, 1.0075f, 1.0055f, 1.0030f, 1.0000f};
static const uint8_t OIF411_CEF_N  = 12;

float ozoneCef(float p_hPa) {
  if (p_hPa <= OIF411_CEF_P[0])             return OIF411_CEF_V[0];
  if (p_hPa >= OIF411_CEF_P[OIF411_CEF_N-1]) return OIF411_CEF_V[OIF411_CEF_N-1];
  for (uint8_t i = 1; i < OIF411_CEF_N; i++) {
    if (p_hPa <= OIF411_CEF_P[i]) {
      float t = (p_hPa - OIF411_CEF_P[i-1]) / (OIF411_CEF_P[i] - OIF411_CEF_P[i-1]);
      return OIF411_CEF_V[i-1] + t * (OIF411_CEF_V[i] - OIF411_CEF_V[i-1]);
    }
  }
  return 1.0f;
}

float ozoneIbg(float p_hPa) {
  const float A0 =  0.00122504f;
  const float A1 =  0.0001241115f;
  const float A2 = -2.687066e-8f;
  float P0 = (ozone_P0 > 0) ? ozone_P0 : 1013.25f;
  float num = A0 + A1*p_hPa  + A2*p_hPa*p_hPa;
  float den = A0 + A1*P0     + A2*P0*P0;
  return (den > 0) ? (num / den) * ozoneBackgroundCurrent : ozoneBackgroundCurrent;
}

void ozoneComputeValues() {
  float p_hPa = (pressureValue > 0) ? pressureValue : 1013.25f;
  float Tp_K  = xdataOzonePumpTemperature + 273.15f;
  float I_eff = xdataOzoneCurrent - ozoneIbg(p_hPa);
  if (I_eff < 0) I_eff = 0;
  float cef = ozoneCef(p_hPa);
  xdataOzonePartialPressure = 4.3087e-4f * I_eff * Tp_K * ozonePumpTime * cef;
  xdataOzonePpb = (p_hPa > 0) ? (xdataOzonePartialPressure * 10000.0f / p_hPa) : 0;
}

static uint16_t parseHex(const char* s, uint8_t len) {
  uint16_t v = 0;
  for (uint8_t i = 0; i < len; i++) {
    char c = s[i];
    uint8_t d = (c >= '0' && c <= '9') ? c-'0' : (c >= 'A' && c <= 'F') ? c-'A'+10 : (c >= 'a' && c <= 'f') ? c-'a'+10 : 0;
    v = (v << 4) | d;
  }
  return v;
}
static uint32_t parseHex32(const char* s, uint8_t len) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < len; i++) {
    char c = s[i];
    uint8_t d = (c >= '0' && c <= '9') ? c-'0' : (c >= 'A' && c <= 'F') ? c-'A'+10 : (c >= 'a' && c <= 'f') ? c-'a'+10 : 0;
    v = (v << 4) | d;
  }
  return v;
}

// data points directly to the content after "xdata=" (already stripped)
void ozoneParseFrame(const char* data) {
  uint8_t dlen = strlen(data);
  if (dlen < 4) return;
  if (data[0] != '0' || data[1] != '5') return;

  xdataInstrumentType   = 5;
  xdataInstrumentNumber = (int)parseHex(data + 2, 2);
  ozoneLastFrameMs      = millis();
  ozoneConnectionError  = false;

  if (dlen == 20) {
    // frame: [type2][num2][pumpT4][I5][batV2][pumpI3][extV2]
    ozoneFrameTotal++;
    uint16_t rawT = parseHex(data + 4, 4);
    float pumpT = (rawT & 0x8000) ? -(float)(rawT & 0x7FFF) * 0.01f
                                  :  (float)(rawT)          * 0.01f;
    xdataOzonePumpTemperature = pumpT;
    xdataOzoneCurrent         = (float)parseHex32(data + 8, 5) * 0.0001f;
    xdataOzoneBatteryVoltage  = (float)parseHex(data + 13, 2) * 0.1f;
    xdataOzonePumpCurrent     = (float)parseHex(data + 15, 3);
    xdataOzoneExtVoltage      = (float)parseHex(data + 18, 2) * 0.1f;
    ozoneComputeValues();
  } else if (dlen == 21 && data[20] == 'I') {
    // ID frame: [type2][num2][serial8][diag4][swver4][I]
    ozoneFrameTotal++;
    memcpy(xdataOzoneSerial, data + 4, 8);
    xdataOzoneSerial[8] = '\0';
    xdataOzoneDiagnostics = parseHex(data + 12, 4);
    xdataOzoneFwVersion   = (uint8_t)parseHex(data + 16, 4);
  }
}

// State-machine approach: scan stream for "xdata=", then collect until CR/LF.
// This re-syncs on every frame prefix and is robust against leading garbage.
void ozoneHandler() {
  if (xdataPortMode != 3) return;
  static char    lineBuf[96];
  static uint8_t lineLen = 0;
  static unsigned long lastByteMs = 0;

  unsigned long _ozStart = millis();
  do {
  while (xdataSerial.available()) {
    char c = xdataSerial.read();
    ozoneRxByteTotal++;

    // Bytes lost to RX-buffer overflow leave a stale partial line behind;
    // a long pause between bytes means a new frame, not a continuation.
    if (lineLen > 0 && (millis() - lastByteMs) > 500UL) lineLen = 0;
    lastByteMs = millis();

    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        // Take the LAST "xdata=" in the line: if a clipped frame got merged
        // with a complete one, the payload after the last header is intact.
        const char* x = NULL;
        for (const char* p = strstr(lineBuf, "xdata="); p != NULL; p = strstr(p + 1, "xdata=")) x = p;
        if (x != NULL) {
          x += 6;  // skip "xdata=" header, payload starts after '='
          strncpy(ozoneDbgRaw, x, sizeof(ozoneDbgRaw) - 1);
          ozoneDbgRaw[sizeof(ozoneDbgRaw) - 1] = '\0';
          ozoneDbgLen = (uint8_t)strlen(x);
          ozoneParseFrame(x);
        } else if (strstr(lineBuf, "CMD:") != NULL) {
          // GCS command interleaved with OIF411 data on the shared RX stream.
          runXdataCommand(lineBuf);
        }
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // line overflow without terminator - discard
    }
  }
  } while (millis() - _ozStart < 1150);

  if (ozoneLastFrameMs > 0 && (millis() - ozoneLastFrameMs) > 3000UL) {
    ozoneConnectionError = true;
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void gpsHandler() {

  GPSManagement();

  if(rsm4x4) {
    gpsStatus = currentM10IntelligentMode;
  }
  else {
    gpsStatus = currentGPSPowerMode;
  }

  // Upper bound on how long we wait for a fresh UBX solution before giving up
  // this cycle. Normally we return far sooner - the instant a solution arrives.
  // Kept tight so a marginal-signal cycle cannot stall long enough to slip a TX slot.
  const uint16_t gpsReadBudgetMs = 800;

  if (gpsOperationMode != 0) {  //if gps disabled then don't unnecesarly try to read it
    // Integrity + diagnostics polls (no CFG-MSGOUT key needed - the receiver answers a
    // zero-length poll, decoded by the parser on the next drain). These do not need a
    // fast rate, so throttle them to ~2 s: it keeps the per-cycle RX burst small (so it
    // does not compete with NAV-PVT or push a TX slot) while still refreshing steadily.
    static unsigned long _lastIntegrity = 0;
    bool pollIntegrity = (millis() - _lastIntegrity > 2000UL);
    if (pollIntegrity) _lastIntegrity = millis();

    if (rsm4x4) {
      // M10: UBX-MON-RF gives the 0..255 CW jam indicator; UBX-NAV-STATUS gives the
      // spoofing state. Both are polled here and answered.
      if (pollIntegrity) {
        if (gpsHardwareJammingMonitor)
          sendUbxPoll(UbxGnss::CLS_MON, UbxGnss::MON_RF);
        if (gpsSpoofingDetection)
          sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_STATUS);
      }
    } else {
      // u-blox 6 (G6010): only NAV-TIMEUTC (the legacy time message does not always
      // stream reliably here). This is a single-GPS receiver: no per-constellation
      // NAV-SAT and no spoofing. MON-HW is not polled - its jamming indicator is not
      // populated meaningfully on this module (it read a stale 255), so we report no
      // jamming rather than a garbage value.
      sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_TIMEUTC);   // time - keep every cycle
    }
    // Event-driven read: drain the UART and stop the moment a fresh position
    // solution is decoded (UBX streams at gpsUpdateRateHz). If none streams in
    // within half the budget, poll the receiver to force an immediate reply.
    unsigned long start = millis();
    bool gotFix = false, polled = false;

    while (millis() - start < gpsReadBudgetMs) {
      while (gpsSerial.available()) {
        if (gps.encode((uint8_t)gpsSerial.read())) gotFix = true;  // fresh position solution
      }
      if (gotFix) break;                          // freshest data in hand - return now

      if (!polled && (millis() - start) > (gpsReadBudgetMs / 2)) {
        if (rsm4x4) sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_PVT);
        else        sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_POSLLH);
        polled = true;
      }
    }

    // Per-constellation satellite counts (NAV-SAT, M10 only - u-blox 6 has no such
    // message). It is by far the largest UBX reply (8 + 12*numSvs bytes, ~370 with a
    // full sky). We poll it only *after* the position read is done and then drain it
    // in one continuous pass, so its long reply streams into a quiet, actively-emptied
    // UART buffer with no NAV-PVT competing. Polling it earlier let its tail overrun
    // the UART RX buffer mid-stream, failing the checksum (inflating ubxerrs) and
    // freezing the counts. Throttled so the extra ~100 ms read is only occasional.
    static unsigned long _lastNavSat = 0;
    if (rsm4x4 && millis() - _lastNavSat > 8000UL) {
      _lastNavSat = millis();
      gps.navSatUpdated = false;
      sendUbxPoll(UbxGnss::CLS_NAV, UbxGnss::NAV_SAT);
      unsigned long t = millis();
      while (millis() - t < 450 && !gps.navSatUpdated) {   // ~730 B multi-GNSS frame + poll latency
        while (gpsSerial.available()) gps.encode((uint8_t)gpsSerial.read());
      }
    }

    gpsTime = gps.time.value() / 100;
    gpsHours = gps.time.hour();
    gpsMinutes = gps.time.minute();
    gpsSeconds = gps.time.second();
    gpsLat = gps.location.lat();
    gpsLong = gps.location.lng();
    gpsAltFresh = gps.navUpdated;    // a fresh position solution was committed this cycle
    gps.navUpdated = false;
    gpsAlt = gps.altitude.meters();
    gpsSpeed = gps.speed.mps();
    gpsSpeedKph = gps.speed.kmph();
    gpsSats = gps.satellites.value();
    gpsHdop = gps.hdop.hdop();        // NAV-PVT/NAV-SOL pDOP (see CONFIG note)

    verticalVelocityCalculationHandler();

    if (xdataPortMode == 1) {
      static unsigned long _lastGpsLog = 0;
      if (millis() - _lastGpsLog >= 30000UL) {
        _lastGpsLog = millis();
        xdataSerial.print(F("[gps]: "));
        xdataSerial.print(gpsLat, 6); xdataSerial.print(F(","));
        xdataSerial.print(gpsLong, 6);
        xdataSerial.print(F(" alt=")); xdataSerial.print((int)gpsAlt);
        xdataSerial.print(F("m sats=")); xdataSerial.print(gpsSats);
        xdataSerial.print(F(" hdop=")); xdataSerial.print(gpsHdop, 1);
        xdataSerial.print(F(" vv=")); xdataSerial.print(vVCalc, 1); xdataSerial.print(F("m/s"));
#ifdef RSM4x4
        // Watch cyclic tracking here: tier reaches 3 (or 2 aggressive) to request PSMCT, and
        // psm shows what the M10 is actually doing - 0 off (continuous), 3 tracking,
        // 4 power optimized tracking (cyclic power-save is working), 5 inactive (asleep).
        xdataSerial.print(F(" tier=")); xdataSerial.print(currentM10IntelligentMode);
        xdataSerial.print(F(" psm="));  xdataSerial.print(gps.psmState);
#endif
        xdataSerial.println();
      }
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

    // GPS integrity warning: the receiver's spoofing flag (UBX-NAV-STATUS), with the
    // in-flight DOP / vertical-speed heuristic as a fallback.
    bool ubxSpoof     = (gps.spoofState  >= 2);    // 2 = spoofing indicated, 3 = multiple
    bool heuristicJam = beganFlying && (gpsHdop > 15 || fabs(vVCalc) > 300);
    if (ubxSpoof || heuristicJam) {
      gpsJamWarning = true;

      if (xdataPortMode == 1) {
        if (ubxSpoof) xdataSerial.println("[warn]: GPS SPOOFING detected!");
        else          xdataSerial.println("[warn]: GPS integrity warning is active!");
      }
    } else {
      gpsJamWarning = false;
    }
  }
}

// Tracks the GPS reset-pin state so startGPS()/shutdownGPS() only act and log on an
// actual on<->off transition. startGPS() is called every GPSManagement() cycle, so
// without this it re-drove the pin and spammed "GPS is ON" on every single loop.
bool gpsPoweredOn = false;

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void shutdownGPS() {
  digitalWrite(GPS_RESET_PIN, LOW);
  if (!gpsPoweredOn) return;                 // already off - nothing to do or log
  gpsPoweredOn = false;
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS shutdown");
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void startGPS() {
  digitalWrite(GPS_RESET_PIN, HIGH);
  if (gpsPoweredOn) return;                  // already on - do not re-log every cycle
  gpsPoweredOn = true;
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS is ON");
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void restartGPS() {
  digitalWrite(GPS_RESET_PIN, LOW);
  delay(3000);
  digitalWrite(GPS_RESET_PIN, HIGH);
  gpsPoweredOn = true;                        // ends powered on

  gpsResetCounter++;

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS restart has been issued");
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void initGPS() {
  if (xdataPortMode == 1) {
    xdataSerial.println(F("[info]: GPS settings are being initialized..."));
  }

  // Baseline dynamic model + navigation masks. On the M10 the model is applied
  // (configurably) by gpsConfigureUbx() below; on the u-blox 6 we send the full
  // CFG-NAV5 baseline here first, then overlay the tracking profile.
  if (ubloxGpsAirborneMode) {
    if (xdataPortMode == 1) {
      xdataSerial.println(F("[info]: setting GPS dynamic model..."));
    }
    if (rsm4x2) {
      sendUblox(sizeof(ubxCfgNav5_dynmodel6), ubxCfgNav5_dynmodel6);
      delay(500);
    }
  }

  // Native UBX data path: disable NMEA, enable the nav + jamming messages the
  // parser decodes, set the update rate (gpsUpdateRateHz) and the tracking
  // profile (elevation / C/N0). Replaces the former NMEA message setup.
  gpsConfigureUbx();

#ifdef RSM4x4
  // Enable the constellation set at start-up: GPS + Galileo + one secondary
  // (BeiDou and/or GLONASS, per gpsSecondaryGnss) + SBAS, optionally QZSS. The
  // intelligent GPS management then tunes which stay on per tier. The receiver was just
  // powered up / reset to its defaults, so invalidate the change-cache to force a full
  // (re)apply here.
  m10ResetConstellationCache();
  m10ResetPmCache();   // receiver reset to default (FULL) power mode too
  m10SetConstellations(true, gpsUseGlonass(), true, gpsUseBeidou(), gpsQzssEnable, gpsSbasEnable, gpsBeidouB1c());

  if(rsm4x4 && m10SuperS) {
    delay(100);
    sendUblox(sizeof(ubxCfgValSet_enableSuperS), ubxCfgValSet_enableSuperS);
  }

  // Ask the receiver which major constellations it actually supports and has enabled
  // (UBX-MON-GNSS) and log it, so the applied config can be verified at a glance.
  if (xdataPortMode == 1) {
    gps.monGnssSeen = false;
    sendUbxPoll(UbxGnss::CLS_MON, UbxGnss::MON_GNSS);
    unsigned long t = millis();
    while (millis() - t < 300 && !gps.monGnssSeen) {
      while (gpsSerial.available()) gps.encode((uint8_t)gpsSerial.read());
    }
    if (gps.monGnssSeen) {
      xdataSerial.print(F("[info]: GNSS supported:"));
      if (gps.gnssSupported & 0x01) xdataSerial.print(F(" GPS"));
      if (gps.gnssSupported & 0x08) xdataSerial.print(F(" GAL"));
      if (gps.gnssSupported & 0x04) xdataSerial.print(F(" BDS"));
      if (gps.gnssSupported & 0x02) xdataSerial.print(F(" GLO"));
      xdataSerial.print(F(" | enabled:"));
      if (gps.gnssEnabled & 0x01) xdataSerial.print(F(" GPS"));
      if (gps.gnssEnabled & 0x08) xdataSerial.print(F(" GAL"));
      if (gps.gnssEnabled & 0x04) xdataSerial.print(F(" BDS"));
      if (gps.gnssEnabled & 0x02) xdataSerial.print(F(" GLO"));
      xdataSerial.println();
    } else {
      xdataSerial.println(F("[info]: GNSS status poll (MON-GNSS) not answered"));
    }
  }
#endif

  if (xdataPortMode == 1) {
    xdataSerial.println(F("[info]: GPS settings done"));
  }
}

// Function to select heater and change its state (on/off)
// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void selectReferencesHeater(int heatingMode) {
  bool changed = (referenceHeaterStatus != heatingMode);
  referenceHeaterStatus = heatingMode;

  if (changed && xdataPortMode == 1) {   // log only on an actual level change, not every call
    xdataSerial.print("[info]: ref. heating ");
    xdataSerial.print(heatingMode);
    xdataSerial.println("/3");
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

void powerHandler() {
  if (readBatteryVoltage() < batteryCutOffVoltage && batteryCutOffVoltage != 0) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[err]: battery cutoff - power off");
    }

    radioDisableTx();

    if (xdataPortMode == 1) {
      xdataSerial.println("\n PSU_SHUTDOWN_PIN set HIGH, bye!");
    }

    hardwarePowerShutdown();
  }
}

// Function to select reading of a sensor and set its state (on/off)
// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
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
      break;

    case 6:
      digitalWrite(SPDT2, powerState);
      digitalWrite(PULLUP_TM, !powerState);
      digitalWrite(PULLUP_HYG, powerState);
      break;

    case 7:
      digitalWrite(SPDT3, powerState);
      digitalWrite(PULLUP_TM, !powerState);
      digitalWrite(PULLUP_HYG, powerState);
      break;

    default:
      // Invalid sensor number, do nothing
      break;
  }
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
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

// Raw reference-resistor frequencies from the most recent calibrateTempSensorBoom()
// call. The factory (mode 2) conversion interpolates against these raw 750 Ohm /
// 1100 Ohm readings instead of the averaged constant used by mode 1.
float lastRefFreq750  = 0;
float lastRefFreq1100 = 0;

#if defined(RSM4x4) || defined(RSM4x2)
/* Factory (Vaisala) calibration conversion - sensor calibration mode 2.
   Reproduces the rs1729 (zilog80) RS41 PTU math using
   the sonde's original factory coefficients fetched from SondeHub. Kept in single
   precision (float, expf/logf) so it also fits the 64 KB flash of the RSM4x2 (F100),
   which has no double-precision FPU - the accuracy loss is far below the sensor's.
     f       - sensor frequency
     f1 / f2 - low / high reference frequencies
   For temperatures f1 = getSensorBoomFreq(1) (750 Ohm), f2 = getSensorBoomFreq(2)
   (1100 Ohm). For humidity f1 = getSensorBoomFreq(7) (0 pF), f2 = getSensorBoomFreq(6)
   (47 pF), T = main temperature. */
// NFW's getSensorBoomFreq() returns a TRUE frequency (f = clk * samples / ticks),
// which is proportional to 1/R (and 1/C). The rs1729 interpolation below
// expects the raw measurement count, which is proportional to R (and C). So we feed
// the reciprocal (period). Any constant scale cancels in the interpolation, which is
// why this recovers Rc = R exactly. measFromFreq() centralises that conversion.
static inline float measFromFreq(float fr) { return (fr > 0.0f) ? (1.0f / fr) : 0.0f; }

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float getFactoryTc(float fr, float fr1, float fr2) {   // main PT temperature
  if (fr <= 0.0f || fr1 <= 0.0f || fr2 <= 0.0f) return -273.15f;
  float f  = measFromFreq(fr), f1 = measFromFreq(fr1), f2 = measFromFreq(fr2);
  float g  = (f2 - f1) / (factoryRefResistorHigh - factoryRefResistorLow);
  float Rb = (f1 * factoryRefResistorHigh - f2 * factoryRefResistorLow) / (f2 - f1);
  float Rc = f / g - Rb;
  float R  = Rc * factoryCalT;
  return (factoryTaylorT0 + factoryTaylorT1 * R + factoryTaylorT2 * R * R + factoryPolyT0) * (1.0f + factoryPolyT1);
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float getFactoryTH(float fr, float fr1, float fr2) {   // heater / RH-sensor temperature
  if (fr <= 0.0f || fr1 <= 0.0f || fr2 <= 0.0f) return -273.15f;
  float f  = measFromFreq(fr), f1 = measFromFreq(fr1), f2 = measFromFreq(fr2);
  float g  = (f2 - f1) / (factoryRefResistorHigh - factoryRefResistorLow);
  float Rb = (f1 * factoryRefResistorHigh - f2 * factoryRefResistorLow) / (f2 - f1);
  float Rc = f / g - Rb;
  float R  = Rc * factoryCalTU;
  return (factoryTaylorTU0 + factoryTaylorTU1 * R + factoryTaylorTU2 * R * R + factoryPolyTrh0) * (1.0f + factoryPolyTrh1);
}

// Saturation water-vapour pressure [Pa] (Hyland & Wexler), used to convert RH
// referenced to the sensor temperature into RH referenced to the air temperature.
float factoryVaporSatP(float Tc) {
  float T = Tc + 273.15f;
  return expf(-5800.2206f / T
             + 1.3914993f
             + 6.5459673f * logf(T)
             - 4.8640239e-2f * T
             + 4.1764768e-5f * T * T
             - 1.4452093e-8f * T * T * T);
}

// Full Vaisala factory relative-humidity calculation Reproduces the matrixU 7x6 calibration surface in (capacitance, sensor temp).
//   f, f1, f2 = humidity / 0pF-ref / 47pF-ref measurement frequencies
//   Tair      = main air temperature (getFactoryTc)
//   Tsensor   = RH-sensor (heater) temperature (getFactoryTH)
// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float getFactoryRH(float fr, float fr1, float fr2, float Tair, float Tsensor) {
  if (factoryCalibU0 == 0.0f) return -1.0f;
  if (fr <= 0.0f || fr1 <= 0.0f || fr2 <= 0.0f) return -1.0f;
  // Reciprocal: counts proportional to capacitance (see measFromFreq note above).
  float f  = measFromFreq(fr), f1 = measFromFreq(fr1), f2 = measFromFreq(fr2);
  float cfh = (f - f1) / (f2 - f1);
  float cap = factoryRefCapLow + (factoryRefCapHigh - factoryRefCapLow) * cfh;
  float Cp = ((float)cap / factoryCalibU0 - 1.0f) * factoryCalibU1;

  float Trh = ((float)Tsensor - 20.0f) / 180.0f;
  float b[6];
  float bk = 1.0f;
  for (int k = 0; k < 6; k++) { b[k] = bk; bk *= Trh; }   // b[k] = Trh^k

  float rh = 0.0f;
  float aj = 1.0f;                                          // aj = Cp^j
  for (int j = 0; j < 7; j++) {
    for (int k = 0; k < 6; k++) {
      rh += aj * b[k] * factoryMatrixU[6 * j + k];
    }
    aj *= Cp;
  }

  if (Tair < -40.0f) rh += (Tair - (-40.0f)) / 12.0f;      // low-temperature correction
  rh *= factoryVaporSatP(Tsensor) / factoryVaporSatP(Tair);

  if (rh < 0.0f)   rh = 0.0f;
  if (rh > 100.0f) rh = 100.0f;
  return rh;
}
#endif

// Function to calibrate the sensor using the two calibration resistors
// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
float calibrateTempSensorBoom() {
  // Get the frequencies for calibration resistors
  float freq750 = getSensorBoomFreq(1);  // Get frequency for 750Ω resistor
  selectSensorBoom(0, 0);
  float freq1100 = getSensorBoomFreq(2);  // Get frequency for 1100Ω resistor
  selectSensorBoom(0, 0);

  // Keep the raw reference frequencies for the factory (mode 2) conversion.
  lastRefFreq750  = freq750;
  lastRefFreq1100 = freq1100;

  // Calibration constant k: R * f (frequency-to-resistance ratio)
  // We use an average to balance between the two calibration resistors.
  float k = (750.0 * freq750 + 1100.0 * freq1100) / 2.0;

  return k;  // Return the calibration constant
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
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

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void sensorBoomHandler() {
  double tempCorrectionFactor = 1.0;

  if (sensorBoomEnable) {

    int lastReferenceHeaterStatus = referenceHeaterStatus;
    selectReferencesHeater(0);

    // Calibrate the sensor and get the calibration factor
    tempSensorBoomCalibrationFactor = calibrateTempSensorBoom();

#if defined(RSM4x4) || defined(RSM4x2)
    if (FACTORY_CAL_ACTIVE) {
      // ===== Factory (Vaisala) calibration path - measurement only (both board families) =====
      mainTemperatureFrequency = getSensorBoomFreq(4);
      if (mainTemperatureFrequency <= 0) {
        sensorBoomMainTempError = true;   // error flag is sent in telemetry / shown by Ground Control
      } else {
        sensorBoomMainTempError = false;
        // Factory mode: absolute Vaisala polynomial - NFW correction offsets are NOT applied.
        mainTemperatureValue = getFactoryTc(mainTemperatureFrequency, lastRefFreq750, lastRefFreq1100);
      }
      selectSensorBoom(0, 0);

      extHeaterTemperatureFrequency = getSensorBoomFreq(3);
      if (extHeaterTemperatureFrequency <= 0) {
        sensorBoomHumidityModuleError = true;
      } else {
        sensorBoomHumidityModuleError = false;
        // Factory mode: absolute Vaisala polynomial - NFW correction offsets are NOT applied.
        extHeaterTemperatureValue = getFactoryTH(extHeaterTemperatureFrequency, lastRefFreq750, lastRefFreq1100);
      }
      selectSensorBoom(0, 0);

      humidityFrequency   = getSensorBoomFreq(5);
      refCapHighFrequency = getSensorBoomFreq(6); // 47pF Ref
      refCapLowFrequency  = getSensorBoomFreq(7); // 0pF Ref
      // raw capacitance kept for reporting / telemetry parity
      humidityCapacitance = 47.0 * (float)(refCapLowFrequency - humidityFrequency) / (refCapLowFrequency - refCapHighFrequency);
      // factory RH: f1 = 0pF ref (gSBF7), f2 = 47pF ref (gSBF6),
      // Tair = main temperature (Tc), Tsensor = heater/RH-sensor temperature (TH)
      humidityValue = getFactoryRH(humidityFrequency, refCapLowFrequency, refCapHighFrequency, mainTemperatureValue, extHeaterTemperatureValue);
    }
#if defined(RSM4x4)
    else {
    // ===== NFW calibration path (mode 1) - RSM4x4 / RSM4x5 only (RSM4x2 is factory-only) =====
    // Get main temperature frequency
    mainTemperatureFrequency = getSensorBoomFreq(4);
    if (mainTemperatureFrequency <= 0) {
      sensorBoomMainTempError = true;  // Error if frequency is invalid

      if (xdataPortMode == 1) {
        xdataSerial.println("[warn]: Main temp sensor boom error (freq invalid)");
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
        xdataSerial.println("[warn]: ExtHeater temp sensor boom error (freq invalid)");
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
    } // end NFW calibration path
#endif  // RSM4x4 NFW else-branch
#endif  // RSM4x4 || RSM4x2 calibration branch

    // Check overall sensor status
    sensorBoomFault = sensorBoomMainTempError || sensorBoomHumidityModuleError;

    if (sensorBoomMainTempError && sensorBoomHumidityModuleError) {
      if (xdataPortMode == 1) {
        xdataSerial.println("[err]: The sensor boom seems disconnected!");
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
  // Vertical velocity now comes straight from the GPS solution (UBX velD),
  // which is a true receiver-computed climb/descent rate - no more differencing
  // successive altitudes (which aliased to 0 when gpsHandler ran faster than the
  // fix rate). Positive = climbing. Reported 0 only when there is no usable fix.
  if (gpsSats <= 3 || !gps.gnssFixOK) {
    vVCalc = 0;
    return;
  }

  vVCalc = gps.verticalVelocity;

  // Keep the altitude reference updated so any consumer of lastGpsAlt stays sane.
  if (gpsAltFresh) {
    lastGpsAltMillisTime = millis();
    lastGpsAlt = gpsAlt;
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

#ifdef RSM4x4
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
#endif  // RSM4x4 (aprsRecorderFormat)

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

#ifdef RSM4x4
// Private landing latch: once the sonde has flown, climbed above the threshold
// and dropped back below it, latch every mode onto its private frequency to keep
// the landing spot off public maps. Factored out of flightComputing() so it can
// also run inside the blocking low-altitude fast-TX loop - otherwise a fast-TX
// window that started before landing would keep broadcasting the descent on the
// public frequencies (the latch was never re-evaluated during that window).
void updatePrivateLanding() {
  if (privateLandingModeEnable && !privateLandingActive && beganFlying) {
    if (gpsAlt > privateLandingAltitudeThreshold) privateLandingArmed = true;
    if (privateLandingArmed && gpsAlt < privateLandingAltitudeThreshold) {
      privateLandingActive = true;
      if (xdataPortMode == 1) xdataSerial.println(F("[flt]: private landing frequencies active"));
    }
  }
}
#endif

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

  // Flight start: climbed flightStartClimbThreshold m above the launch baseline
  // for 5 consecutive fixes. Works at any ascent rate; the streak rejects GPS spikes.
  if (gpsSats >= 4) {
    // Wait flightBaselineSettleTime after the first fix before latching the baseline:
    // a cold-start GPS altitude can be 100-200 m off and only converges over a few
    // seconds, and latching it too early caused false liftoffs on the ground.
    if (flightFixAcquiredMillis == 0) flightFixAcquiredMillis = millis();

    if (!flightBaselineSet && (millis() - flightFixAcquiredMillis) >= flightBaselineSettleTime) {
      flightBaseAlt     = gpsAlt;
      flightPrevAlt     = gpsAlt;
      flightBaselineSet = true;
    }

    if (flightBaselineSet && !beganFlying && gpsAlt != flightPrevAlt) {   // only act on a fresh fix value
      if ((gpsAlt - flightBaseAlt) >= (float)flightStartClimbThreshold) {
        if (flightSustainedRise < 255) flightSustainedRise++;
      } else {
        flightSustainedRise = 0;
      }
      flightPrevAlt = gpsAlt;

      if (flightSustainedRise >= 5) {
        beganFlying = true;
        if (xdataPortMode == 1) xdataSerial.println(F("[flt]: liftoff!"));
      }
    }
  }

  if (gpsAlt + burstDetectionThreshold < maxAlt && !burstDetected) {
    burstDetected = true;
    if (xdataPortMode == 1) xdataSerial.println(F("[flt]: burst!"));
  }

  // Landing: after burst, descended back below the climb threshold above baseline.
  if (beganFlying && burstDetected && !hasLanded) {
    if ((gpsAlt - flightBaseAlt) < (float)flightStartClimbThreshold) {
      hasLanded = true;
      landingTimeMillis = millis();
      if (xdataPortMode == 1) xdataSerial.println(F("[flt]: landed!"));
    }
  }

  if (disableGpsImprovementInFlight && beganFlying) {
    cancelGpsImprovement = true;
  }

#ifdef RSM4x4
  updatePrivateLanding();
#endif

#ifdef RSM4x4
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
#endif

  // Ground-level pressure P0 for ozone calc: average 5 onboard readings, or fall
  // back to ISA sea level after 20 s if no pressure is available (e.g. no RPM411).
  if (xdataPortMode == 3 && !ozone_P0Set) {
    if (pressureValue > 1.0f) {
      ozone_P0Accumulator += pressureValue;
      ozone_P0SampleCount++;
      if (ozone_P0SampleCount >= 5) {
        ozone_P0    = ozone_P0Accumulator / 5.0f;
        ozone_P0Set = true;
      }
    } else if (millis() > 20000UL) {
      ozone_P0    = 1013.25f;   // ISA sea-level fallback (no onboard pressure)
      ozone_P0Set = true;
    }
  }
}

#ifdef RSM4x4
void lowAltitudeFastTxMode() {
  setRadioPower(7);
  while (millis() - lowAltitudeFastTxModeBeginTime < lowAltitudeFastTxDuration && !lowAltitudeFastTxModeEnd) {
    gpsHandler();
    updatePrivateLanding();   // keep the private-landing latch live during this blocking window
    ozoneHandler();
    sensorBoomHandler();
    pressureHandler();

    #ifdef RSM4x4
    if (horusEnable) {
      int pkt_len = build_horus_binary_packet_v2(rawbuffer);
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      setRadioModulation(0);
      setRadioFrequency(privateLandingActive ? horusPrivateFreq : horusFreqTable[0]);

      radioEnableTx();
      fsk4_preamble(horusPreambleLength);
      fsk4_write(codedbuffer, coded_len);
      radioDisableTx();
    }
    #endif

    if (horusV3Enable) {
      int pkt_len = buildHorusV3Packet(rawbuffer);

      // Bomb out if we can't encode
      if (pkt_len == 0){
        return;
      }
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      setRadioModulation(0);  // CW modulation
      setRadioFrequency(privateLandingActive ? horusV3PrivateFreq : horusV3FreqTable[0]);

      radioEnableTx();

      fsk4_preamble(horusPreambleLength);
      fsk4_write(codedbuffer, coded_len);
      radioDisableTx();
    }

    if (aprsEnable) {
      setRadioModulation(2);
      setRadioFrequency((privateLandingActive ? aprsPrivateFreq : aprsFreqTable[0]) - 0.002);  //its lower due to the deviation in FSK adding 0.002MHz when the signal is in total 10kHz wide

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
#endif  // RSM4x4 (lowAltitudeFastTxMode)

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
      float pipFreq = pipFrequencyMhz;
#ifdef RSM4x4
      if (privateLandingActive) pipFreq = pipPrivateFreq;
#endif
      setRadioPower(pipRadioPower);
      setRadioModulation(0);
      setRadioFrequency(pipFreq);

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: PIP on (MHz): ");
        xdataSerial.println(pipFreq);
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
      if (morseBeaconMode) {
        morseMsg = morseBeaconText;   // beacon mode: send the fixed custom text
      } else {
        morseMsg = createRttyMorsePayload();   // telemetry mode: send live data
      }
      const char* morseMsgCstr = morseMsg.c_str();
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Morse payload created: ");
        xdataSerial.println(morseMsg);
        xdataSerial.println();
      }

      float morseFreq = morseFrequencyMhz;
#ifdef RSM4x4
      if (privateLandingActive) morseFreq = morsePrivateFreq;
#endif
      setRadioPower(morseRadioPower);
      setRadioModulation(0);  // CW modulation
      setRadioFrequency(morseFreq);

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Morse transmitting on (MHz): ");
        xdataSerial.println(morseFreq);
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Transmitting morse...");
      }

      uint8_t morseRepeats = morseBeaconMode ? (morseBeaconRepeat == 0 ? 1 : morseBeaconRepeat) : 1;
      for (uint8_t r = 0; r < morseRepeats; r++) {
        transmitMorseString(morseMsgCstr, morseUnitTime);
      }
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

#ifdef RSM4x4
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

      float rttyFreq = privateLandingActive ? rttyPrivateFreq : rttyFrequencyMhz;
      setRadioPower(rttyRadioPower);
      setRadioModulation(0);  // CW modulation
      setRadioFrequency(rttyFreq);
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: RTTY frequency set to (MHz): ");
        xdataSerial.println(rttyFreq);
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
#endif  // RSM4x4 (rttyTx)

#ifdef RSM4x4
void horusTx() {
  int freqTableSize = sizeof(horusFreqTable) / sizeof(horusFreqTable[0]);

  if (horusEnable) {
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: HORUS V2 mode enabled");
    }

    if (radioEnablePA) {
      int pkt_len = build_horus_binary_packet_v2(rawbuffer);
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HORUS V2 payload created.");
      }

      // Private landing: transmit on the single private frequency only.
      if (privateLandingActive) freqTableSize = 1;

      for (int i = 0; i < freqTableSize; i++) {
        float currentFreq = privateLandingActive ? horusPrivateFreq : horusFreqTable[i];

        setRadioPower(horusRadioPower);
        setRadioModulation(0);
        setRadioFrequency(currentFreq);

        if (xdataPortMode == 1) {
          xdataSerial.print("[info]: TX HORUS V2 ");
          xdataSerial.println(currentFreq);
        }

        radioEnableTx();
        fsk4_preamble(horusPreambleLength);
        fsk4_write(codedbuffer, coded_len);
        radioDisableTx();
      }

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: HORUS V2 done");
      }

    } else {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }
  }
}
#endif

void horusV3Tx() {
  // Calculate the number of frequencies in the table automatically
  int freqTableSize = sizeof(horusV3FreqTable) / sizeof(horusV3FreqTable[0]);

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

#ifdef RSM4x4
      // Private landing: transmit on the single private frequency only.
      if (privateLandingActive) freqTableSize = 1;
#endif

      // 2. Loop through every frequency in the table
      for (int i = 0; i < freqTableSize; i++) {
        float currentFreq = horusV3FreqTable[i];
#ifdef RSM4x4
        if (privateLandingActive) currentFreq = horusV3PrivateFreq;
#endif

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
#ifdef RSM4x4
      // Private landing: transmit on the single private frequency only.
      if (privateLandingActive) tableSize = 1;
#endif
      for (int f = 0; f < tableSize; f++) {
        float currentFreq = aprsFreqTable[f];
#ifdef RSM4x4
        if (privateLandingActive) currentFreq = aprsPrivateFreq;
#endif
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

#if defined(RSM4x4) || defined(RSM4x2)   // dataRecorder runs on both boards
void dataRecorderTx() {
  if (!dataRecorderEnable || millis() - lastDataRecorderTransmission <= dataRecorderInterval) {
    return;
  }
  lastDataRecorderTransmission = millis();

  if (!horusV3Enable || !radioEnablePA) {
    return;
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: dataRecorder TX start");
  }

  setRadioPower(horusV3RadioPower);
  setRadioModulation(0);
  setRadioFrequency(horusV3FreqTable[0]);

  // Send all pages A=GPS, B=stats, C=thermal (+ D=pump, E=oif in ozone mode) back-to-back in
  // this ONE interval, refreshing GPS and the sensors between frames so each carries fresh
  // data. The whole set goes out together every dataRecorderInterval. On very short TX
  // intervals the burst can occasionally overrun a scheduled slot (that mode's window is then
  // skipped once) - that is acceptable, the recorder data is worth it.
  // 5 pages normally (A gps, B integrity, C stats, D thermal, E sat counts);
  // ozone mode adds F (pump) and G (OIF411).
  const uint8_t totalDrPages = (xdataPortMode == 3) ? 7 : 5;
  for (uint8_t page = 0; page < totalDrPages; page++) {
    // RSM4x2 (u-blox 6) has no jamming/spoofing and no per-constellation NAV-SAT, so its
    // GNSS integrity page (B) and satellite-counts page (C) carry nothing meaningful -
    // skip them entirely on that board.
    if (rsm4x2 && (page == 1 || page == 2)) continue;

    int pkt_len = buildHorusV3PacketDataRecorder(rawbuffer, page);
    if (pkt_len == 0) {
      if (xdataPortMode == 1) {
        xdataSerial.print("[error]: dataRecorder page ");
        xdataSerial.print(page);
        xdataSerial.println(" encoding failed, skipping");
      }
      continue;
    }

    int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer, (unsigned char*)rawbuffer, pkt_len);

    if (xdataPortMode == 1) {
      const char* labels[] = {"A(gps)", "B(integ)", "C(sats)", "D(stats)", "E(heat)", "F(ozone)", "G(oif)"};
      xdataSerial.print("[info]: dataRecorder ");
      xdataSerial.print(labels[page]);
      xdataSerial.print(" uncoded=");
      xdataSerial.print(pkt_len);
      xdataSerial.print("B coded=");
      xdataSerial.print(coded_len);
      xdataSerial.println("B");
    }

    radioEnableTx();
    fsk4_preamble(horusPreambleLength);
    fsk4_write(codedbuffer, coded_len);
    radioDisableTx();

    // Refresh GPS time and the sensors between every page except the last. Without this
    // the later pages reuse the timestamp captured before the first page, so a decoder
    // sees several frames with an identical time and drops them as repeats.
    if (page < totalDrPages - 1) {
      gpsHandler();
      sensorBoomHandler();
      if (pressureMode == 1 && !rpm411Error) readRPM411();
    }
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: dataRecorder TX done");
  }
}
#endif  // dataRecorder (dataRecorderTx) - both boards

#ifdef RSM4x4   // Ultra power save after landing is RSM4x4-only: the RSM4x2 (F100) build has
                // no room for it once everything else is enabled.
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

          setRadioModulation(0);
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
        unsigned long modeChangeDelayCallbackTimer = millis() + 180000;   // transmit the last position every 3 minutes

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
#endif  // RSM4x4 (ultraPowerSaveHandler)

// Update the stage code and announce it: "[stage]: XX" in mode 1, "$STG|XX" in
// mode 3 (mode 3 needs the beacon since $NFW frames only start once the scheduler runs).
void setStage(const char* code) {
  if (strcmp(nfwCurrentStage, code) == 0) return;
  memcpy(nfwCurrentStage, code, 3);
  if (xdataPortMode == 1) {
    xdataSerial.print(F("[stage]: "));
    xdataSerial.println(nfwCurrentStage);
  } else if (xdataPortMode == 3) {
    xdataSerial.print(F("$STG|"));
    xdataSerial.println(nfwCurrentStage);
  }
}

void temperatureCalibration() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Temp cal - reading sensors...");
  }

  if (autoTemperatureCalibration) {

    if (autoTemperatureCalibrationMethod == 1) {  //using constant start environment temperature
      setStage("11");
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: temp cal: method 1 (const env T)");
      }
    } else if (autoTemperatureCalibrationMethod == 2) {  //based on the PCB temperature
      setStage("12");
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: temp cal: method 2 (PCB poly)");
      }
    }

    mainTemperatureCorrectionC = 0;
    sensorBoomHandler();

    if (autoTemperatureCalibrationMethod == 1) {  //using constant start environment temperature
      mainTemperatureCorrectionC = environmentStartupAirTemperature - mainTemperatureValue;

      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: temp cal: env constant applied");
      }
    } else if (autoTemperatureCalibrationMethod == 2) {  //based on the PCB temperature
      float internalTemperature = readAvgIntTemp();
      // Plain multiplies instead of pow(x,2)/pow(x,3): identical result, and it avoids
      // pulling the (double) libm pow() into the flash-tight F100 build.
      float it2 = internalTemperature * internalTemperature;
      float it3 = it2 * internalTemperature;
      float selfHeatingCorrectedInternalTemperature = -8.5f + 1.307f * internalTemperature - 0.001461f * it2 - 0.000082f * it3;
      mainTemperatureCorrectionC = selfHeatingCorrectedInternalTemperature - mainTemperatureValue;

      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: PCB=");
        xdataSerial.print(internalTemperature);
        xdataSerial.print("C est_air=");
        xdataSerial.print(selfHeatingCorrectedInternalTemperature);
        xdataSerial.println("C");
      }
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("mainTemperatureCorrectionC = ");
      xdataSerial.print(mainTemperatureCorrectionC);
      xdataSerial.println("*C");
    }
  }
  if (autoHumidityModuleTemperatureCorrection) {  //automatically correct the humidity module readings - simple calibration
    setStage("15");

    extHeaterTemperatureCorrectionC = 0;
    sensorBoomHandler();
    extHeaterTemperatureCorrectionC = mainTemperatureValue - extHeaterTemperatureValue;

    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: extHeater cal offset = ");
      xdataSerial.print(extHeaterTemperatureCorrectionC);
      xdataSerial.println("*C");
    }
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Temp cal complete");
  }
}

void reconditioningPhase() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Reconditioning phase start");
    xdataSerial.println("[warn]: extHeater heating soon - don't touch");
  }

  setStage("20");

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
        xdataSerial.println("[err]: e220 - sensor boom error during reconditioning");
      }

      return;
    }

    extHeaterHandler(true, reconditioningTemperature, extHeaterTemperatureValue);

    if (xdataPortMode == 1) {
      xdataSerial.print("[warn]: extHeater T = ");
      xdataSerial.print(extHeaterTemperatureValue);
      xdataSerial.println("*C");
    }
  }

  extHeaterHandler(false, 0, 0);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: reconditioning done, heating off");
  }
}

void zeroHumidityCheck() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Zero-humidity cal start...");
    xdataSerial.println("[warn]: sensor heats - keep still, no wind, RH<70%, T>0C");
  }

  setStage("21");

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
      xdataSerial.println("[err]: e221 - sensor boom error before zero-humidity measurement");
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
      xdataSerial.println("[err]: e222 - wrong meas conditions (check T cal and env)");
    }

    return;
  } else {
    for (int i = 0; i < 7; i++) {
      sensorBoomHandler();
      buttonHandlerSimplified();
      interfaceHandler();
      extHeaterHandler(true, humidityCalibrationMeasurementTemperature + 15, extHeaterTemperatureValue);  // preheat straight to the hold target (~140 °C)

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

  while (measurementCount < 10) {
    buttonHandlerSimplified();
    sensorBoomHandler();
    extHeaterHandler(true, humidityCalibrationMeasurementTemperature + 15, extHeaterTemperatureValue);  // maintain ~140 °C (125 + 15)
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
        xdataSerial.println("[err]: e221 - sensor boom error during zero-humidity measurement");
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
        xdataSerial.println("/10.");
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
        xdataSerial.println("[warn]: e221 - cal timeout (unstable env or HW issue)");
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


bool temperatureCheckError = false;   // set by temperatureCheck()
bool humidityCheckError    = false;   // set by humidityCheck()

// Factory-mode start-up self-checks. Factory (Vaisala) calibration now runs on both
// board families (RSM4x2 / RSM4x1 use it exclusively), so these are compiled for both.
#if defined(RSM4x4) || defined(RSM4x2)

// temperatureCheck - verifies the main and heater temperature sensors read consistent
// values (a healthy boom at rest reads almost the same on both). Runs at start-up only,
// on a cold boom; it is NOT re-runnable from Ground Control because by then the humidity
// check may have heated the sensor. A momentary post-power-on settling can briefly exceed
// the window, so it auto-retries up to 2 extra times before flagging an error.
void temperatureCheck() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Factory temperature CHECK...");
  }
  setStage("13");

  for (int attempt = 1; attempt <= 3; attempt++) {
    sensorBoomHandler();
    buttonHandlerSimplified();
    interfaceHandler();

    if (sensorBoomMainTempError || sensorBoomHumidityModuleError) {
      temperatureCheckError = true; calibrationError = true;  // light the red fault LED
      if (xdataPortMode == 1) {
        xdataSerial.println("[err]: temperature CHECK - sensor boom error");
      }
      return;   // a boom fault is not something a retry can fix
    }

    float diff = mainTemperatureValue - extHeaterTemperatureValue;
    if (diff < 0) diff = -diff;

    if (diff <= 3.0f) {
      temperatureCheckError = false;
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: temperature CHECK passed");
      }
      return;
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("[warn]: temperature CHECK attempt "); xdataSerial.print(attempt);
      xdataSerial.print("/3 - sensors differ by "); xdataSerial.print(diff); xdataSerial.println(" C (>3)");
    }
    if (attempt < 3) delay(800);
  }

  temperatureCheckError = true; calibrationError = true;  // light the red fault LED
  if (xdataPortMode == 1) {
    xdataSerial.println("[err]: temperature CHECK FAILED after 3 attempts (>3 C)");
  }
}

// humidityCheck - runs a one-minute reconditioning phase (heats the sensor to ~138 °C),
// then holds ~135 °C and verifies a bone-dry reading (< 2 %RH). Fails if the sensor never
// exceeds 115 °C within the minute (heater/boom problem) or if the dry reading is high.
// May be re-run from Ground Control.
void humidityCheck() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Factory humidity CHECK (reconditioning)...");
    xdataSerial.println("[warn]: Humidity module heating to ~138C - don't touch sensor");
  }
  setStage("22");

  unsigned long beginMillis = millis();
  unsigned long reached115Millis = 0;
  bool reached115 = false;
  delay(500);

  // Reconditioning at ~138 C. The dwell lasts 30 s measured FROM the moment the sensor
  // first exceeds 115 C; if it never reaches 115 C within 60 s of heating, the heater /
  // boom is faulty. NOTE: these are millis() budgets, and millis() is frozen while
  // getSensorBoomFreq() runs with interrupts disabled, so the real wall-clock time is
  // somewhat longer (markedly so on the slow F100).
  while (true) {
    orangeLed(); delay(100); bothLedOff();
    sensorBoomHandler();
    buttonHandlerSimplified();
    interfaceHandler();

    if (sensorBoomHumidityModuleError) {
      extHeaterHandler(false, 0, 0);
      humidityCheckError = true; calibrationError = true;  // light the red fault LED
      if (xdataPortMode == 1) {
        xdataSerial.println("[err]: humidity CHECK - sensor boom error during reconditioning");
      }
      return;
    }

    extHeaterHandler(true, 138, extHeaterTemperatureValue);

    if (!reached115 && extHeaterTemperatureValue > 115.0f) {
      reached115 = true;
      reached115Millis = millis();   // start the 30 s reconditioning dwell
    }

    if (xdataPortMode == 1) {
      xdataSerial.print("[info]: humidity CHECK reconditioning = ");
      xdataSerial.print(extHeaterTemperatureValue); xdataSerial.println(" C");
    }

    if (reached115) {
      if (millis() - reached115Millis >= 30000) break;   // 30 s dwell after reaching 115 C
    } else if (millis() - beginMillis >= 60000) {
      break;   // never reached 115 C -> fall through to the failure check below
    }
  }

  // Must have exceeded 115 °C, otherwise the heater/boom is faulty.
  if (!reached115) {
    extHeaterHandler(false, 0, 0);
    humidityCheckError = true; calibrationError = true;  // light the red fault LED
    if (xdataPortMode == 1) {
      xdataSerial.println("[err]: humidity CHECK FAILED - sensor did not exceed 115C in 60 s");
    }
    return;
  }

  // Hold ~135 C and confirm a dry reading (< 2 %RH).
  setStage("23");
  float rhSum = 0; int rhCount = 0;
  unsigned long holdBegin = millis();
  while (rhCount < 5 && millis() - holdBegin < 10000) {
    sensorBoomHandler();
    extHeaterHandler(true, 135, extHeaterTemperatureValue);
    buttonHandlerSimplified();
    interfaceHandler();

    if (extHeaterTemperatureValue > 115.0f && !sensorBoomHumidityModuleError) {
      rhSum += humidityValue; rhCount++;
    }
    orangeLed(); delay(80); bothLedOff();
  }
  extHeaterHandler(false, 0, 0);

  float dryRH = (rhCount > 0) ? (rhSum / rhCount) : 100.0f;
  if (dryRH > 2.0f) {
    humidityCheckError = true; calibrationError = true;  // light the red fault LED
    if (xdataPortMode == 1) {
      xdataSerial.print("[err]: humidity CHECK FAILED - dry reading ");
      xdataSerial.print(dryRH); xdataSerial.println(" %RH (>2)");
    }
  } else {
    humidityCheckError = false;
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: humidity CHECK passed");
    }
  }
}
#endif

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void flightHeatingHandler() {
  // Thermal control only needs to run about once a second - the heater/PCB thermal time
  // constants are seconds long. Running it every scheduler loop (~100x/s) did a blocking
  // thermistor read and printed three log lines each time, which flooded the link and
  // slowed the loop enough to starve GPS reads and destabilise the clock.
  static unsigned long _lastHeat = 0;
  if (millis() - _lastHeat < 1000UL) return;
  _lastHeat = millis();

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
      xdataSerial.print("[info]: RefHeat T=");
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

// MCU die temperature from the STM32 internal temperature sensor (both boards).
// The two STM32 families behave very differently, so each is handled correctly:
//   * STM32L4 (RSM4x4): the sensor voltage RISES with temperature (positive slope) and
//     the chip ships with two factory calibration points (TS_CAL1 @30C, TS_CAL2 @130C,
//     both taken at VDDA = 3.0 V). We interpolate between them and normalise the live
//     reading to that 3.0 V reference via VREFINT - this is the accurate, no-guess method.
//   * STM32F1 (RSM4x2): no factory cal; the sensor voltage FALLS with temperature, so we
//     use the classic (V25 - Vsense)/avg_slope formula with cpuTempSensorVoltageAt25degC.
// Falls back to the board-temperature proxy if the internal channel is not exposed.
float readMcuTemperature() {
#if defined(ATEMP)
  float tsRaw = (float)analogRead(ATEMP);
  #if defined(RSM4x4) && defined(TEMPSENSOR_CAL1_ADDR) && defined(TEMPSENSOR_CAL2_ADDR)
    const uint16_t c1 = *TEMPSENSOR_CAL1_ADDR;
    const uint16_t c2 = *TEMPSENSOR_CAL2_ADDR;
    if (c2 == c1) return (float)readAvgIntTemp();
    #if defined(AVREF) && defined(VREFINT_CAL_ADDR)
      uint32_t vrefRaw = analogRead(AVREF);
      if (vrefRaw > 0) tsRaw = tsRaw * (float)(*VREFINT_CAL_ADDR) / (float)vrefRaw;  // normalise to the 3.0 V cal supply
    #endif
    return (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * (tsRaw - (float)c1) / (float)(c2 - c1) + (float)TEMPSENSOR_CAL1_TEMP;
  #else
    float vSense = (tsRaw / 1024.0f) * 3.0f;   // F100: 10-bit ADC, regulated 3.0 V
    return (cpuTempSensorVoltageAt25degC - vSense) / 0.0043f + 25.0f;  // negative slope
  #endif
#else
  return (float)readAvgIntTemp();   // internal channel not exposed by the core for this variant
#endif
}

int readAvgIntTemp() {
  int radioTemp = static_cast<int>(readRadioTemp());
  int thermistorTemp = static_cast<int>(readThermistorTemp());

  if (abs(radioTemp) > 120) {  //in case of error
    radioTemp = thermistorTemp;
  } else if (abs(thermistorTemp) > 150) {
    thermistorTemp = radioTemp;
  }

  long sum = (long)radioTemp + thermistorTemp;
  int count = 2;

  if (pressureMode == 1 && !rpm411Error) {
    sum += static_cast<int>(rpm411InternalTemperature);
    count++;
  }

#if defined(ATEMP)
  // Include the MCU die temperature in the board-temperature average (it sits in the same
  // enclosure and tracks the PCB closely). Guarded by ATEMP so readMcuTemperature() never
  // falls back to readAvgIntTemp() here, which would recurse.
  int mcuTemp = static_cast<int>(readMcuTemperature());
  if (abs(mcuTemp) < 150) { sum += mcuTemp; count++; }
#endif

  return static_cast<int>(sum / count);
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

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void humidityModuleHeaterPowerControl(unsigned int heaterPower) {  //0 - OFF, 1-255 - only low power heater PWM, 256-500 - low power heater at max and high power heater PWM-controlled
  extHeaterPwmStatus = heaterPower;

  if (xdataPortMode == 1) {
    xdataSerial.print("[info]: extHeater pwr ");
    xdataSerial.print(heaterPower);
    xdataSerial.println("/500");
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

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
void gpsQuietMode() {
    if (xdataPortMode == 1) xdataSerial.println(F("[info]: Entering GPS Quiet Mode"));

    setStage("31");

    unsigned long startQuietMillis = millis();
    unsigned long lastUpdate = startQuietMillis;
    unsigned long fixHeldSince = 0;   // millis() when the current continuous fix began (0 = no fix yet)

    // Stay quiet until it is cancelled or the overall silence window elapses. Normally we
    // leave once a fix is acquired (gpsSats >= 4), but improvedGpsHoldAfterFix keeps us
    // quiet for that much longer after the fix so it can settle and gather more satellites
    // before the radio resumes. Losing the fix restarts that hold.
    while ((millis() - startQuietMillis < radioSilenceDuration) && !cancelGpsImprovement) {

        if (gpsSats >= 4) {
            if (fixHeldSince == 0) {
                fixHeldSince = millis();
                if (improvedGpsHoldAfterFix > 0 && xdataPortMode == 1) {
                    xdataSerial.print(F("[info]: fix acquired, holding quiet "));
                    xdataSerial.print(improvedGpsHoldAfterFix / 1000);
                    xdataSerial.println(F("s more"));
                }
            }
            if (millis() - fixHeldSince >= improvedGpsHoldAfterFix) break;   // fix held long enough
        } else {
            fixHeldSince = 0;   // no fix (or lost it) - restart the post-fix hold
        }

        unsigned long now = millis();
        unsigned long elapsed = now - lastUpdate;

        // Keep sch_sysMs advancing while waiting
        if (elapsed > 0) {
            sch_sysMs += elapsed;
            lastUpdate = now;
        }

        if (sensorBoomFault || calibrationError || rpm411Error || vBatWarn) {
          redLed();
        } else {
          orangeLed();
        }

        // Run handlers to keep system responsive
        gpsHandler();
        ozoneHandler();
        buttonHandler();
        interfaceHandler();
        sensorBoomHandler();
        flightHeatingHandler();
        pressureHandler();
    }

    if (gpsSats >= 4) setStage("32");

    if (xdataPortMode == 1) xdataSerial.println(F("[info]: Exiting GPS Quiet Mode"));

    sch_lastMillis = millis();
}

static void sch_tickTime() {
  unsigned long now = millis();
  sch_sysMs += (now >= sch_lastMillis) ? (now - sch_lastMillis) : now;
  sch_lastMillis = now;
}

static unsigned long sch_nextSlot(unsigned long nowMs, uint16_t periodSec, uint16_t offsetSec) {
  if (periodSec == 0) return 0xFFFFFFFFUL;
  const unsigned long pMs = (unsigned long)periodSec * 1000UL;
  const unsigned long oMs = (unsigned long)offsetSec * 1000UL;
  unsigned long base = (nowMs >= oMs) ? (nowMs - oMs) : 0UL;
  unsigned long next = (base / pMs) * pMs + oMs;
  if (next <= nowMs) next += pMs;
  return next;
}

// NOTE on minimum spacing: after a mode transmits we reschedule it to the plain next
// grid slot (sch_nextSlot). We used to push that slot a whole period forward whenever
// it fell less than half a period after the *current time*, to avoid a double-fire.
// But "current time" is measured after the transmission, so a long TX - or a second
// mode sharing the same slot pushing the clock forward - made a perfectly valid next
// slot look "too close" and got skipped, dropping roughly every other transmission.
// The real double-fire case (a GPS clock jump landing a slot right after a TX) is
// already caught by the millis()-based minimum-interval guard in checkMode below,
// which is immune to clock adjustments, so no schedule-time skipping is needed.

// Milliseconds from now until the nearest enabled scheduled transmission. Returns
// 0xFFFFFFFF when nothing is scheduled. Used by long blocking work (the data recorder)
// to check, with a fresh clock tick, whether a slot is close enough that it should hold
// off / yield rather than overrun it.
unsigned long sch_msToNextTx() {
  sch_tickTime();
  unsigned long nearest = 0xFFFFFFFFUL;
  auto f = [&](bool en, unsigned long nxt) {
    if (en && nxt != 0 && nxt != 0xFFFFFFFFUL && nxt < nearest) nearest = nxt;
  };
  f(pipEnable,     sch_nextPipMs);
  f(horusV3Enable, sch_nextHorusV3Ms);
  #ifdef RSM4x4
  f(horusEnable,   sch_nextHorusMs);
  #endif
  f(aprsEnable,    sch_nextAprsMs);
  #ifdef RSM4x4
  f(rttyEnable,    sch_nextRttyMs);
  #endif
  f(morseEnable,   sch_nextMorseMs);
  if (nearest == 0xFFFFFFFFUL) return 0xFFFFFFFFUL;
  return (nearest > sch_sysMs) ? (nearest - sch_sysMs) : 0UL;
}

static void sch_syncGps() {
  // The clock depends on valid GPS *time*, not on the position sat count: a transmission
  // briefly desensitises the receiver so the first read after it can momentarily show 0
  // sats while the UTC time is still perfectly valid. Keying sync-lost off sat count made
  // that transient drop the clock. So we only drop sync if the time itself goes invalid or
  // stale. Age tolerance is 6 s because one transmission blocks for up to ~4-5 s with no
  // read; the clock free-runs accurately on millis() meanwhile.
  if (!gps.time.isValid() || gps.time.age() > 6000) {
    if (sch_gpsSynced && xdataPortMode == 1)
      xdataSerial.println(F("[sch]: GPS sync lost"));
    sch_gpsSynced = false;
    return;
  }

  // Sub-second-precise GPS time of day, in ms. The whole-second UTC is refined by the
  // NAV-PVT/NAV-TIMEUTC nanosecond fraction, then extrapolated from the fix epoch to NOW
  // by adding the fix age (the fix may be up to a second or two old between reads). This
  // is what makes a genuine sub-second clock lock possible - the old code only had
  // whole-second time, so slots could never land closer than +/-1 s.
  long gpsTodMs =
      (long)gps.time.hour()   * 3600000L +
      (long)gps.time.minute() * 60000L   +
      (long)gps.time.second() * 1000L    +
      gps.timeNano / 1000000L +                  // ns -> ms (signed sub-second fraction)
      (long)gps.time.age();                      // extrapolate from fix epoch to now
  while (gpsTodMs < 0)          gpsTodMs += 86400000L;
  while (gpsTodMs >= 86400000L) gpsTodMs -= 86400000L;

  if (!sch_everSeeded) {
    // First fix ever: seed the scheduler clock straight to the GPS time-of-day. Seeding
    // absolutely (rather than applying the full diff against the tiny since-boot value)
    // keeps sch_sysMs small and correct. Applying that first diff used to underflow the
    // unsigned counter past zero; because 2^32 ms is not a whole number of days, the
    // time-of-day reading then stayed corrupted and the clock looped forever issuing
    // ~25,000,000 ms "large adj" corrections, wrecking the TX schedule.
    sch_sysMs = gpsTodMs;
    sch_lastMillis = millis();
    sch_nextPipMs = sch_nextHorusV3Ms = sch_nextHorusMs =
    sch_nextAprsMs = sch_nextRttyMs = sch_nextMorseMs = 0;
    sch_everSeeded = true;
    sch_gpsSynced  = true;
    if (xdataPortMode == 1) {
      xdataSerial.print(F("[sch]: GPS synced UTC "));
      if (gps.time.hour()   < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.hour());   xdataSerial.print(':');
      if (gps.time.minute() < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.minute()); xdataSerial.print(':');
      if (gps.time.second() < 10) xdataSerial.print('0'); xdataSerial.println(gps.time.second());
    }
    return;
  }

  // Already seeded once. This path also handles re-acquisition after a GPS loss: while the
  // fix was gone the clock free-ran on the MCU millis() timer, so apply only an incremental
  // correction. sch_sysMs is already large, so it cannot underflow here.
  long diffMs = (long)gpsTodMs - (long)(sch_sysMs % 86400000UL);
  if (diffMs >  43200000L) diffMs -= 86400000L;
  if (diffMs < -43200000L) diffMs += 86400000L;

  if (labs(diffMs) >= 2000) {
    sch_sysMs = (unsigned long)((long)sch_sysMs + diffMs);
    sch_lastMillis = millis();
    sch_nextPipMs = sch_nextHorusV3Ms = sch_nextHorusMs =
    sch_nextAprsMs = sch_nextRttyMs = sch_nextMorseMs = 0;
    if (xdataPortMode == 1) {
      xdataSerial.print(F("[sch]: GPS large adj ")); xdataSerial.print(diffMs);
      xdataSerial.println(F("ms -> rescheduled"));
    }
  } else if (labs(diffMs) >= 30) {
    // Small drift correction. With sub-second GPS time this keeps the clock tightly
    // locked; the 30 ms deadband just avoids churning on measurement jitter.
    sch_sysMs = (unsigned long)((long)sch_sysMs + diffMs);
    sch_lastMillis = millis();
  }

  if (!sch_gpsSynced) {
    // GPS came back after a dropout - just note it; the clock kept running on the MCU.
    sch_gpsSynced = true;
    if (xdataPortMode == 1) {
      xdataSerial.print(F("[sch]: GPS resynced UTC "));
      if (gps.time.hour()   < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.hour());   xdataSerial.print(':');
      if (gps.time.minute() < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.minute()); xdataSerial.print(':');
      if (gps.time.second() < 10) xdataSerial.print('0'); xdataSerial.println(gps.time.second());
    }
  }
}

void schedulerInit() {
  sch_lastMillis = millis();
  sch_sysMs      = 0;
  sch_gpsSynced  = false;
  sch_everSeeded = false;
  sch_nextPipMs = sch_nextHorusV3Ms = sch_nextHorusMs =
  sch_nextAprsMs = sch_nextRttyMs = sch_nextMorseMs = 0;
  sch_lastSensorBoom = sch_lastPressure = sch_lastInterface = 0;
  sch_lastGps = sch_lastOzone = 0;

  // Select simple fast-TX mode from the shortest enabled DATA-mode interval. Pip is not a
  // data packet (it is a tiny beacon burst), so a short Pip interval alone does not switch
  // the whole sonde into fast mode - but Pip is still sent each cycle when fast mode is on.
  uint16_t fastIv = 0xFFFF;
  if (horusV3Enable && horusV3TimeSyncSeconds < FAST_TX_MIN_SYNC_SECONDS && horusV3TimeSyncSeconds < fastIv) fastIv = horusV3TimeSyncSeconds;
  if (horusEnable   && horusTimeSyncSeconds   < FAST_TX_MIN_SYNC_SECONDS && horusTimeSyncSeconds   < fastIv) fastIv = horusTimeSyncSeconds;
  if (aprsEnable    && aprsTimeSyncSeconds    < FAST_TX_MIN_SYNC_SECONDS && aprsTimeSyncSeconds    < fastIv) fastIv = aprsTimeSyncSeconds;
  if (rttyEnable    && rttyTimeSyncSeconds    < FAST_TX_MIN_SYNC_SECONDS && rttyTimeSyncSeconds    < fastIv) fastIv = rttyTimeSyncSeconds;
  if (morseEnable   && morseTimeSyncSeconds   < FAST_TX_MIN_SYNC_SECONDS && morseTimeSyncSeconds   < fastIv) fastIv = morseTimeSyncSeconds;
  fastTxMode    = (fastIv != 0xFFFF);
  fastTxDelayMs = fastTxMode ? ((unsigned long)fastIv * 1000UL) : 0;

  if (xdataPortMode == 1 && fastTxMode) {
    xdataSerial.print(F("[sch]: simple fast-TX mode - delay "));
    xdataSerial.print(fastTxDelayMs);
    xdataSerial.println(F(" ms between cycles, no GPS-clock sync"));
  }
}

void schedulerLoop() {

  if (xdataPortMode == 1) {
    static unsigned long _hbMs = 0;
    if (millis() - _hbMs >= 60000UL) {
      _hbMs = millis();
      xdataSerial.print(F("[info]: up="));
      xdataSerial.print(millis() / 60000UL);
      xdataSerial.print(F("m sats="));
      xdataSerial.print(gpsSats);
      xdataSerial.print(F(" bat="));
      xdataSerial.print(readBatteryVoltage(), 2);
      xdataSerial.print(F("V T="));
      xdataSerial.print(mainTemperatureValue, 1);
      xdataSerial.print(F("C H="));
      xdataSerial.print(humidityValue);   // uint16_t: no digits arg (print(int,0) would emit a raw byte)
      if (err) {
        xdataSerial.print(F("% ERR"));
        if (sensorBoomFault)  xdataSerial.print(F(" sensorBoom"));
        if (calibrationError) xdataSerial.print(F(" calibration"));
        if (rpm411Error)      xdataSerial.print(F(" rpm411"));
        if (vBatWarn)         xdataSerial.print(F(" vBat"));
        xdataSerial.println();
      } else {
        xdataSerial.println(F("% OK"));
      }
      serialStatusHandler();
    }
  }

  sch_tickTime();
  sch_syncGps();

  // Only enter the (blocking) radio-quiet acquisition mode if the sat count stays low for
  // a few seconds - not on the momentary 0-sat glitch right after a transmission, which
  // recovers within a read or two and must not disrupt the schedule.
  static unsigned long _lowSatsSince = 0;
  if (gpsSats < 4) { if (_lowSatsSince == 0) _lowSatsSince = millis(); }
  else _lowSatsSince = 0;
  bool sustainedLowSats = (_lowSatsSince != 0 && (millis() - _lowSatsSince) > 3000UL);

  if (improvedGpsPerformance && !cancelGpsImprovement && sustainedLowSats) {
    gpsQuietMode();
    sch_tickTime();
  }

  // ===== Simple fast-TX mode (no GPS-clock scheduling) =====
  // Refresh GPS + sensors, transmit every enabled mode back-to-back, then wait the
  // configured delay. Chosen when a data mode's interval is set below 5 s (see schedulerInit).
  if (fastTxMode) {
    gpsHandler();        sch_lastGps = millis();

    if (sensorBoomEnable) {
      // Every cycle, unless boom power saving is on - then only every its interval.
      if (!sensorBoomPowerSaving || (millis() - sch_lastSensorBoom) >= sensorBoomPowerSavingInterval) {
        sensorBoomHandler(); sch_lastSensorBoom = millis();
      }
    }
    pressureHandler();   sch_lastPressure = millis();
    ozoneHandler();
    interfaceHandler();  sch_lastInterface = millis();   // keep $NFW telemetry / Ground Control alive

    if (pipEnable)     { pipTx();     }
    if (horusV3Enable) { horusV3Tx(); }
    #ifdef RSM4x4
    if (horusEnable)   { horusTx();   }
    #endif
    if (aprsEnable)    { aprsTx();    }
    #ifdef RSM4x4
    if (rttyEnable)    { rttyTx();    }
    #endif
    if (morseEnable)   { morseTx();   }

    // Keep the essential periodic handlers running (same set the normal path runs).
    xdataCmdDrain();
    deviceStatusHandler();
    flightComputing();
    buttonHandler();
    powerHandler();
    initRecorderData();
    flightHeatingHandler();
    #ifdef RSM4x4
    ultraPowerSaveHandler();
    #endif
    autoResetHandler();

    if (fastTxDelayMs) delay(fastTxDelayMs);
    return;
  }

  unsigned long nowMs = sch_sysMs;
  if (pipEnable     && sch_nextPipMs     == 0) sch_nextPipMs     = sch_nextSlot(nowMs, pipTimeSyncSeconds,     pipTimeSyncOffsetSeconds);
  if (horusV3Enable && sch_nextHorusV3Ms == 0) sch_nextHorusV3Ms = sch_nextSlot(nowMs, horusV3TimeSyncSeconds, horusV3TimeSyncOffsetSeconds);
  #ifdef RSM4x4
  if (horusEnable   && sch_nextHorusMs   == 0) sch_nextHorusMs   = sch_nextSlot(nowMs, horusTimeSyncSeconds,   horusTimeSyncOffsetSeconds);
  #endif
  if (aprsEnable    && sch_nextAprsMs    == 0) sch_nextAprsMs    = sch_nextSlot(nowMs, aprsTimeSyncSeconds,    aprsTimeSyncOffsetSeconds);
  #ifdef RSM4x4
  if (rttyEnable    && sch_nextRttyMs    == 0) sch_nextRttyMs    = sch_nextSlot(nowMs, rttyTimeSyncSeconds,    rttyTimeSyncOffsetSeconds);
  #endif
  if (morseEnable   && sch_nextMorseMs   == 0) sch_nextMorseMs   = sch_nextSlot(nowMs, morseTimeSyncSeconds,   morseTimeSyncOffsetSeconds);

  unsigned long minToTxMs = 0xFFFFFFFFUL;
  {
    auto track = [&](bool en, unsigned long nxt) {
      if (!en || nxt == 0 || nxt == 0xFFFFFFFFUL) return;
      unsigned long d = (nxt > nowMs) ? (nxt - nowMs) : 0UL;
      if (d < minToTxMs) minToTxMs = d;
    };
    track(pipEnable,     sch_nextPipMs);
    track(horusV3Enable, sch_nextHorusV3Ms);
    #ifdef RSM4x4
    track(horusEnable,   sch_nextHorusMs);
    #endif
    track(aprsEnable,    sch_nextAprsMs);
    #ifdef RSM4x4
    track(rttyEnable,    sch_nextRttyMs);
    #endif
    track(morseEnable,   sch_nextMorseMs);
  }

  {
    unsigned long hw       = millis();
    unsigned long boomAge  = hw - sch_lastSensorBoom;
    unsigned long pressAge = hw - sch_lastPressure;
    unsigned long ifaceAge = hw - sch_lastInterface;

    const bool txImminent = (minToTxMs < 2000UL);
    const bool boomStale  = (boomAge  > 10000UL);
    const bool pressStale = (pressAge > 10000UL);
    const bool ifaceStale = (ifaceAge > 10000UL);

    // Two independent sensor-boom read policies:
    //   power saving OFF - CPU/freshness policy: read whenever there is spare time
    //     (between transmissions, as often as possible). If the scheduler has been too
    //     busy to do that, boomStale forces a read once the data is older than 10s.
    //   power saving ON  - energy policy: read strictly every
    //     sensorBoomPowerSavingInterval (default 30s), independent of CPU load and TX
    //     timing. This deliberately accepts older data to spend less on the boom.
    if (sensorBoomPowerSaving) {
      if (boomAge >= sensorBoomPowerSavingInterval) {
        sensorBoomHandler();
        sch_lastSensorBoom = millis();
      }
    } else {
      if (!txImminent || boomStale) {
        sensorBoomHandler();
        sch_lastSensorBoom = millis();
      }
    }

    if (!txImminent || pressStale) {
      pressureHandler();
      sch_lastPressure = millis();
    }

    if ((!txImminent && ifaceAge > 3000UL) || ifaceStale) {
      interfaceHandler();
      sch_lastInterface = millis();
    }
  }

  // GPS (~1.2 s) and ozone (~1.15 s) are the two long blocking reads, and the data
  // recorder burst is even longer. Running any of them right before a scheduled slot is
  // what pushed transmissions a few seconds off time. Gate them like the sensor boom
  // above: while a TX is imminent, skip the long work so the dispatch below hits the
  // slot precisely, but a staleness fallback still forces the read so data never goes
  // cold. GPS keeps a short 2 s freshness floor (it drives the clock sync); ozone uses
  // the "every 15 s if there is no spare time" rule, matching the Horus cadence.
  {
    const bool txImminent = (minToTxMs < 2500UL);
    const unsigned long hw2    = millis();
    const unsigned long gpsAge = hw2 - sch_lastGps;
    const unsigned long ozAge  = hw2 - sch_lastOzone;

    // The data recorder is the longest blocking job (several pages + a GPS/boom refresh
    // between each). Only begin a burst when the next scheduled transmission is clearly
    // far off, using a fresh clock tick rather than the stale minToTxMs computed above
    // (the sensor/pressure reads in between have already eaten into it). It also yields
    // between pages, so this just keeps it from starting too close to a slot.
    if (sch_msToNextTx() > 5000UL) {
      dataRecorderTx();   // dataRecorder on both boards (RSM4x4 + RSM4x2); self-gated by its own interval
    }

    // The full gpsHandler() is heavy: it runs GPSManagement() (which can send ACK-waited
    // tier/constellation config), polls, an ~800 ms read and a NAV-SAT drain - together
    // easily a few seconds. It must NEVER run right before a slot, or the transmission
    // lands seconds late. So run it only when no TX is imminent. When one is imminent but
    // the fix is getting stale, do only a quick non-blocking UART drain (no config, no
    // ACK waits, no long read) so position/time stay fresh without blocking the slot.
    if (!txImminent) {
      gpsHandler();
      sch_lastGps = millis();
    } else if (gpsOperationMode != 0 && gpsAge > 2000UL) {
      while (gpsSerial.available()) gps.encode((uint8_t)gpsSerial.read());
    }

    if (!txImminent || ozAge > 15000UL) {
      ozoneHandler();     // returns immediately when xdataPortMode != 3
      sch_lastOzone = millis();
    }
  }
  xdataCmdDrain();
  deviceStatusHandler();
  flightComputing();
  buttonHandler();
  powerHandler();
  initRecorderData();
  flightHeatingHandler();
  #ifdef RSM4x4
  ultraPowerSaveHandler();
  #endif
  autoResetHandler();


  {
    sch_tickTime();
    nowMs = sch_sysMs;

    unsigned long nearestMs = 0xFFFFFFFFUL;
    auto findNearest = [&](bool en, unsigned long nxt) {
      if (!en || nxt == 0 || nxt == 0xFFFFFFFFUL) return;
      if (nxt < nearestMs) nearestMs = nxt;
    };
    findNearest(pipEnable,     sch_nextPipMs);
    findNearest(horusV3Enable, sch_nextHorusV3Ms);
    #ifdef RSM4x4
    findNearest(horusEnable,   sch_nextHorusMs);
    #endif
    findNearest(aprsEnable,    sch_nextAprsMs);
    #ifdef RSM4x4
    findNearest(rttyEnable,    sch_nextRttyMs);
    #endif
    findNearest(morseEnable,   sch_nextMorseMs);

    if (nearestMs != 0xFFFFFFFFUL && nearestMs > nowMs) {
      unsigned long toWait = nearestMs - nowMs;
      // Matches the 2500 ms gating window above: once the long reads are being skipped,
      // busy-wait precisely to the slot so the TX lands on time instead of a loop late.
      if (toWait < 2500UL) {
        if (xdataPortMode == 1) {
          xdataSerial.print(F("[sch]: pre-TX wait ")); xdataSerial.print(toWait); xdataSerial.println(F("ms"));
        }
        unsigned long bailAt = millis() + toWait + 1500UL;
        while (sch_sysMs < nearestMs) {
          sch_tickTime();
          if (millis() > bailAt) break;
          // Keep draining the GPS UART through the wait so the streamed NAV-PVT keeps
          // gps.time fresh (a byte drain, not the full blocking gpsHandler()).
          if (gpsOperationMode != 0) { while (gpsSerial.available()) gps.encode((uint8_t)gpsSerial.read()); }
          // Do the packet prep DURING the wait, finishing before the slot, so the slot
          // itself just fires the radio. Refresh the sensor boom + pressure while the slot
          // is still >=1 s away (the boom read blocks a few hundred ms); using the same 2 s
          // freshness the dispatch checks means the dispatch then skips it, so the TX is no
          // longer delayed ~1 s by a boom read landing right on the slot. Keep Ground Control
          // alive by sending the $NFW frame (short) up to ~300 ms before the slot - without
          // this the interface froze for the whole wait. sch_tickTime() after each blocking
          // call keeps the slot maths honest.
          if (nearestMs > sch_sysMs + 1000UL && (millis() - sch_lastSensorBoom) > 2000UL) {
            sensorBoomHandler(); sch_lastSensorBoom = millis();
            pressureHandler();   sch_lastPressure   = millis();
            sch_tickTime();
          }
          if (nearestMs > sch_sysMs + 300UL && (millis() - sch_lastInterface) > 400UL) {
            interfaceHandler(); sch_lastInterface = millis(); sch_tickTime();
          }
          buttonHandler();
          delayMicroseconds(200);
        }
      }
    }
  }

  {
    bool anyDone;
    // When several modes share a slot they are sent back to back in this loop. Refresh
    // the sensor boom / pressure only once, before the first of them, instead of before
    // every transmission: a redundant boom read (which briefly disables interrupts) was
    // adding a couple of seconds between, e.g., the Horus and APRS packets of the same
    // slot, so APRS landed noticeably after its window.
    bool burstRefreshed = false;
    do {
      anyDone = false;
      sch_tickTime();
      nowMs = sch_sysMs;

      unsigned long pickMs  = 0xFFFFFFFFUL;
      int           pickIdx = -1;  // 0=pip 1=horusV3 2=horus 3=aprs 4=rtty 5=morse

      auto checkMode = [&](bool en, unsigned long &nxt, uint16_t per, uint16_t off, int idx) {
        if (!en || nxt == 0 || nxt == 0xFFFFFFFFUL) return;
        if (nowMs < nxt) return;
        // Minimum-interval guard, measured in millis() so it survives a GPS clock
        // re-alignment: never let a mode fire again less than half its period after it
        // last actually transmitted (which would double-transmit, e.g. APRS at xx:00 and
        // again ~10 s later). Crucially we HOLD the slot here rather than skip it: leaving
        // nxt untouched lets the mode fire the instant the minimum spacing is met, one
        // loop later, so spacing is enforced without ever dropping a transmission. The
        // old code rescheduled to the next grid slot here, which is what made a mode
        // transmit only every second interval whenever this guard tripped.
        if (sch_lastTxHw[idx] != 0 && (millis() - sch_lastTxHw[idx]) < ((unsigned long)per * 500UL)) {
          return;
        }
        long overdue  = (long)(nowMs - nxt);
        long periodMs = (long)per * 1000L;
        if (overdue >= periodMs) {
          if (xdataPortMode == 1) {
            xdataSerial.print(F("[sch]: skip (late ")); xdataSerial.print(overdue / 1000); xdataSerial.println(F("s)"));
          }
          nxt = sch_nextSlot(nowMs, per, off);
          anyDone = true;
        } else if (nxt < pickMs) {
          pickMs  = nxt;
          pickIdx = idx;
        }
      };

      checkMode(pipEnable,     sch_nextPipMs,     pipTimeSyncSeconds,     pipTimeSyncOffsetSeconds,     0);
      checkMode(horusV3Enable, sch_nextHorusV3Ms, horusV3TimeSyncSeconds, horusV3TimeSyncOffsetSeconds, 1);
      #ifdef RSM4x4
      checkMode(horusEnable,   sch_nextHorusMs,   horusTimeSyncSeconds,   horusTimeSyncOffsetSeconds,   2);
      #endif
      checkMode(aprsEnable,    sch_nextAprsMs,    aprsTimeSyncSeconds,    aprsTimeSyncOffsetSeconds,    3);
      #ifdef RSM4x4
      checkMode(rttyEnable,    sch_nextRttyMs,    rttyTimeSyncSeconds,    rttyTimeSyncOffsetSeconds,    4);
      #endif
      checkMode(morseEnable,   sch_nextMorseMs,   morseTimeSyncSeconds,   morseTimeSyncOffsetSeconds,   5);

      if (pickIdx >= 0) {
        if (!burstRefreshed) {
          if ((millis() - sch_lastSensorBoom) > 2000UL) {
            sensorBoomHandler(); sch_lastSensorBoom = millis();
          }
          if ((millis() - sch_lastPressure) > 2000UL) {
            pressureHandler(); sch_lastPressure = millis();
          }
          burstRefreshed = true;
        }

        if (xdataPortMode == 1) {
          const char* modeNames[] = {"PIP","HorusV3","HorusV2","APRS","RTTY","Morse"};
          xdataSerial.print(F("[sch]: TX ")); xdataSerial.println(modeNames[pickIdx]);
        }

        switch (pickIdx) {
          case 0: pipTx();     sch_tickTime(); sch_nextPipMs     = sch_nextSlot(sch_sysMs, pipTimeSyncSeconds,     pipTimeSyncOffsetSeconds);     break;
          case 1: horusV3Tx(); sch_tickTime(); sch_nextHorusV3Ms = sch_nextSlot(sch_sysMs, horusV3TimeSyncSeconds, horusV3TimeSyncOffsetSeconds); break;
          #ifdef RSM4x4
          case 2: horusTx();   sch_tickTime(); sch_nextHorusMs   = sch_nextSlot(sch_sysMs, horusTimeSyncSeconds,   horusTimeSyncOffsetSeconds);   break;
          #endif
          case 3: aprsTx();    sch_tickTime(); sch_nextAprsMs    = sch_nextSlot(sch_sysMs, aprsTimeSyncSeconds,    aprsTimeSyncOffsetSeconds);    break;
          #ifdef RSM4x4
          case 4: rttyTx();    sch_tickTime(); sch_nextRttyMs    = sch_nextSlot(sch_sysMs, rttyTimeSyncSeconds,    rttyTimeSyncOffsetSeconds);    break;
          #endif
          case 5: morseTx();   sch_tickTime(); sch_nextMorseMs   = sch_nextSlot(sch_sysMs, morseTimeSyncSeconds,   morseTimeSyncOffsetSeconds);   break;
        }
        sch_lastTxHw[pickIdx] = millis();   // for the minimum-interval guard in checkMode

        // A transmission blocks for seconds, during which the GPS UART fills with a
        // backlog of NAV-PVT frames. Reading those FIFO would feed the clock timestamps
        // that are already seconds old, making it lurch (the large-adj oscillation and
        // late transmissions). Discard the backlog and reset the parser so the next read
        // gets a current fix, keeping the clock locked.
        if (gpsOperationMode != 0) {
          while (gpsSerial.available()) gpsSerial.read();
          gps.resetParse();
        }

        if (xdataPortMode == 1) {
          const char* modeNames[] = {"PIP","HorusV3","HorusV2","APRS","RTTY","Morse"};
          xdataSerial.print(F("[sch]: done ")); xdataSerial.println(modeNames[pickIdx]);
        }
        anyDone = true;
      }

    } while (anyDone);
  }

}

// Runs a GCS command found in a line (from "CMD:" onward). Shared by xdataCmdDrain
// (mode 1) and the ozone parser (mode 3). Returns true if a command matched.
bool runXdataCommand(const char* line) {
  const char* cmd = strstr(line, "CMD:");
  if (cmd == NULL) return false;

  if (strncmp(cmd, "CMD:REBOOT", 10) == 0) {
    xdataSerial.println(F("[info]: Rebooting on command..."));
    delay(100);
    NVIC_SystemReset();
  } else if (strncmp(cmd, "CMD:SHUTDOWN", 12) == 0) {
    hardwarePowerShutdown();
  } else if (strncmp(cmd, "CMD:RECONDITION", 15) == 0 && sensorBoomEnable) {
    xdataSerial.println(F("[info]: Starting reconditioning..."));
    reconditioningPhase();
#if defined(RSM4x4)
  // NFW-calibration commands: compiled on RSM4x4 / RSM4x5 only. RSM4x2 / RSM4x1 use
  // factory (Vaisala) calibration, so the NFW calibration routines are not built there.
  } else if (strncmp(cmd, "CMD:ZEROHUM", 11) == 0 && sensorBoomEnable && humidityModuleEnable) {
    xdataSerial.println(F("[info]: Starting zero-humidity calibration..."));
    zeroHumidityCheck();
    maxHumidityCapacitance = zeroHumidityCapacitance + humidityCapacitanceRangeDelta;
  } else if (strncmp(cmd, "CMD:TEMPCAL", 11) == 0 && sensorBoomEnable) {
    xdataSerial.println(F("[info]: Starting temperature calibration..."));
    temperatureCalibration();
  } else if (strncmp(cmd, "CMD:HUMDEBUG", 12) == 0 && sensorBoomEnable) {
    xdataSerial.println(F("[info]: Starting humidity range debug..."));
    char _prevStage[4];
    memcpy(_prevStage, nfwCurrentStage, sizeof(_prevStage));
    humidityCalibrationDebug = true;
    humidityDeltaCalibrationDebug();
    humidityCalibrationDebug = false;
    setStage(_prevStage);   // restore the pre-debug stage so the stage leaves 25/26 and Ground Control can reopen the dialog next time
#endif
  // Factory humidity check - re-runnable on both board families. The temperature check is
  // start-up only: by now the humidity check may have heated the boom, so a fresh comparison is invalid.
  } else if (strncmp(cmd, "CMD:HUMCHECK", 12) == 0 && sensorBoomEnable && humidityModuleEnable) {
    xdataSerial.println(F("[info]: Starting humidity CHECK..."));
    humidityCheck();
  } else if (strncmp(cmd, "CMD:STOP", 8) == 0) {
    _hrdStopRequested = true;
    xdataSerial.println(F("[info]: Stop requested."));
  } else {
    return false;
  }
  return true;
}

// Mode-1 command drain (mode 3 drains the shared RX stream in ozoneHandler instead).
void xdataCmdDrain() {
  if (xdataPortMode != 1) return;
  static char _cmdBuf[40];
  static uint8_t _cmdLen = 0;
  while (xdataSerial.available()) {
    char c = (char)xdataSerial.read();
    if (c == '\n' || c == '\r') {
      if (_cmdLen > 0) {
        _cmdBuf[_cmdLen] = '\0';
        runXdataCommand(_cmdBuf);
        _cmdLen = 0;
      }
    } else if (_cmdLen < sizeof(_cmdBuf) - 1) {
      _cmdBuf[_cmdLen++] = c;
    } else {
      _cmdLen = 0;
    }
  }
}

void interfaceHandler() {
  if (xdataPortMode != 1 && xdataPortMode != 3) return;

  xdataCmdDrain();

  // Mode 3: the xdata UART is full-duplex, so NFW frames (GCS link) and the OIF411
  // instrument share it full-time at 9600 baud; the ozone parser runs separately.
  static uint16_t nfw_seq = 0;
  char* buf = g_txScratch.nfwFrame;   // shares g_txScratch (not live during Horus/APRS TX)
  uint16_t pos = 0;
  uint8_t  chk = 0;

  #define NW_S(s)   do { const char *_p=(s); while(*_p){buf[pos++]=*_p;chk^=(uint8_t)*_p++;} } while(0)
  #define NW_D()    do { buf[pos++]='|'; chk^='|'; } while(0)
  #define NW_I(v)   do { char _t[16]; snprintf(_t,sizeof(_t),"%ld",(long)(v)); NW_S(_t); } while(0)
  #define NW_F(v,d) do { char _t[16]; dtostrf((float)(v),1,(d),_t); NW_S(_t); } while(0)

  buf[pos++] = '$';
  NW_S("NFW");
  NW_D(); NW_I(nfw_seq++);
  // 1 - System
  NW_D(); NW_I(rsm4x2);
  NW_D(); NW_I(rsm4x4);
  NW_D(); NW_S(NFW_VERSION);
  NW_D(); NW_I(millis());
  NW_D(); NW_I(autoResetEnable);
  NW_D(); NW_I(buttonMode);
  NW_D(); NW_I(ledStatusEnable);
  NW_D(); NW_I(ledAutoDisableHeight);
  // 2 - GPS
  NW_D(); NW_I(ubloxGpsAirborneMode);
  NW_D(); NW_I(gpsTimeoutWatchdog);
  NW_D(); NW_I(improvedGpsPerformance);
  NW_D(); NW_I(disableGpsImprovementInFlight);
  NW_D(); NW_I(gpsOperationMode);
  NW_D(); NW_F(gpsLat,  6);
  NW_D(); NW_F(gpsLong, 6);
  NW_D(); NW_F(gpsAlt,  2);
  NW_D(); NW_I(gpsSats);
  NW_D(); NW_I(gpsHours);
  NW_D(); NW_I(gpsMinutes);
  NW_D(); NW_I(gpsSeconds);
  NW_D(); NW_F(gpsSpeedKph, 2);
  NW_D(); NW_F(gpsHdop,     2);
  NW_D(); NW_F(vVCalc,      2);
  NW_D(); NW_I(gpsStatus);
  NW_D(); NW_I(gpsJamWarning);
  // 3 - Radio General
  NW_D(); NW_I(radioEnablePA);
  NW_D(); NW_S(CALLSIGN);
  NW_D(); NW_F(readRadioTemp(), 2);
  // 4 - PIP
  NW_D(); NW_I(pipEnable);
  NW_D(); NW_F(pipFrequencyMhz, 3);
  NW_D(); NW_I(pipTimeSyncSeconds);
  NW_D(); NW_I(pipTimeSyncOffsetSeconds);
  NW_D(); NW_I(pipRadioPower);
  // 5 - Horus V2
  NW_D(); NW_I(horusEnable);
  NW_D(); NW_F(horusFreqTable[0], 3);
  NW_D(); NW_I(horusTimeSyncSeconds);
  NW_D(); NW_I(horusTimeSyncOffsetSeconds);
  NW_D(); NW_I(horusPayloadId);
  NW_D(); NW_I(horusRadioPower);
  // 5.1 - Horus V3
  NW_D(); NW_I(horusV3Enable);
  NW_D(); NW_F(horusFreqTable[0], 3);
  NW_D(); NW_I(horusTimeSyncSeconds);
  NW_D(); NW_I(horusTimeSyncOffsetSeconds);
  NW_D(); NW_S(HORUS_V3_CALLSIGN);
  NW_D(); NW_I(horusV3RadioPower);
  // 6 - APRS
  NW_D(); NW_I(aprsEnable);
  NW_D(); NW_F(aprsFreqTable[0], 3);
  NW_D(); NW_I(aprsTimeSyncSeconds);
  NW_D(); NW_I(aprsTimeSyncOffsetSeconds);
  NW_D(); NW_S(aprsCall);
  NW_D(); NW_S(aprsComment.c_str());
  NW_D(); NW_I(aprsSsid);
  NW_D(); NW_I(aprsOperationMode);
  NW_D(); NW_I(aprsRadioPower);
  NW_D(); NW_I(aprsToneCalibrationMode);
  // 7 - RTTY
  NW_D(); NW_I(rttyEnable);
  NW_D(); NW_F(rttyFrequencyMhz, 3);
  NW_D(); NW_I(rttyTimeSyncSeconds);
  NW_D(); NW_I(rttyTimeSyncOffsetSeconds);
  NW_D(); NW_I(rttyRadioPower);
  // 7.1 - Morse
  NW_D(); NW_I(morseEnable);
  NW_D(); NW_F(morseFrequencyMhz, 3);
  NW_D(); NW_I(morseTimeSyncSeconds);
  NW_D(); NW_I(morseTimeSyncOffsetSeconds);
  NW_D(); NW_I(morseRadioPower);
  // 8 - Power
  NW_D(); NW_F(readBatteryVoltage(), 2);
  NW_D(); NW_F(vBatWarnValue,        2);
  NW_D(); NW_F(batteryCutOffVoltage, 2);
  NW_D(); NW_I(ultraPowerSaveAfterLanding);
  // 9 - Flight stats
  NW_D(); NW_I(maxAlt);
  NW_D(); NW_I(maxSpeed);
  NW_D(); NW_I(maxAscentRate);
  NW_D(); NW_I(maxDescentRate);
  NW_D(); NW_I(maxMainTemperature);
  NW_D(); NW_I(minMainTemperature);
  NW_D(); NW_I(maxInternalTemp);
  NW_D(); NW_I(minInternalTemp);
  NW_D(); NW_I(beganFlying);
  NW_D(); NW_I(burstDetected);
  NW_D(); NW_I(hasLanded);
  NW_D(); NW_I(flightStartClimbThreshold);
  NW_D(); NW_I(burstDetectionThreshold);
  NW_D(); NW_I(lowAltitudeFastTxThreshold);
  // 10 - Sensor boom & heating
  NW_D(); NW_I(sensorBoomEnable);
  NW_D(); NW_F(readThermistorTemp(), 2);
  NW_D(); NW_F(mainTemperatureValue, 2);
  NW_D(); NW_F(extHeaterTemperatureValue, 2);
  NW_D(); NW_F(humidityValue, 1);
  NW_D(); NW_F(pressureValue, 2);
  NW_D(); NW_F(mainTemperatureCorrectionC, 2);
  NW_D(); NW_F(extHeaterTemperatureCorrectionC, 2);
  NW_D(); NW_I(autoTemperatureCalibration);
  NW_D(); NW_I(autoTemperatureCalibrationMethod);
  NW_D(); NW_F(environmentStartupAirTemperature, 2);
  NW_D(); NW_I(autoHumidityModuleTemperatureCorrection);
  NW_D(); NW_I(humidityModuleEnable);
  NW_D(); NW_I(zeroHumidityCalibration);
  NW_D(); NW_F(humidityCapacitanceRangeDelta, 2);
  NW_D(); NW_F(zeroHumidityCapacitance, 2);
  NW_D(); NW_I(calibrationError);
  NW_D(); NW_I(extHeaterPwmStatus);
  NW_D(); NW_I(referenceHeaterStatus);
  NW_D(); NW_I(referenceAreaTargetTemperature);
  NW_D(); NW_I(humidityModuleHeating);
  NW_D(); NW_I(sensorBoomMainTempError);
  NW_D(); NW_I(sensorBoomHumidityModuleError);
  // Measurement mode + MCU temperature + factory-mode self-check diagnostics
  NW_D(); NW_I(FACTORY_CAL_ACTIVE ? 2 : 1);   // measurementMode: 1 = NFW, 2 = Vaisala factory
  NW_D(); NW_F(readMcuTemperature(), 1);       // MCU die temperature (°C)
  NW_D(); NW_I(temperatureCheckError);
  NW_D(); NW_I(humidityCheckError);
  NW_D(); NW_S(boomSerialNumber.c_str());       // measurement-boom serial (factory mode); empty otherwise
  NW_D(); NW_F(humidityCapacitance, 2);        // kept: used by the NFW humidity-range calibration window
  // Pressure & RPM411
  NW_D(); NW_I(pressureMode);
  NW_D(); NW_F(rpm411InternalTemperature, 2);
  NW_D(); NW_S(RPM411SerialNumber);
  NW_D(); NW_I(rpm411Error);
  NW_D(); NW_F(seaLevelPressure, 2);
  // 12 - Recorder
  NW_D(); NW_I(dataRecorderEnable);
  NW_D(); NW_I(dataRecorderInterval);
  NW_D(); NW_I(dataRecorderFlightNoiseFiltering);
  NW_D(); NW_I(recorderInitialized);
  // 13 - Status + stage  (warn field kept at 0 for frame compatibility; status is only ok/err now)
  NW_D(); NW_I(err);
  NW_D(); NW_I(0);
  NW_D(); NW_I(ok);
  NW_D(); NW_S(nfwCurrentStage);
  // 14 - XDATA / OIF411 ozone
  NW_D(); NW_I(xdataPortMode);
  NW_D(); NW_F(xdataOzonePumpTemperature, 2);
  NW_D(); NW_F(xdataOzoneCurrent,         4);
  NW_D(); NW_F(xdataOzoneBatteryVoltage,  2);
  NW_D(); NW_F(xdataOzonePumpCurrent,     1);
  NW_D(); NW_F(xdataOzoneExtVoltage,      2);
  NW_D(); NW_I(xdataOzoneDiagnostics);
  NW_D(); NW_I(xdataOzoneFwVersion);
  NW_D(); NW_S(xdataOzoneSerial);
  NW_D(); NW_F(xdataOzonePartialPressure, 4);
  NW_D(); NW_F(xdataOzonePpb,             2);
  NW_D(); NW_F(ozone_P0,                  2);
  NW_D(); NW_I(ozoneConnectionError);    // 142 - OIF411 connection timeout flag
  NW_D(); NW_I(ozoneDbgLen);             // 143 - last raw frame length (0=none received yet)
  NW_D(); NW_S(ozoneDbgRaw);             // 144 - last raw xdata= frame content (hex ASCII)
  NW_D(); NW_I(ozoneRxByteTotal);        // 145 - total bytes received on xdata RX
  NW_D(); NW_I(ozoneFrameTotal);         // 146 - OIF411 frames successfully parsed
  // 15 - GPS diagnostics (v68 native UBX). Appended at the end so existing field
  // positions stay put. Keep this list in sync with NFW_FIELDS in the Ground
  // Control page (same order).
  NW_D(); NW_I(gps.fixType);                    // 0 none, 2 2D, 3 3D
  NW_D(); NW_F((float)gps.hAccMm / 1000.0, 1);  // horizontal accuracy (m)
  NW_D(); NW_F((float)gps.vAccMm / 1000.0, 1);  // vertical accuracy (m)
  NW_D(); NW_F((float)gps.sAccMmS / 1000.0, 2); // speed accuracy (m/s)
  NW_D(); NW_I(gps.jamIndicator);               // 0..255 CW jamming indicator
  NW_D(); NW_I(gps.spoofState);                 // 0 unknown,1 none,2 spoofing,3 multiple
  NW_D(); NW_I(gpsCfgNakCount);                 // GPS config messages rejected
  NW_D(); NW_I(gpsUpdateRateHz);
  NW_D(); NW_I(gpsDynamicModel);
  NW_D(); NW_I(gpsTrackingProfile);
  NW_D(); NW_I(gpsSecondaryGnss);   // 0 BeiDou B1C + GLONASS, 1 BeiDou B1I only, 2 GLONASS only
  NW_D(); NW_I(gpsQzssEnable);
  NW_D(); NW_I(gpsSbasEnable);
  // 16 - satellites used per constellation (NAV-SAT) + UBX frame errors. Both
  // BeiDou and GLONASS are sent; the unused one is 0. Keep in sync with NFW_FIELDS.
  NW_D(); NW_I(gps.satGps);
  NW_D(); NW_I(gps.satGal);
  NW_D(); NW_I(gps.satBds);
  NW_D(); NW_I(gps.satGlo);
  NW_D(); NW_I(gps.satSbas);
  NW_D(); NW_I(gps.frameErrors);
  NW_D(); NW_I(simultaneousGnssSetup);   // GPS brought up early (during setup/calibration)
  NW_D(); NW_I(gpsResetCounter);         // GPS module resets (timeout-watchdog recoveries)
  NW_D(); NW_I(gps.psmState);            // M10 power-save state: 0 off,3 tracking,4 power-optimized,5 inactive

  buf[pos++] = '*';
  buf[pos++] = "0123456789ABCDEF"[chk >> 4];
  buf[pos++] = "0123456789ABCDEF"[chk & 0xF];
  buf[pos++] = '\r';
  buf[pos++] = '\n';
  xdataSerial.write((const uint8_t*)buf, pos);
  xdataCmdDrain();  // process commands that arrived during blocking frame TX

  #undef NW_S
  #undef NW_D
  #undef NW_I
  #undef NW_F
}

// [RS41-NFW-SA] Source-Available Module - NOT under GPL-3.0. See LICENSING.md and LICENSE.source-available.
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
    setStage("25");
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: entering humidity delta calibration debug mode");
    }

    while (extHeaterTemperatureValue > 40) {
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Cooldown (<40C) T=");
        xdataSerial.print(extHeaterTemperatureValue);
        xdataSerial.println(" *C");
      }

      orangeLed();
      delay(50);
      bothLedOff();

      sensorBoomHandler();
    }

    humidityCapacitanceRangeDelta = 0;

    setStage("26");
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: place at 100%RH, watch humidityCapacitanceRangeDelta");
    }

    float _capDeltaMax = 0.0f;
    for (;;) {
      greenLed();
      delay(50);
      bothLedOff();

      sensorBoomHandler();

      float _liveDelta = (humidityCapacitance - zeroHumidityCapacitance) * 1.08f;  // delta vs zero-humidity capacitance, with calibration factor
      if (_liveDelta > _capDeltaMax) _capDeltaMax = _liveDelta;
      // humidityCapacitanceRangeDelta holds the PEAK delta - that is the value to record and
      // put in CONFIG.h.
      humidityCapacitanceRangeDelta = _capDeltaMax;

      if (xdataPortMode == 1) {
        xdataSerial.print("humidityCapacitance = ");
        xdataSerial.print(humidityCapacitance);
        xdataSerial.print(" uF,  live delta = ");
        xdataSerial.print(_liveDelta);
        xdataSerial.print(",  peak (record this) = ");
        xdataSerial.println(_capDeltaMax);
      }

      interfaceHandler();
      xdataCmdDrain();   // mode 1: GCS commands (CMD:STOP)
      ozoneHandler();    // mode 3: GCS commands ride the shared OIF411 RX stream

      if (_hrdStopRequested) {
        _hrdStopRequested = false;
        if (xdataPortMode == 1) {
          xdataSerial.println("[info]: Humidity range debug stopped by command.");
        }
        break;
      }

      if (analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN) && analogRead(VBAT_PIN) > 100) {
        setStage("27");
        if (xdataPortMode == 1) {
          xdataSerial.println("[info]: power off - set humidityCapacitanceRangeDelta, disable debug");
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
    if (xdataPortMode == 1 && !lastRpm411ErrorState) {
      xdataSerial.println("[err]: RPM411 connection error");
    }
    lastRpm411ErrorState = true;
  }
  else {
    rpm411Error = false;
    RPM411ParseConfigData();
    if (xdataPortMode == 1 && lastRpm411ErrorState) {
      xdataSerial.println("[info]: RPM411 OK");
    }
    lastRpm411ErrorState = false;
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

    if (xdataPortMode == 1 && !lastRpm411ErrorState) {
      xdataSerial.println("[err]: RPM411 connection error");
    }
    lastRpm411ErrorState = true;

  }
  else {
    rpm411Error = false;
    RPM411ParseReadings();

    if (xdataPortMode == 1 && lastRpm411ErrorState) {
      xdataSerial.println("[info]: RPM411 OK");
    }
    lastRpm411ErrorState = false;

  }
}

void RPM411ParseConfigData() {
  // Keep only printable ASCII (0x20-0x7E). The serial goes into the $NFW telemetry
  // frame, which Ground Control decodes as UTF-8 - any non-ASCII byte (e.g. 0xFF when
  // the RPM411 is absent or the read failed) would show up as the replacement glyph
  // and, worse, break the frame checksum so the whole frame is dropped. Stop at the
  // first non-printable byte, which also trims null/0xFF padding.
  char tmp[9];
  int n = 0;
  for (int i = 0; i < 8; i++) {
    char c = (char)RPM411ConfigData[1][(41 + i) % 33];
    if (c < 0x20 || c > 0x7E) break;
    tmp[n++] = c;
  }
  tmp[n] = '\0';

  // Only overwrite the stored serial if we actually decoded one. A disconnected or
  // garbled read yields an empty string - in that case keep the last known serial so it
  // does not vanish to "--" in Ground Control on a momentary RPM411 disconnect.
  if (n > 0) {
    memcpy(RPM411SerialNumber, tmp, n + 1);
  }
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

    // Barometric pressure from GPS altitude using the ISA layer model. Available on both
    // boards: kept in single precision (float powf()/expf()), and on the RSM4x2 (F100) the
    // factory calibration already links expf()/logf(), so the marginal flash cost is small.
    // float powf()/expf() reproduce the same hPa to far better than 0.1 hPa. The three
    // inter-layer factors are compile-time constants (verified against the standard
    // atmosphere: 226.3 / 54.7 / 8.7 hPa base pressures for P0 = 1013.25).
    const float gMR = 0.0341632f;   // g0*M/R (1/m)

    const float P0  = seaLevelPressure;      // measured MSL pressure (hPa)
    const float P11 = P0  * 0.22336105f;     // base of the 11-20 km layer
    const float P20 = P11 * 0.24190845f;     // base of the 20-32 km layer
    const float P32 = P20 * 0.15854540f;     // base of the 32+ km layer

    float Pb, Tb, Lb, hb;
    if (gpsAlt < 11000.0f)      { Pb = P0;  Tb = 288.15f; Lb = -0.0065f; hb = 0.0f;     }
    else if (gpsAlt < 20000.0f) { Pb = P11; Tb = 216.65f; Lb = 0.0f;     hb = 11000.0f; }
    else if (gpsAlt < 32000.0f) { Pb = P20; Tb = 216.65f; Lb = 0.0010f;  hb = 20000.0f; }
    else                        { Pb = P32; Tb = 228.65f; Lb = 0.0028f;  hb = 32000.0f; }

    if (Lb == 0.0f) {
      pressureValue = Pb * expf(-gMR * (gpsAlt - hb) / Tb);
    } else {
      float term = 1.0f + Lb * (gpsAlt - hb) / Tb;
      pressureValue = (term > 0.0f) ? Pb * powf(term, -gMR / Lb) : 0.0f;  // clamp below the model
    }
  }
  else {
    pressureValue = 0;
  }

  if (xdataPortMode == 1 && pressureMode != 0) {
    static unsigned long _lastPresLog = 0;
    if (millis() - _lastPresLog >= 15000UL) {
      _lastPresLog = millis();
      xdataSerial.print(F("[pres]: "));
      xdataSerial.print(pressureValue, 1);
      xdataSerial.println(F(" hPa"));
    }
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

// Safety net: if the MCU ever takes a hard fault it would otherwise spin forever in the
// default handler (a dead sonde). Instead, reset so it recovers on its own.
extern "C" void HardFault_Handler(void) {
  NVIC_SystemReset();
  while (1) {}
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
  }

  setStage("01");
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: stage 01 - HW init");
  }

  if (rsm4x4) {
    gpsSerial.begin(gpsBaudRate);
  } else if (rsm4x2) {
    gpsSerial.begin(gpsBaudRate);
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Serial init ok");
  }

  if (rsm4x4) {
    analogReadResolution(12);
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: ADC 12bit ok");
    }
  }

  analogWriteResolution(8);    // Set PWM resolution
  analogWriteFrequency(1000);  // Set PWM frequency
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: PWM 8bit 1kHz ok");
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: MCO init...");
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); // MCO1 with divider 1 from HSI clock source, output on PA8

  SPI_2.begin();
  digitalWrite(CS_RADIO_SPI, HIGH);  // Deselect the SI4432 CS pin
  digitalWrite(CS_SPI, HIGH);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: SPI2 ok");
  }

  setStage("02");
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: stage 02 - GPS/radio init");
  }

  // The GPS is powered up a little further down, AFTER the reference/humidity heaters are
  // switched off - those draw ~180 mA and their switching noise degrades a GPS cold-start
  // fix, so the receiver must acquire with them off. Hold it on reset for now.
  // simultaneousGnssSetup decides whether that power-up happens here (so it acquires during
  // setup and calibration) or only after calibration.
  bool gpsStartEarly = (gpsOperationMode != 0) && simultaneousGnssSetup;
  shutdownGPS();

  digitalWrite(CS_RADIO_SPI, LOW);
  initSi4032();
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 regs ok");
  }

  setRadioPower(6);
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 PA=6 (50mW)");
  }

  writeRegister(0x72, 0x05);

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 dev=0x07");
  }

  fsk4_bitDuration = (uint32_t)1000000 / horusBdr;  //horus 100baud delay calculation

  // Heaters OFF before the GPS powers up (see the note above). Done here, after the Si4032
  // is initialised, because on RSM4x2 the reference heater is switched through the Si4032.
  selectReferencesHeater(0);   //turn off reference heating
  extHeaterHandler(false, 0, 0);

  // Now bring the GPS up (heaters are off). simultaneousGnssSetup: on -> acquire during the
  // rest of setup and calibration; off -> it stays on reset and is started after calibration.
  if (gpsStartEarly) {
    startGPS();
    delay(1000);
    initGPS();
  }

  setStage("03");
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: stage 03 - boom/RPM411 init");
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Boom/RPM411 init...");
  }

  selectSensorBoom(0, 0);  //turn off all sensor boom measurement circuits

  if(pressureMode == 1) {
    initRPM411();
    readRPM411();
    pressureKalmanEst = pressureValue;

  }
  pressureHandler();

  orangeLed();

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: HW init done");
  }

  if (foxHuntMode) {
    shutdownGPS();
    foxHuntModeLoop();
  }

  // Factory (Vaisala) mode uses absolute calibration polynomials, so the NFW
  // startup temperature-offset calibration is skipped - otherwise it would add an
  // offset on top of the factory reading and shift it to environmentStartupAirTemperature.
  if (sensorBoomEnable && FACTORY_CAL_ACTIVE) {
    // ===== Factory (Vaisala) mode (both board families) =====
    // The NFW startup calibrations (temperature offset, zero-humidity, capacitance
    // range) are intentionally NOT run - the factory coefficients are absolute.
    // Optional factory-mode self-checks.
    if (factoryTemperatureCheck) {
      temperatureCheck();
    }
    if (factoryHumidityCheck && humidityModuleEnable) {
      humidityCheck();
    }
  }
#if defined(RSM4x4)
  // ===== NFW mode: original startup calibration chain (RSM4x4 / RSM4x5 only) =====
  else if (sensorBoomEnable) {
    temperatureCalibration();

    if (reconditioningEnabled) {
      reconditioningPhase();
    }

    if (humidityModuleEnable && zeroHumidityCalibration) {
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Humidity cal start");
      }
      zeroHumidityCheck();
      if (xdataPortMode == 1) {
        xdataSerial.println("[info]: Humidity cal done");
      }
    }
  }
#endif  // RSM4x4 NFW startup chain

  maxHumidityFrequency = zeroHumidityFrequency - humidityRangeDelta;
  maxHumidityCapacitance = zeroHumidityCapacitance + humidityCapacitanceRangeDelta;

  if (xdataPortMode == 1) {
    xdataSerial.print("0RH_cap | 100RH_cap => ");
    xdataSerial.print(zeroHumidityCapacitance);
    xdataSerial.print(" | ");
    xdataSerial.println(maxHumidityCapacitance);
  }

  humidityDeltaCalibrationDebug();

  // Bring the GPS up now if it was deliberately held on reset through calibration
  // (i.e. simultaneousGnssSetup was off, so it was not already started early above).
  if (!gpsStartEarly && gpsOperationMode != 0) {
    startGPS();
    delay(1000);
    initGPS();
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Setup done, entering main loop");
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

  setStage("04");
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: stage 04 - main loop start");
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
