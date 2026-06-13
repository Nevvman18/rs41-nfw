/*
RS41-NFW - versatile, feature-rich and user-friendly custom firmware for ALL revisions of Vaisala RS41 radiosondes
Released on GPL-3.0 license.
Authors: Franek Łada (nevvman, SP5FRA)

Version 66 (public, stable)

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
#define NFW_VERSION "RS41-NFW v66, GPL-3.0 Franek Lada (nevvman, SP5FRA)"  //This is the firmware version You are running

//===== Libraries and lib-dependant definitions (nothing to modify)
/* No libraries are required to be installed, all dependencies are shipped within the project folder. */
#include "horus_l2.h"
//#include "horus_l2.cpp"
#include <SPI.h>
#include "gps.h"
#include "HorusBinaryV3.h"

TinyGPSPlus gps;

#include <HardwareTimer.h>

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
bool gpsTimeoutCounterActive = false;
unsigned long gpsTimerBegin = 0;
int currentGPSPowerMode = 0;  // RSM4x2/4x1 GPS power state (sent as Horus "gpspwr"): 0 not set, 1 max-performance/continuous, 2 power-save
unsigned long lastPowerSaveChange = 0;
unsigned int rttyFrameCounter = 0;
unsigned long lowAltitudeFastTxModeBeginTime = 0;
unsigned int gpsResetCounter = 0;
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
float   flightBaseAlt = 0.0f;        // launch-baseline altitude, captured at first valid fix
bool    flightBaselineSet = false;
float   flightPrevAlt = 0.0f;        // last fix altitude, to count only new fixes
uint8_t flightSustainedRise = 0;     // consecutive fixes >= climb threshold (noise rejection)
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
bool _hrdStopRequested = false;
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

// ===== SCHEDULER STATE =====
// Managed by schedulerInit() / schedulerLoop(). Do not access directly from other code.

unsigned long sch_sysMs    = 0;   // UTC time-of-day in ms (GPS-synced when available)
unsigned long sch_lastMillis = 0; // Previous millis() snapshot used for time tracking
bool          sch_gpsSynced  = false; // True when GPS time is valid and locked

// Next scheduled TX times in sch_sysMs units (ms since UTC midnight).
// Value 0 = uninitialized → compute first slot on the next scheduler pass.
unsigned long sch_nextPipMs     = 0;
unsigned long sch_nextHorusV3Ms = 0;
unsigned long sch_nextHorusMs   = 0;
unsigned long sch_nextAprsMs    = 0;
unsigned long sch_nextRttyMs    = 0;
unsigned long sch_nextMorseMs   = 0;

// Sensor-update freshness - millis()-based (immune to GPS clock corrections)
unsigned long sch_lastSensorBoom = 0;
unsigned long sch_lastPressure   = 0;
unsigned long sch_lastInterface  = 0;
unsigned long sch_lastGps        = 0;
unsigned long sch_lastOzone      = 0;

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

  // Clamp all fields to their ASN.1-schema limits before encoding.
  // Exceeding any limit causes the encoder to return an error and drop the packet.
  const uint8_t  asn_sats      = constrain(gpsSats,                                  0,      31);
  const int32_t  asn_alt       = constrain((int32_t)gpsAlt,                      -1000,   50000);
  const uint16_t asn_speed     = constrain((int32_t)gpsSpeedKph,                     0,     512);
  const uint16_t asn_pressure  = constrain((int32_t)(pressureValue*10),              0,   12000);
  const int16_t  asn_tempExt   = constrain((int32_t)(mainTemperatureValue*10),    -1023,    1023);
  const int16_t  asn_tempCust1 = constrain((int32_t)(extHeaterTemperatureValue*10), -1023,  1023);
  const uint8_t  asn_humidity  = constrain((int32_t)humidityValue,                   0,     100);

  horusTelemetry asnMessage = {
        .payloadCallsign  = HORUS_V3_CALLSIGN,
        .sequenceNumber = horusV3PacketCount,
        .timeOfDaySeconds  = gpsHours*3600 + gpsMinutes*60 + gpsSeconds,
        .latitude = (int)(gpsLat*100000),
        .longitude = (int)(gpsLong*100000),
        .altitudeMeters = asn_alt,
        .extraSensors = {
          .nCount = (xdataPortMode == 3 && horusV3ExtraSensorsEnable) ? 3 : 1,
          .arr = {
            { .name = "gpspwr", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = {gpsStatus} } } }, .exist = { .name = true, .values = true } },
            { .name = "p3",     .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzonePartialPressure } } } }, .exist = { .name = xdataPortMode == 3, .values = xdataPortMode == 3 } },
            { .name = "o3ppb",  .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)xdataOzonePpb } } } },        .exist = { .name = xdataPortMode == 3, .values = xdataPortMode == 3 } },
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

  horusTelemetry asnMessage = {
      .payloadCallsign = HORUS_V3_CALLSIGN,
      .sequenceNumber = horusV3PacketCount,
      .timeOfDaySeconds = gpsHours * 3600 + gpsMinutes * 60 + gpsSeconds,
      .latitude = (int)(gpsLat * 100000),
      .longitude = (int)(gpsLong * 100000),
      .altitudeMeters = dr_alt,

      // 4 individually named sensors per packet (ASN.1 hard limit: max 4 per packet)
      // page 0 = A: GPS diagnostics  page 1 = B: flight stats  page 2 = C: thermal/heater
      .extraSensors = [&]() -> horusAdditionalSensors {
        if (page == 0) {
          return { .nCount = 4, .arr = {
            { .name = "hdop",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsHdop          } } } }, .exist = { .name = true, .values = true } },
            { .name = "jam",    .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsJamWarning    } } } }, .exist = { .name = true, .values = true } },
            { .name = "resets", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsResetCounter  } } } }, .exist = { .name = true, .values = true } },
            { .name = "gpspwr", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)gpsStatus        } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 1) {
          return { .nCount = 4, .arr = {
            { .name = "flying", .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)beganFlying      } } } }, .exist = { .name = true, .values = true } },
            { .name = "burst",  .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)burstDetected    } } } }, .exist = { .name = true, .values = true } },
            { .name = "hmax",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)maxAlt           } } } }, .exist = { .name = true, .values = true } },
            { .name = "vmax",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)maxSpeed         } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 3) {
          // Physical measurements go out as floats (horusReal) so a decoder shows the
          // real value (17.1 C), not a scaled integer (171). Only flags/versions stay int.
          return { .nCount = 4, .arr = {
            { .name = "pumpt",  .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzonePumpTemperature } } } }, .exist = { .name = true, .values = true } },
            { .name = "o3c",    .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzoneCurrent         } } } }, .exist = { .name = true, .values = true } },
            { .name = "pumpu",  .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzoneBatteryVoltage  } } } }, .exist = { .name = true, .values = true } },
            { .name = "pumpc",  .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzonePumpCurrent     } } } }, .exist = { .name = true, .values = true } },
          }};
        } else if (page == 4) {
          return { .nCount = 4, .arr = {
            { .name = "oifu",    .values = { .kind = horusReal_PRESENT, .u = { .horusReal = { .nCount = 1, .arr = { xdataOzoneExtVoltage                 } } } }, .exist = { .name = true, .values = true } },
            { .name = "oiferr",  .values = { .kind = horusInt_PRESENT,  .u = { .horusInt  = { .nCount = 1, .arr = { (xdataOzoneDiagnostics != 0) ? 1 : 0 } } } }, .exist = { .name = true, .values = true } },
            { .name = "oifver",  .values = { .kind = horusInt_PRESENT,  .u = { .horusInt  = { .nCount = 1, .arr = { (int)xdataOzoneFwVersion             } } } }, .exist = { .name = true, .values = true } },
            { .name = "o3ppb",   .values = { .kind = horusInt_PRESENT, .u = { .horusInt = { .nCount = 1, .arr = { (int)xdataOzonePpb                   } } } }, .exist = { .name = true, .values = true } },
          }};
        } else {
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


// ─── OIF411 Ozone Sonde Handler ───────────────────────────────────────────────

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
    // Measurement frame: [type2][num2][pumpT4][I5][batV2][pumpI3][extV2]
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
        
        if(xdataPortMode == 2) {   // GPS bridge mode: echo raw NMEA to the xdata port
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
      static unsigned long _lastGpsLog = 0;
      if (millis() - _lastGpsLog >= 30000UL) {
        _lastGpsLog = millis();
        xdataSerial.print(F("[gps]: "));
        xdataSerial.print(gpsLat, 6); xdataSerial.print(F(","));
        xdataSerial.print(gpsLong, 6);
        xdataSerial.print(F(" alt=")); xdataSerial.print((int)gpsAlt);
        xdataSerial.print(F("m sats=")); xdataSerial.print(gpsSats);
        xdataSerial.print(F(" hdop=")); xdataSerial.print(gpsHdop, 1);
        xdataSerial.print(F(" vv=")); xdataSerial.print(vVCalc, 1);
        xdataSerial.println(F("m/s"));
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

    if (beganFlying && (gpsHdop > 15 || abs(vVCalc) > 300)) {
      gpsJamWarning = true;

      if (xdataPortMode == 1) {
        xdataSerial.println("[warn]: GPS Jam warning is active!");
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
        xdataSerial.print("[err]: Heater OFF - overheat threshold (*C): ");
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
        xdataSerial.print("[info]: Temp below re-enable threshold (*C): ");
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
        xdataSerial.print("[info]: Heater OFF - cooldown (s): ");
        xdataSerial.println(heaterCooldownTimeSec);
      }

      heaterOffTime = currentMillis;  // Record the time the heater was turned off
      isReferenceHeaterOn = false;             // Update the heater state
      heaterDebugState = 10;
    } else if (isReferenceHeaterOn) {
      if (xdataPortMode == 1) {
        xdataSerial.print("[info]: Heater ON (*C): ");
        xdataSerial.println(currentTemp);
      }

      heaterDebugState = 11;
    } else {
      // If the heater is off, check if cooldown has passed and if it should be re-enabled
      if (currentMillis - heaterOffTime >= heaterCooldownTimeSec * 1000) {
        if (currentTemp < autoHeaterThreshold) {
          if (xdataPortMode == 1) {
            xdataSerial.print("[info]: Cooldown done, T under threshold (*C): ");
            xdataSerial.println(autoHeaterThreshold);
            xdataSerial.println("[info]: HEATER ON!");
          }

          enableRefHeaterRing();  // Turn on the heater

          heaterOnTime = currentMillis;  // Record the time the heater was turned on
          isReferenceHeaterOn = true;             // Update the heater state
          heaterDebugState = 11;
        } else {
          if (xdataPortMode == 1) {
            xdataSerial.println("[info]: autoRefHeat: T above threshold, skip");
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
        xdataSerial.print("[err]: Heater OFF - overheat threshold (*C): ");
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
        xdataSerial.print("[info]: Temp below re-enable threshold (*C): ");
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
      xdataSerial.println("[err]: BATTERY CUT-OFF VOLTAGE, SYSTEM WILL POWER OFF");
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

    // Check overall sensor status
    sensorBoomFault = sensorBoomMainTempError || sensorBoomHumidityModuleError;

    if (sensorBoomMainTempError && sensorBoomHumidityModuleError) {
      if (xdataPortMode == 1) {
        xdataSerial.println("[err]: The sensor boom seems disconnected!");
      }
    }

    if (xdataPortMode == 1) {
      static unsigned long _lastBoomLog = 0;
      if (millis() - _lastBoomLog >= 15000UL) {
        _lastBoomLog = millis();
        xdataSerial.print(F("[boom]: T="));
        xdataSerial.print(mainTemperatureValue, 1);
        xdataSerial.print(F("C H="));
        xdataSerial.print(humidityValue, 0);
        xdataSerial.print(F("% xT="));
        xdataSerial.print(extHeaterTemperatureValue, 1);
        xdataSerial.print(F("C pcb="));
        xdataSerial.print(readThermistorTemp(), 1);
        xdataSerial.println(F("C"));
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
    if (!flightBaselineSet) {
      flightBaseAlt     = gpsAlt;
      flightPrevAlt     = gpsAlt;
      flightBaselineSet = true;
    }

    if (!beganFlying && gpsAlt != flightPrevAlt) {   // only act on a fresh fix value
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
    ozoneHandler();
    sensorBoomHandler();
    pressureHandler();

    #ifdef RSM4x4
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
    #endif

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

      for (int i = 0; i < freqTableSize; i++) {
        float currentFreq = horusFreqTable[i];

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

  // Send 3 standard pages (A=GPS, B=stats, C=thermal) + 2 OIF411 pages (D=pump, E=oif) when in ozone mode
  const uint8_t totalDrPages = (xdataPortMode == 3) ? 5 : 3;
  for (uint8_t page = 0; page < totalDrPages; page++) {
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
      const char* labels[] = {"A(gps)", "B(stats)", "C(heat)", "D(ozone)", "E(oif)"};
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
    // the later pages (C/D/E in ozone mode) reuse the timestamp captured before page A,
    // so a decoder sees several frames with an identical time and drops them as repeats.
    if (page < totalDrPages - 1) {
      gpsHandler();
      sensorBoomHandler();
      if (!rpm411Error) readRPM411();
    }
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: dataRecorder TX done");
  }
}
#endif  // dataRecorder (dataRecorderTx) - both boards

#ifdef RSM4x4
void ultraPowerSaveHandler() {
  if (ultraPowerSaveAfterLanding) {
    if (hasLanded && millis() - landingTimeMillis > 1200000) {  //20 minutes after landing
      gpsOperationMode = 0;
      sensorBoomEnable = false;
      ledStatusEnable = false;
      selectSensorBoom(0, 0);
      setRadioPower(6);  //NOTE: power save mode changes the power to 50mW, which may not be what a powersave is meant to be. However, sonde laying on the ground has a very poor radio propagation and range, therefore a couple second long transmission won't impact it much
      for (;;) {
        #ifdef RSM4x4
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
        #endif

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
      float selfHeatingCorrectedInternalTemperature = -8.5 + 1.307 * internalTemperature - 0.001461 * pow(internalTemperature, 2) - 0.000082 * pow(internalTemperature, 3);
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
    xdataSerial.println("[warn]: Humidity module heating soon - don't touch sensor");
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
      xdataSerial.print("[warn]: Current humidity module temperature = ");
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
    xdataSerial.println("[info]: Zero-humidity cal start...");
    xdataSerial.println("[warn]: Sensor heats! Keep device still: no wind, RH<70%, T>0C, don't touch sensor");
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

    setStage("31");

    unsigned long startQuietMillis = millis();
    unsigned long lastUpdate = startQuietMillis;

    // Stay quiet until the silence window elapses, a fix is acquired (gpsSats >= 4),
    // or it is cancelled - so TX resumes immediately on fix instead of wasting the window.
    while ((millis() - startQuietMillis < radioSilenceDuration) && gpsSats < 4 && !cancelGpsImprovement) {

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

static void sch_syncGps() {
  if (gpsSats < 4 || !gps.time.isValid() || gps.time.age() > 1100) {
    if (sch_gpsSynced && xdataPortMode == 1)
      xdataSerial.println(F("[sch]: GPS sync lost"));
    sch_gpsSynced = false;
    return;
  }

  unsigned long gpsTodMs =
      (unsigned long)gps.time.hour()   * 3600000UL +
      (unsigned long)gps.time.minute() * 60000UL   +
      (unsigned long)gps.time.second() * 1000UL;

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
  } else if (labs(diffMs) >= 100) {
    sch_sysMs = (unsigned long)((long)sch_sysMs + diffMs);
    sch_lastMillis = millis();
  }

  if (!sch_gpsSynced) {
    if (xdataPortMode == 1) {
      xdataSerial.print(F("[sch]: GPS synced UTC "));
      if (gps.time.hour()   < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.hour());   xdataSerial.print(':');
      if (gps.time.minute() < 10) xdataSerial.print('0'); xdataSerial.print(gps.time.minute()); xdataSerial.print(':');
      if (gps.time.second() < 10) xdataSerial.print('0'); xdataSerial.println(gps.time.second());
    }
    sch_gpsSynced = true;
  }
}

void schedulerInit() {
  sch_lastMillis = millis();
  sch_sysMs      = 0;
  sch_gpsSynced  = false;
  sch_nextPipMs = sch_nextHorusV3Ms = sch_nextHorusMs =
  sch_nextAprsMs = sch_nextRttyMs = sch_nextMorseMs = 0;
  sch_lastSensorBoom = sch_lastPressure = sch_lastInterface = 0;
  sch_lastGps = sch_lastOzone = 0;
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
      xdataSerial.print(humidityValue, 0);
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

  if (improvedGpsPerformance && !cancelGpsImprovement && gpsSats < 4) {
    gpsQuietMode();
    sch_tickTime();
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

    if (!txImminent) {
      dataRecorderTx();   // dataRecorder on both boards (RSM4x4 + RSM4x2); self-gated by its own interval
    }

    if (!txImminent || gpsAge > 2000UL) {
      gpsHandler();
      sch_lastGps = millis();
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
          if ((millis() - sch_lastSensorBoom) > 10000UL) {
            sensorBoomHandler(); sch_lastSensorBoom = millis();
            pressureHandler();   sch_lastPressure   = millis();
          }
          buttonHandler();
          delayMicroseconds(200);
        }
      }
    }
  }

  {
    bool anyDone;
    do {
      anyDone = false;
      sch_tickTime();
      nowMs = sch_sysMs;

      unsigned long pickMs  = 0xFFFFFFFFUL;
      int           pickIdx = -1;  // 0=pip 1=horusV3 2=horus 3=aprs 4=rtty 5=morse

      auto checkMode = [&](bool en, unsigned long &nxt, uint16_t per, uint16_t off, int idx) {
        if (!en || nxt == 0 || nxt == 0xFFFFFFFFUL) return;
        if (nowMs < nxt) return;
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
        if ((millis() - sch_lastSensorBoom) > 2000UL) {
          sensorBoomHandler(); sch_lastSensorBoom = millis();
        }
        if ((millis() - sch_lastPressure) > 2000UL) {
          pressureHandler(); sch_lastPressure = millis();
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
  static char buf[1024];
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
  NW_D(); NW_F(tempSensorBoomCalibrationFactor, 2);
  NW_D(); NW_I(calibrationError);
  NW_D(); NW_I(extHeaterPwmStatus);
  NW_D(); NW_I(referenceHeaterStatus);
  NW_D(); NW_I(referenceAreaTargetTemperature);
  NW_D(); NW_I(humidityModuleHeating);
  NW_D(); NW_I(sensorBoomMainTempError);
  NW_D(); NW_I(sensorBoomHumidityModuleError);
  // Pressure & RPM411
  NW_D(); NW_I(pressureMode);
  NW_D(); NW_F(rpm411InternalTemperature, 2);
  NW_D(); NW_S(RPM411SerialNumber);
  NW_D(); NW_I(rpm411Error);
  NW_D(); NW_F(seaLevelPressure, 2);
  // 11 - Raw sensor data
  NW_D(); NW_F(mainTemperatureFrequency,       2);
  NW_D(); NW_F(mainTemperatureResistance,      2);
  NW_D(); NW_F(extHeaterTemperatureFrequency,  2);
  NW_D(); NW_F(extHeaterTemperatureResistance, 2);
  NW_D(); NW_F(extHeaterTemperatureValue,      2);
  NW_D(); NW_F(humidityCapacitance,            2);
  NW_D(); NW_F(maxHumidityCapacitance,         2);
  NW_D(); NW_I(THERMISTOR_R25);
  NW_D(); NW_I(THERMISTOR_B);
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
      xdataSerial.println("[info]: Ready - place sensor at 100%RH, observe humidityCapacitanceRangeDelta");
    }

    float _capDeltaMax = 0.0f;
    for (;;) {
      greenLed();
      delay(50);
      bothLedOff();

      sensorBoomHandler();

      float _liveDelta = (humidityCapacitance - zeroHumidityCapacitance) * 0.95f;  // delta vs zero-humidity capacitance, with calibration factor
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
          xdataSerial.println("[info]: Power off - reprogram humidityCapacitanceRangeDelta, disable humidityCalibrationDebug");
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
  int n = 0;
  for (int i = 0; i < 8; i++) {
    char c = (char)RPM411ConfigData[1][(41 + i) % 33];
    if (c < 0x20 || c > 0x7E) break;
    RPM411SerialNumber[n++] = c;
  }

  RPM411SerialNumber[n] = '\0';
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
    // Layer 0-11 km (lapse)
    double P11 = P0 * pow(1.0 + (-0.0065 * 11000.0) / 288.15, -gMR / -0.0065);

    // Layer 11-20 km (isothermal)
    double P20 = P11 * exp(-gMR * (20000.0 - 11000.0) / 216.65);

    // Layer 20-32 km (lapse)
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
  } else if (xdataPortMode == 2) {   // GPS bridge mode
    xdataSerial.begin(115200);
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

  setStage("03");
  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: stage 03 - boom/heater init");
  }

  if (xdataPortMode == 1) {
    xdataSerial.println("[info]: Boom/heater/RPM411 init...");
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
    xdataSerial.println("[info]: HW init done");
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
      xdataSerial.println("[info]: Humidity cal start");
    }
    zeroHumidityCheck();
    if (xdataPortMode == 1) {
      xdataSerial.println("[info]: Humidity cal done");
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
