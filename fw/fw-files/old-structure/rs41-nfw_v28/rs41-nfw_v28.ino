//===== Libraries and lib-dependant definitions
#include "horus_l2.h"
//#include "horus_l2.cpp"
#include <SPI.h>
#include <TinyGPSPlus.h>
TinyGPSPlus gps;


//===== Device revision definitions
#define RSM4x4 //new pcb versions
//#define RSM4x2 //old pcb versions, also rsm4x1

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
  #define HEAT_REF PC6   //reference heating resistors - LOW=disabled, HIGH=enabled - draw about 180mA of current and after a while may burn skin
  bool heaterPinControlAvail = true;
  #define REF_THERM PB1  //reference heating thermistor
  #define PULLUP_TM PB12 //ring oscillator activation mosfet for temperature reading
  #define PULLUP_HYG PA2 //ring oscillator activation mosfet for humidity reading
  #define SPST1 PB3 //idk spst1
  #define SPST2 PA3 //idk spst2
  #define SPST3 PC14 //boom hygro heater temperature
  #define SPST4 PC15 //boom main temperature
  #define MEAS_OUT PA1 //ring oscillator measurement output
  #define HEAT_HUM1 PA7
  #define HEAT_HUM2 PB8

  #define SI4032_CLOCK 26.0

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
  #define VBAT_PIN PA5          //battery voltage 
  #define VBTN_PIN PA6          //button state voltage divider, CHECK VALUES AT LOWER SUPPLY VOLTAGES

  #define MOSI_RADIO_SPI PB15
  #define MISO_RADIO_SPI PB14
  #define SCK_RADIO_SPI PB13
  #define CS_RADIO_SPI PC13
  #define HEAT_REF 0   //not available on older versions, switched only by si4032 gpio
  bool heaterPinControlAvail = false;
  #define REF_THERM PB1  //reference heating thermistor
  #define PULLUP_TM PB12 //ring oscillator activation mosfet for temperature reading
  #define PULLUP_HYG PA2 //ring oscillator activation mosfet for humidity reading
  #define SPST1 PB6 //calibration resistors - 90kHz
  #define SPST2 PA3 //cal. res. - 62kHz
  #define SPST3 PC14 //boom hygro heater temperature
  #define SPST4 PC15 //boom main temperature
  #define MEAS_OUT PA1 //ring oscillator measurement output
  #define HEAT_HUM1 PA7
  #define HEAT_HUM2 PB9

  #define SI4032_CLOCK 26.0

  //===== Interfaces
  //SPI_2 interface class (radio communication etc.)
  SPIClass SPI_2(PB15, PB14, PB13);  // MOSI, MISO, SCK for SPI2
  // ublox gps              rx    tx
  HardwareSerial gpsSerial(PA10, PA9);
  int gpsBaudRate = 9600;

#else
  #error "Please define the pcb model!"
#endif

// XDATA (2,3)          rx    tx
HardwareSerial xdataSerial(PB11, PB10);


unsigned char tx_buf[64] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };

//===== Radio signals config
#define CALLSIGN "N0CALLN0CALL"  //max 10 chars long, currently used for rtty
//#define CALLSIGN_SHORT "N0CALL1" //max 6 chars long, currently unused
#define PREAMBLE "AA"            //max 2 long
int defaultRadioPwrSetting = 7; //default TX power, also see lines down below; 0 = -1dBm (~0.8mW), 1 = 2dBm (~1.6mW), 2 = 5dBm (~3 mW), 3 = 8dBm (~6 mW), 4 = 11dBm (~12 mW), 5 = 14dBm (25 mW), 6 = 17dBm (50 mW), 7 = 20dBm (100 mW)
int powerSaveRadioPwrSetting = -1; //radio TX power for power save feature - deterimnes the TX power level at which the sonde will be transmitting when certain altitude (powerSaveAltitude), set to -1 to disable the powerSave features applying to the TX power. If this option is activated, the button logic for changing the radio power won't work

bool enableAddData = false;      //rtty-only mode, when false, the additional-data space is filled with zeros

bool pipEnable = false; //pip tx mode
float pipFrequencyMhz = 434.5; //pip tx frequency
int pipLengthMs = 1000; //pip signal length in ms
int pipRepeat = 3; //pip signal repeat count in 1 transmit group

bool morseEnable = false; //morse tx mode
float morseFrequencyMhz = 434.65; //morse tx frequency
int morseUnitTime = 40;  //ms

bool rttyEnable = false; //rtty tx mode
bool rttyShortenMsg = false; //false- all data, true - without preamble, time, speed and adddata
float rttyFrequencyMhz = 434.78; //rtty tx frequency
int rttyBitDelay = 22000;  //22000 ~= 45bdrate, 13333 ~= 75bdr
#define RTTY_RADIO_MARK_OFFSET 0x02 //for space offset set to 0x01, the rtty tone spacing will be: 0x02 - 270Hz spacing, 0x03 - 540Hz spacing | SPACING OTHER THAN 270HZ DOES NOT WORK (at lesast on my tests, will check later)
#define RTTY_RADIO_SPACE_OFFSET 0x01 //usually set to 0x01

bool horusEnable = true; //horus v2 tx mode
float horusFrequencyMhz = 434.74;
unsigned int horusPayloadId = 256;
int horusBdr = 100;

bool bufTxEnable = false; //alpha state! mode
float fskFrequencyMhz = 436; //txBuf tx frequency

bool radioEnablePA = false;  //default tx state
bool radioSleep = true; //lowers power consumption and recalibrates oscillator (drift compensation)
int defaultModeChangeDelay = 0; //in milliseconds, 0 - disable, 0<delay<2000 - standard delay, 2000<delay - delay + radio sleep mode and recalibration (if radioSleep enabled); default delay between radio transmission modes
int powerSaveModeChangeDelay = -1; //as above, but activates when the powerSave is ON, set to -1 to disable changing of the transmission delay above powerSaveAltitude


//===== Other operation config
bool ledStatusEnable = true;
int ledAutoDisableHeight = 1000; //height in meters above which the status LEDs get disabled
const int xdataPortMode = 1; //0 - disabled, 1 - debug uart, 2 - i2c (NO implementation now), 3 - xdata sensors (oif411)
float vBatWarnValue = 2.5; //battery warning voltage
float vBatErrValue = 2.3; //error voltage
float batteryCutOffVoltage = 0; //good for nimh cell life, below 0.8V per AA cell the damage could occur; if you plan on maximising the working time, set to 0V to disable the auto turn OFF
int ovhtWarnValue = 45; //overheating warning
int ovhtErrValue = 55; //overheating error
int gpsSatsWarnValue = 4; 
bool ubloxGpsAirborneMode = true; //sets the uBlox GPS module to the Airborne 1G Dynamic Model, which should prevent from loosing fix above 18km altitude
int refHeatingMode = 0; //0 - off, 1 - auto, 2 - always on
int refHeaterAutoActivationHeight = 0; //set to 0 to disable auto heater enable above set level, if other than 0 then it means height above which the heater gets auto enabled
unsigned long heaterWorkingTimeSec = 600; //heater heating time
unsigned long heaterCooldownTimeSec = 3; //heater cooldown time after heating process
int autoHeaterThreshold = 6; //auto heater temperature threshold, WARNING! If rtty used baud is faster than 45bdr, the threshold should be at 14*C to prevent the radio from loosing PPL-lock, best would be even to set the heating to always on. If only horus mode is used, it is not mandatory, altough for standard flights that dont require more than 12h of operation the 6*C is advised for defrosting and keeping the internals slightly above ice temperature.
int refHeaterCriticalDisableTemp = 72; //heater critical temp which disables the heating if exceeded
int refHeaterCriticalReenableTemp = 67; //heater temperature at which heating gets re-enabled after being cut down due to too high temperature
int gpsNmeaMsgWaitTime = 1200; //waiting time for gps message
int oif411MsgWaitTime = 1200; //waiting time for oif411 message
int powerSaveAltitude = 3000; //altitude in meters above which the powerSave features start to occur (currently, TX power is lowered from defaultRadioPwrSetting to powerSaveRadioPwrSetting and the transmision interval is changed from modeChangeDelay to powerSaveModeChangeDelay), set to -1 to completely disable all powerSave features
bool sensorBoomEnable = true; //enables sensor boom measurement (currently only temperatures, humidity is being engineered) and diagnostics
float mainTemperatureCorrectionC = 0;
float extHeaterTemperatureCorrectionC = 25;


//===== System internal variables, shouldn't be changed here
int btnCounter = 0;
int bufPacketLength = 64;
int txRepeatCounter = 0;
float batVFactor = 1.0;
bool ledsEnable = ledStatusEnable; //internal boolean used for height disable etc.
#define THERMISTOR_R25 10400  // 10k Ohms at 25°C thermistor 
#define THERMISTOR_B 4295     // Beta parameter calculated thermistor
String rttyMsg;
String morseMsg;
unsigned long gpsTime;
int gpsHours;
int gpsMinutes;
int gpsSeconds;
float gpsLat;
float gpsLong;
float gpsAlt;
float gpsSpeed;
int gpsSats; //system wide variables, for use in functions that dont read the gps on their own
unsigned long heaterOnTime = 0;   // Stores the time when heater was turned on
unsigned long heaterOffTime = 0;  // Stores the time when heater was turned off
bool isHeaterOn = false;          // Tracks the current state of the heater
bool isHeaterPausedOvht = false;
uint8_t heaterDebugState = 0; //5 - off, 11 - ON in auto, 10- auto but OFF, 21 - forced on, 19 - overheated when in auto, 29 - ovht when manual
int deviceDebugState = 0; //heaterdebugstate + [0 if ok; 100 if warn; 200 if err]
bool err = false; //const red light, status state
bool warn = false; //orange light, status state
bool ok = true;; //green light, status state
bool vBatErr = false;
bool vBatWarn = false;
bool ovhtErr = false;
bool ovhtWarn = false;
bool gpsFixWarn = false;
bool sensorBoomWarn = false;
bool sensorBoomErr = false;
int horusPacketCount;
int xdataInstrumentType = 0;
int xdataInstrumentNumber = 0;
float xdataOzonePumpTemperature = 0;
float xdataOzoneCurrent = 0;
float xdataOzoneBatteryVoltage = 0;
int xdataOzonePumpCurrent = 0;
int modeChangeDelay = defaultModeChangeDelay;
int radioPwrSetting = defaultRadioPwrSetting;

// Sensor boom
float mainTemperatureFrequency;
float mainTemperaturePeriod;
float mainTemperatureResistance;
float mainTemperatureValue;
float extHeaterTemperatureFrequency;
float extHeaterTemperaturePeriod;
float extHeaterTemperatureResistance;
float extHeaterTemperatureValue;
uint32_t measFirstEdgeTime = 0;
float tempSensorBoomCalibrationFactor = 0;
bool sensorBoomMainTempError = false;
bool sensorBoomHumidityModuleError = false;
bool sensorBoomGeneralError = false;



//Based on https://github.com/cturvey/RandomNinjaChef/blob/main/uBloxHABceiling.c , and  https://github.com/Nevvman18/rs41-nfw/issues/3
uint8_t ubx_cfg_valset_dyn6[] = { // Series 9 and 10
  0xB5,0x62,0x06,0x8A,0x09,0x00,  // Header/Command/Size  UBX-CFG-VALSET (RAM)
  0x00,0x01,0x00,0x00,0x21,0x00,0x11,0x20,0x06, // Payload data (0x20110021 CFG-NAVSPG-DYNMODEL = 6)
  0xF2,0x4F }; //hardcoded checksum

uint8_t ubx_cfg_valget_dyn6[] = {
    0xB5, 0x62, 0x06, 0x8B, 0x08, 0x00,  // Header for UBX-CFG-VALGET
    0x00, 0x00, 0x00, 0x00,              // Reserved
    0x21, 0x00, 0x11, 0x20,               // Key for CFG-NAVSPG-DYNMODEL
    0xEB, 0x57 //hardcoded checksum
  };


struct uBloxChecksum {
  uint8_t ck_a;
  uint8_t ck_b;
};

struct uBloxHeader {
  uint8_t sync1;
  uint8_t sync2;
  uint8_t messageClass;
  uint8_t messageId;
  uint16_t payloadSize;
};

struct uBloxCFGNAV5Payload {
  uint16_t mask;
  uint8_t dynModel;
  uint8_t fixMode;
  int32_t fixedAlt;
  uint32_t fixedAltVar;
  int8_t minElev;
  uint8_t drLimit;
  uint16_t pDop;
  uint16_t tDop;
  uint16_t pAcc;
  uint16_t tAcc;
  uint8_t staticHoldThresh;
  uint8_t dgpsTimeOut;
  uint32_t reserved2;
  uint32_t reserved3;
  uint32_t reserved4;
};

struct uBloxPacket {
  uBloxHeader header;
  union {
    uBloxCFGNAV5Payload cfgnav5;
  } data;
};




//===== Horus mode deifinitions
// Horus v2 Mode 1 (32-byte) Binary Packet
//== FOR MORE INFO SEE int build_horus_binary_packet_v2(char *buffer) function around line 600
struct HorusBinaryPacketV2
{
    uint16_t     PayloadID;
    uint16_t	Counter;
    uint8_t	Hours;
    uint8_t	Minutes;
    uint8_t	Seconds;
    float	Latitude;
    float	Longitude;
    uint16_t  	Altitude;
    uint8_t     Speed;       // Speed in Knots (1-255 knots)
    uint8_t     Sats;
    int8_t      Temp;        // Twos Complement Temp value.
    uint8_t     BattVoltage; // 0 = 0.5v, 255 = 2.0V, linear steps in-between.
    // The following 9 bytes (up to the CRC) are user-customizable. The following just
    // provides an example of how they could be used.
    int16_t     dummy1;      // unsigned int uint8_t
    int16_t     dummy2;       // Float float
    uint8_t     dummy3;     // battery voltage test uint8_t
    uint16_t     dummy4;     // divide by 10 uint8_t
    uint16_t     dummy5;    // divide by 100 uint16_t
    uint16_t    Checksum;    // CRC16-CCITT Checksum.
}  __attribute__ ((packed));

// Buffers and counters.
char rawbuffer [128];   // Buffer to temporarily store a raw binary packet.
char codedbuffer [128]; // Buffer to store an encoded binary packet
char debugbuffer[256]; // Buffer to store debug strings

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




//===== Radio config functions 

void init_RFM22(void) { //initial radio config, it is dynamic in code but this is the entry configuration for cw to work
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
  //writeRegister(0x71, 0x22);    // GFSK, fd[8]=0, no invert for TX/RX data, FIFO mode, txclk-->gpio
  //writeRegister(0x71, 0x00);    //CW
  //writeRegister(0x71, 0x10);    //FSK
  //writeRegister(0x72, 0x48);    // Frequency deviation setting to 45K=72*625
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

void setRadioFrequency(const float frequency_mhz) {
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
  }
  else if (modulationNumber == 1) {
    writeRegister(0x71, 0b00010001);  //OOK
  }
  else if (modulationNumber == 2) {
    writeRegister(0x71, 0b00010010);  //FSK
  }
  else if (modulationNumber == 3) {
    writeRegister(0x71, 0x22);  //GFSK
  }
  else {
    writeRegister(0x71, 0x00);  //error handling
  }
}



//===== Radio modes functions

//unused buffer mode for now, development implementation seems to transmit something out of the FIFO
void txBuf(void) {
  writeRegister(0x07, 0x01);  // To ready mode

  writeRegister(0x08, 0x03);  // FIFO reset
  writeRegister(0x08, 0x00);  // Clear FIFO

  writeRegister(0x34, 64);               // Preamble = 64 nibbles
  writeRegister(0x3E, bufPacketLength);  // Packet length
  for (unsigned char i = 0; i < bufPacketLength; i++) {
    writeRegister(0x7F, tx_buf[i]);  // Send payload to the FIFO
  }

  writeRegister(0x07, 0x09);  // Start TX

  delay(50);  // Add a small delay to ensure the transmission completes

  writeRegister(0x07, 0x01);  // To ready mode
}

//rtty
void sendRTTYPacket(const char* message) {
  while (*message) {
    sendCharacter(*message++);
  }
}

void sendBit(bool bitValue) {
  if (bitValue) {
    setRadioSmallOffset(RTTY_RADIO_MARK_OFFSET);
  } else {
    setRadioSmallOffset(RTTY_RADIO_SPACE_OFFSET);
  }
  delayMicroseconds(rttyBitDelay); // Fixed delay according to baud rate
  buttonHandler();
}

void sendStartBits() {
  // Send start bits (usually 1 start bit)
  sendBit(0); // Start bit (0)
}

void sendCharacter(char character) {
  // Encode character to RTTY format (assuming default encoding and no special encoding needed for . and -)
  
  // Send start bits
  sendStartBits();

  // Character encoding (use default encoding, 7-bit or 8-bit)
  uint8_t encoding = (uint8_t)character; // Assuming ASCII encoding for characters
  for (int i = 0; i < 8; i++) {
    sendBit((encoding >> i) & 0x01);
  }

  // Send stop bits (1.5 stop bits)
  sendBit(1); // First stop bit
  delayMicroseconds(rttyBitDelay); // Delay between stop bits
  sendBit(1); // Extra stop bit (part of 1.5 stop bits)
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

void fsk4_idle(){
    fsk4_tone(0);
}

void fsk4_preamble(uint8_t len){
    int k;
    for (k=0; k<len; k++){
        fsk4_writebyte(0x1B);
    }
}

size_t fsk4_writebyte(uint8_t b){
    int k;
    // Send symbols MSB first.
    for (k=0;k<4;k++)
    {
        // Extract 4FSK symbol (2 bits)
        uint8_t symbol = (b & 0xC0) >> 6;
        // Modulate
        fsk4_tone(symbol);
        // Shift to next symbol.
        b = b << 2;
    }

  return(1);
}

size_t fsk4_write(char* buff, size_t len){
  size_t n = 0;
  for(size_t i = 0; i < len; i++) {
    n += fsk4_writebyte(buff[i]);
  }
  return(n);
}



//===== Radio payload creation

String createRttyPayload() {
  String payload;
  String addData;

  String formattedLat = String(gpsLat, 5);
  String formattedLong = String(gpsLong, 5);

  if (enableAddData) {
    if(xdataPortMode == 3) {
      String ozoneCurrent = String(xdataOzoneCurrent, 2);  // Convert to string with 2 decimal places
      ozoneCurrent = ozoneCurrent.substring(0, 4);         // Ensure max length of 4 characters

      String batteryVoltage = String(xdataOzoneBatteryVoltage); // Convert to string
      batteryVoltage = batteryVoltage.substring(0, 2);          // Ensure max length of 2 characters

      String pumpTemperature = String(xdataOzonePumpTemperature); // Convert to string
      pumpTemperature = pumpTemperature.substring(0, 2);     // Ensure max length of 2 characters

      // Combine the formatted strings
      String addData = ozoneCurrent + ";" + batteryVoltage + ";" + pumpTemperature;
    }
  } else {
    addData = "0000000000";  // Default additional data
  }

  payload = String(PREAMBLE) + ";" + String(CALLSIGN) + ";" + String(gpsTime) + ";" + String(gpsLat, 5) + ";" + String(gpsLong, 5) + ";" + String(gpsAlt, 1) + ";" + String(gpsSpeed) + ";" + String(gpsSats) + ";" + String(readBatteryVoltage()) + ";" + String(mainTemperatureValue, 1) + ";" + addData;  

  return payload;
}

String createRttyShortPayload() {
  String payload;
  String addData;

  String formattedLat = String(gpsLat, 5);
  String formattedLong = String(gpsLong, 5);

  payload = String(CALLSIGN) + ";" + String(gpsLat, 5) + ";" + String(gpsLong, 5) + ";" + String(gpsAlt, 1) + ";" + String(gpsSats) + ";" + String(mainTemperatureValue) + ";" + String(readThermistorTemp(), 1);  

  return payload;
}

int build_horus_binary_packet_v2(char *buffer){
  // Generate a Horus Binary v2 packet, and populate it with data.
  // The assignments in this function should be replaced with real data
  horusPacketCount++;
  
  struct HorusBinaryPacketV2 BinaryPacketV2;

  BinaryPacketV2.PayloadID = horusPayloadId; // 256 = 4FSKTEST-V2. Refer https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt | you can attempt to modify this according to your needs
  BinaryPacketV2.Counter = horusPacketCount;
  BinaryPacketV2.Hours = gpsHours;
  BinaryPacketV2.Minutes = gpsMinutes;
  BinaryPacketV2.Seconds = gpsSeconds;
  BinaryPacketV2.Latitude = gpsLat;
  BinaryPacketV2.Longitude = gpsLong;
  BinaryPacketV2.Altitude = gpsAlt;
  BinaryPacketV2.Speed = gpsSpeed;
  BinaryPacketV2.BattVoltage = map(readBatteryVoltage() * 100, 0, 5 * 100, 0, 255);
  BinaryPacketV2.Sats = gpsSats;
  BinaryPacketV2.Temp = readThermistorTemp();
  // Custom section. This is an example only, and the 9 bytes in this section can be used in other
  // ways. Refer here for details: https://github.com/projecthorus/horusdemodlib/wiki/5-Customising-a-Horus-Binary-v2-
  
  if(xdataPortMode == 3) { //ozone data tx
    BinaryPacketV2.dummy1 = xdataOzoneCurrent; //-32768 - 32767 int16_t /100
    BinaryPacketV2.dummy2 = xdataOzoneBatteryVoltage * 10; //-32768 - 32767 int16_t /10
    BinaryPacketV2.dummy3 = deviceDebugState; //0 - 255 uint8_t
    BinaryPacketV2.dummy4 = xdataOzonePumpTemperature * 10; //0 - 65535 uint16_t /10
    BinaryPacketV2.dummy5 = 0; //unused in this decoding scheme
  }
  else { //default, matching rs41ng
    BinaryPacketV2.dummy1 = 0; //-32768 - 32767 int16_t
    BinaryPacketV2.dummy2 = mainTemperatureValue * 10; //-32768 - 32767 int16_t
    BinaryPacketV2.dummy3 = 0; //0 - 255 uint8_t
    BinaryPacketV2.dummy4 = 0; //0 - 65535 uint16_t
    BinaryPacketV2.dummy5 = deviceDebugState;
  }
  

  BinaryPacketV2.Checksum = (uint16_t)crc16((unsigned char*)&BinaryPacketV2,sizeof(BinaryPacketV2)-2);

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
unsigned int crc16(unsigned char *string, unsigned int len) {
    unsigned int crc = 0xFFFF; // Initial seed
    for (unsigned int i = 0; i < len; i++) {
        crc = _crc_xmodem_update(crc, string[i]);
    }
    return crc;
}

void PrintHex(char *data, uint8_t length, char *tmp){
 // Print char data as hex
 byte first ;
 int j=0;
 for (uint8_t i=0; i<length; i++) 
 {
   first = ((uint8_t)data[i] >> 4) | 48;
   if (first > 57) tmp[j] = first + (byte)39;
   else tmp[j] = first ;
   j++;

   first = ((uint8_t)data[i] & 0x0F) | 48;
   if (first > 57) tmp[j] = first + (byte)39; 
   else tmp[j] = first;
   j++;
 }
 tmp[length*2] = 0;
}


//===== System operation handlers

void buttonHandler() {
  if (analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN) && analogRead(VBAT_PIN) > 100) {
    if(xdataPortMode == 1) {
      xdataSerial.println("Button pressed");
    }
    btnCounter = 0;
    while (btnCounter < 7 && analogRead(VBTN_PIN) + 50 > analogRead(VBAT_PIN)) {
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(250);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(250);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, HIGH);
      if(xdataPortMode == 1) {
        xdataSerial.print("*");
      }
      btnCounter++;
    }
  }
  if (btnCounter == 1) {

  }
  else if (btnCounter == 2) {
    if (radioEnablePA == true) {
      radioEnablePA = false;
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      if(xdataPortMode == 1) {
        xdataSerial.println("Radio PA disabled");
      }
    }
    else {
      radioEnablePA = true;
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      if(xdataPortMode == 1) {
        xdataSerial.println("Radio PA enabled");
      }
    }
  }
  else if (btnCounter == 3) {
    if(powerSaveRadioPwrSetting == -1) {
      if (defaultRadioPwrSetting != 7) {
        defaultRadioPwrSetting = 7;
        setRadioPower(7);

        digitalWrite(GREEN_LED_PIN, LOW);
        delay(50);
        digitalWrite(GREEN_LED_PIN, HIGH);
        delay(50);
        digitalWrite(GREEN_LED_PIN, LOW);
        delay(50);
        digitalWrite(GREEN_LED_PIN, HIGH);
        delay(50);
        digitalWrite(GREEN_LED_PIN, LOW);
        delay(50);
        digitalWrite(GREEN_LED_PIN, HIGH);
        if(xdataPortMode == 1) {
          xdataSerial.println("Radio PA power set to 100mW (+20dBm, MAX!)");
        }  
      }
      else {
        defaultRadioPwrSetting = 0;
        setRadioPower(0);

        digitalWrite(RED_LED_PIN, LOW);
        delay(50);
        digitalWrite(RED_LED_PIN, HIGH);
        delay(50);
        digitalWrite(RED_LED_PIN, LOW);
        delay(50);
        digitalWrite(RED_LED_PIN, HIGH);
        delay(50);
        digitalWrite(RED_LED_PIN, LOW);
        delay(50);
        digitalWrite(RED_LED_PIN, HIGH);
        if(xdataPortMode == 1) {
          xdataSerial.println("Radio PA power set to 2mW (min)");
        }
      }
    }
    else {
      digitalWrite(RED_LED_PIN, LOW);
      delay(400);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(100);
    }    
  }
  else if (btnCounter == 4) {
    if(rttyEnable != true) {
      rttyEnable = true;

      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      
      if(xdataPortMode == 1) {
        xdataSerial.println("RTTY enabled");
      } 
    }
    else {
      rttyEnable = false;

      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);

      if(xdataPortMode == 1) {
        xdataSerial.println("RTTY disabled");
      }
    }
  }
  else if (btnCounter == 5) {
    if(rttyShortenMsg != true) {
      rttyShortenMsg = true;

      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);

      if(xdataPortMode == 1) {
        xdataSerial.println("RTTY short messages format enabled");
      }
    }
    else {
      rttyShortenMsg = false;

      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      if(xdataPortMode == 1) {
        xdataSerial.println("RTTY standard messages format enabled");
      }
    }
  }
  else if (btnCounter == 6) {
    if(refHeatingMode == 0) { //if it is disabled
      refHeatingMode = 1; //set to auto

      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);

      if(xdataPortMode == 1) {
        xdataSerial.println("Reference heater - AUTO mode!");
      }
    }
    else if(refHeatingMode == 1) { //if set to auto
      refHeatingMode = 2; //set to always on

      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_LED_PIN, LOW);
      delay(50);
      digitalWrite(RED_LED_PIN, HIGH);

      if(xdataPortMode == 1) {
        xdataSerial.println("Reference heater - ALWAYS ON!");
      }
    }
    else {
      refHeatingMode = 0; //turn heating off
      
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(50);
      digitalWrite(GREEN_LED_PIN, LOW);
      delay(50);
      digitalWrite(GREEN_LED_PIN, HIGH);

      if(xdataPortMode == 1) {
        xdataSerial.println("Reference heater - disabled");
      }
    }

  }
  else if (btnCounter == 7) {
    digitalWrite(RED_LED_PIN, LOW);
    radioDisableTx();
    if(xdataPortMode == 1) {
      xdataSerial.println("\n PSU_SHUTDOWN_PIN set HIGH, bye!");
    }
    delay(3000);
    digitalWrite(PSU_SHUTDOWN_PIN, HIGH);
  } else {}

  btnCounter = 0;
}

void deviceStatusHandler() {
  // Temporary flags for each check
  vBatErr = false;
  vBatWarn = false;
  ovhtErr = false;
  ovhtWarn = false;
  gpsFixWarn = false;

  err = false;
  warn = false;
  ok = true; // Default to ok until proven otherwise

  // Evaluate battery voltage
  float vBat = readBatteryVoltage();
  if (vBat < vBatErrValue) {
      vBatErr = true;
  } else if (vBat < vBatWarnValue) {
      vBatWarn = true;
  }

  // Evaluate thermistor temperature
  float temp = readThermistorTemp();
  if (temp > ovhtErrValue && refHeatingMode == 0) {
      ovhtErr = true;
  } else if (temp > ovhtWarnValue && refHeatingMode == 0) {
      ovhtWarn = true;
  }

  // Evaluate GPS status
  if (gpsSats < gpsSatsWarnValue) {
      gpsFixWarn = true;
  }

  // Evaluate sensor boom errors
  if(sensorBoomMainTempError && sensorBoomHumidityModuleError) {
    sensorBoomErr = true;
    sensorBoomWarn = false;
  }
  else if(sensorBoomMainTempError || sensorBoomHumidityModuleError) {
    sensorBoomWarn = true;
    sensorBoomErr = false;
  }
  else {
    sensorBoomWarn = false;
    sensorBoomErr = false;
  }

  // Combine the results to determine the final state
  if (vBatErr || ovhtErr || sensorBoomErr) {
      err = true;
      ok = false;
  } else if (vBatWarn || ovhtWarn || gpsFixWarn || sensorBoomWarn) {
      warn = true;
      ok = false;
  } else {
      // Explicitly clear warning if no errors or warnings are active
      warn = false;
      ok = true;
  }

  // Set deviceDebugState based on error/warn/ok flags
  if(ok) {
    deviceDebugState = 0;
    deviceDebugState += heaterDebugState;
  }
  else if(warn) {
    deviceDebugState = 0;
    deviceDebugState += 100;
    deviceDebugState += heaterDebugState;
  }
  else {
    deviceDebugState = 0;
    deviceDebugState += 200;
    deviceDebugState += heaterDebugState;
  }

  // Cap the debug state within range
  if(deviceDebugState < 0 || deviceDebugState > 255) {
    deviceDebugState = 249;
  }

  // LED Handling
  if(ledStatusEnable) {
    if(gpsAlt > ledAutoDisableHeight) { // disable leds after launch
      ledsEnable = false;
    }
    else {
      ledsEnable = true;
    }

    if(ledsEnable) {
      if(err) {
        digitalWrite(GREEN_LED_PIN, HIGH);  // Green LED on (Error state)
        digitalWrite(RED_LED_PIN, LOW);     // Red LED off
      }
      else if(warn) {
        digitalWrite(GREEN_LED_PIN, LOW);   // Both LEDs off (Warning state)
        digitalWrite(RED_LED_PIN, LOW);
      }
      else if(ok) {
        digitalWrite(GREEN_LED_PIN, LOW);   // Green LED off
        digitalWrite(RED_LED_PIN, HIGH);    // Red LED on (OK state)
      }
    }
  }
  else {
    digitalWrite(GREEN_LED_PIN, HIGH);  // Both LEDs on when LED status is disabled
    digitalWrite(RED_LED_PIN, HIGH);
  }
}


void serialStatusHandler() {
    if(xdataPortMode == 1) {
      if(vBatErr) {
        xdataSerial.println("[ERR]: vBatErr - critically low voltage");
      }

      if(vBatWarn) {
        xdataSerial.println("[WARN]: vBatWarn - low voltage");
      }

      if(ovhtErr) {
        xdataSerial.println("[ERR]: ovhtErr - internal temperature very high");
      }

      if(ovhtWarn) {
        xdataSerial.println("[WARN]: ovhtWarn - internal temperature high");
      }

      if(gpsFixWarn) {
        xdataSerial.println("[WARN]: gpsFix - gps isn't locked on position, give sky clearance, waiting for fix...");
      }

      if(ok) {
        xdataSerial.println("[ok]: Device working properly");
      }
  
  }
    
}


float readBatteryVoltage() {
  float batV;

  if(rsm4x4) { //12bit adc
    batV = ((float)analogRead(VBAT_PIN) / 4095) * 3 * 2 * batVFactor;
  }
  else { //10bit adc
    batV = ((float)analogRead(VBAT_PIN) / 1024) * 3 * 2 * batVFactor;
  }

  return batV;
}

float readThermistorTemp() {
  int adcValue = analogRead(REF_THERM);

  float voltage;

  // Convert ADC value to voltage
  if(rsm4x4) {
    voltage = adcValue * (3.0 / 4095);  // 3.0V reference, 12-bit ADC
  }
  else {
    voltage = adcValue * (3.0 / 1023); //10bit adc
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

uint8_t readRadioTemp() {
  // Read the ADC value from register 0x11
  uint8_t temp = readRegister(0x11);

  // Convert ADC value to signed temperature value
  int16_t temp_2 = -64 + ((int16_t)temp * 5) / 10;

  // Trigger ADC to capture another measurement by writing 0x80 to register 0x0F
  writeRegister(0x0F, 0x80);

  // Cast temperature value to int8_t and return
  return (uint8_t)temp_2;
}

void updateGpsData() {
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < gpsNmeaMsgWaitTime);
  

  gpsTime = gps.time.value() / 100;
  gpsHours = gps.time.hour();
  gpsMinutes = gps.time.minute();
  gpsSeconds = gps.time.second();
  gpsLat = gps.location.lat();
  gpsLong = gps.location.lng();
  gpsAlt = gps.altitude.meters();
  gpsSpeed = gps.speed.mps();
  gpsSats = gps.satellites.value();

  if(xdataPortMode == 1) {
    xdataSerial.println("[info]: GPS data obtained and updated");
  }
}


void disableRefHeaterRing() {
  digitalWrite(PULLUP_TM, LOW); //disable temperature ring oscillator power
  digitalWrite(PULLUP_HYG, LOW); //disable humidity ring oscillator power

  if(rsm4x4) {
    digitalWrite(HEAT_REF, LOW); //disable reference heater
  }
  else if(rsm4x2) {
    writeRegister(0x0C, 0x01); //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
  }
}
  

void enableRefHeaterRing() {
  digitalWrite(PULLUP_TM, LOW); //enable temperature ring oscillator power
  digitalWrite(PULLUP_HYG, LOW); //enable humidity ring oscillator power

  if(rsm4x4) {
    digitalWrite(HEAT_REF, HIGH); //enable reference heater
  }
  else if(rsm4x2) {
    writeRegister(0x0C, 0x00); //change state of GPIO_1 pin output of SI4032 chip (it has 3 configurable GPIOs), older PCBs had heating controlled via its GPIOs
  }
}


void refHeaterHandler() {
    unsigned long currentMillis = millis();
    float currentTemp = readThermistorTemp();

    // Mode 0: Heating Off
    if (refHeatingMode == 0) {
        disableRefHeaterRing(); // Ensure the heater is off

        isHeaterOn = false;
        heaterDebugState = 5;

        if(xdataPortMode == 1) {
          xdataSerial.println("[info]: Heating is completely OFF.");
        }
        
        return; // Exit early
    }

    // Mode 1: Auto Mode
    if (refHeatingMode == 1) {
        // Disable heater if temperature exceeds the critical disable threshold
        if (currentTemp > refHeaterCriticalDisableTemp) {
            disableRefHeaterRing(); // Turn off the heater

            if(xdataPortMode == 1) {
              xdataSerial.print("[ERR]: Heater has been turned OFF because temp exceeds threshold of (*C): ");
              xdataSerial.println(refHeaterCriticalDisableTemp);
            }
            
            heaterOffTime = currentMillis; // Record the time the heater was turned off

            isHeaterOn = false; // Update the heater state
            isHeaterPausedOvht = true; // Mark that the heater was paused due to overheating
            heaterDebugState = 19;

            return; // Exit early
        }

        // Re-enable the heater if it was paused due to overheating and the temperature drops below the re-enable threshold
        if (isHeaterPausedOvht && currentTemp <= refHeaterCriticalReenableTemp) {
          if(xdataPortMode == 1) {
            xdataSerial.print("[info]: Temperature dropped below re-enable threshold of (*C): ");
            xdataSerial.println(refHeaterCriticalReenableTemp);
          }
            
            isHeaterPausedOvht = false; // Clear the overheating flag

            enableRefHeaterRing(); // Turn on the heater

            if(xdataPortMode == 1) {
              xdataSerial.println("[info]: HEATER RE-ENABLED!");
            }

            isHeaterOn = true; // Update the heater state
            heaterDebugState = 11;

            return; // Exit early after re-enabling
        }

        // Disable heater if the timer condition is met
        if (isHeaterOn && currentMillis - heaterOnTime >= heaterWorkingTimeSec * 1000) {
            disableRefHeaterRing(); // Turn off the heater
            if(xdataPortMode == 1) {
              xdataSerial.print("[info]: Heater OFF due to timer, cooldown for (seconds): ");
              xdataSerial.println(heaterCooldownTimeSec);
            }

            heaterOffTime = currentMillis; // Record the time the heater was turned off
            isHeaterOn = false; // Update the heater state
            heaterDebugState = 10;
        }
        else if (isHeaterOn) {
          if(xdataPortMode == 1) {
            xdataSerial.print("[info]: Heater is currently ON, at (*C): ");
            xdataSerial.println(currentTemp);
          }

          heaterDebugState = 11;
        }
        else {
            // If the heater is off, check if cooldown has passed and if it should be re-enabled
            if (currentMillis - heaterOffTime >= heaterCooldownTimeSec * 1000) {
                if (currentTemp < autoHeaterThreshold) {
                  if(xdataPortMode == 1) {
                    xdataSerial.print("[info]: Cooldown time passed, thermistor temp is under threshold of (*C): ");
                    xdataSerial.println(autoHeaterThreshold);
                    xdataSerial.println("[info]: HEATER ON!");
                  }

                    enableRefHeaterRing(); // Turn on the heater

                    heaterOnTime = currentMillis; // Record the time the heater was turned on
                    isHeaterOn = true; // Update the heater state
                    heaterDebugState = 11;
                }
                else {
                  if(xdataPortMode == 1) {
                    xdataSerial.println("[info]: autoRefHeating is enabled, but the temperature is higher than the threshold.");
                  }
                    
                    heaterDebugState = 10;
                }
            }
        }
        return; // Exit early
    }

    // Mode 2: Heater Always On with Safety Control
    if (refHeatingMode == 2) {
        // Check if the temperature exceeds the critical disable threshold
        if (currentTemp > refHeaterCriticalDisableTemp) {
            disableRefHeaterRing(); // Turn off the heater

            if(xdataPortMode == 1) {
              xdataSerial.print("[ERR]: Heater has been turned OFF because temp exceeds threshold of (*C): ");
              xdataSerial.println(refHeaterCriticalDisableTemp);
            }
            
            heaterOffTime = currentMillis; // Record the time the heater was turned off
            isHeaterOn = false; // Update the heater state
            isHeaterPausedOvht = true; // Mark that the heater was paused due to overheating
            heaterDebugState = 29;

            return; // Exit early
        }

        // Re-enable the heater immediately after temperature drops below re-enable threshold
        if (isHeaterPausedOvht && currentTemp <= refHeaterCriticalReenableTemp) {
          if(xdataPortMode == 1) {
            xdataSerial.print("[info]: Temperature dropped below re-enable threshold of (*C): ");
            xdataSerial.println(refHeaterCriticalReenableTemp);
          }
            
            isHeaterPausedOvht = false; // Clear the overheating flag

            enableRefHeaterRing(); // Turn on the heater

            if(xdataPortMode == 1) {
              xdataSerial.println("[info]: HEATER RE-ENABLED!");
            }

            isHeaterOn = true; // Update the heater state
            heaterOnTime = currentMillis; // Reset the heater's working time
            heaterDebugState = 21;
        }
        else if (!isHeaterOn) {
            // If the heater is not on, ensure it is turned on
            enableRefHeaterRing(); // Turn on the heater

            if(xdataPortMode == 1) {
              xdataSerial.println("[info]: Heating is always ON with safety control.");
            }
            
            isHeaterOn = true; // Update the heater state
            heaterOnTime = currentMillis; // Reset the heater's working time
            heaterDebugState = 21;
        }
        return; // Exit early
    }
}

void refHeaterHeightActivator() {
  if(refHeaterAutoActivationHeight != 0) {
    if(gpsAlt > refHeaterAutoActivationHeight) {
      refHeatingMode = 2;
    }
    else if(gpsAlt < refHeaterAutoActivationHeight) {
      refHeatingMode = 1;
    }
    else {
      refHeatingMode = 1;
    }
  }
}


void powerHandler() {
  if(readBatteryVoltage() < batteryCutOffVoltage && batteryCutOffVoltage != 0) {
    digitalWrite(RED_LED_PIN, LOW);
    delay(100);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(100);
    digitalWrite(RED_LED_PIN, LOW);
    delay(100);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(100);
    digitalWrite(RED_LED_PIN, LOW);
    delay(100);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(100);
    digitalWrite(RED_LED_PIN, LOW);
    delay(100);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(100);
    digitalWrite(RED_LED_PIN, LOW);

    setRadioModulation(0);  // CW modulation
    setRadioFrequency(rttyFrequencyMhz);

    if(xdataPortMode == 1) {
      xdataSerial.println("[ERR] BATTERY CUT-OFF VOLTAGE, SYSTEM WILL POWER OFF");
    }

    radioEnableTx();
    setRadioSmallOffset(RTTY_RADIO_MARK_OFFSET);
    delay(250); //mark character idle
    sendRTTYPacket("\n\n VBAT-CUTOFF VBAT-CUTOFF VBAT-CUTOFF");
    radioDisableTx();

    if(xdataPortMode == 1) {
      xdataSerial.println("\n PSU_SHUTDOWN_PIN set HIGH, bye!");
    }
    
    delay(250);
    digitalWrite(PSU_SHUTDOWN_PIN, HIGH);
  }

  if(powerSaveAltitude != -1) {
    if(gpsAlt > powerSaveAltitude) {
      if(powerSaveRadioPwrSetting != -1) {
        radioPwrSetting = powerSaveRadioPwrSetting;
        setRadioPower(radioPwrSetting);
      }

      if(powerSaveModeChangeDelay != -1) {
        modeChangeDelay = powerSaveModeChangeDelay;
      }

    }
    else {
      if(powerSaveRadioPwrSetting != -1) {
        radioPwrSetting = defaultRadioPwrSetting;
        setRadioPower(radioPwrSetting);
      }

      if(powerSaveModeChangeDelay != -1) {
        modeChangeDelay = defaultModeChangeDelay;
      }
    }  
  }
}


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
    int rawBatteryVoltage = strtol(data.substring(13, 15).c_str(), NULL, 16);
    xdataOzoneBatteryVoltage = rawBatteryVoltage / 10.0;  // Convert to Volts (V)
    
    // Extract and decode ozone pump current
    xdataOzonePumpCurrent = strtol(data.substring(15, 18).c_str(), NULL, 16);
  }
}

// Based on: https://github.com/cturvey/RandomNinjaChef/blob/main/uBloxChecksum.c and others from this repo, big thanks!
void uBloxM10Checksum(uint8_t *data) // Assumes buffer is large enough and modifyable
{
  int i, length;
  uint8_t a, b;

  a = 0; // Clear initial checksum bytes
  b = 0;

  length = data[4] + (data[5] << 8); // 16-bit Payload Length
  
  for(i=2; i<(length + 6); i++) // Sum over body
  {
    a += data[i];
    b += a;
  }

  data[i+0] = a; // Write checksum bytes into tail
  data[i+1] = b;
}
 
void sendUbloxM10(int Size, uint8_t *Buffer)
{
  //uBloxM10Checksum(Buffer); // Add/compute checksum bytes for packet 
  gpsSerial.write(Buffer, Size);  // Arduino style byte send
}

bool ubloxM10DynMod() {
  if (xdataPortMode == 1) {
    xdataSerial.println("[info] Sending UBX-CFG-VALSET command to set dynamic model to 6...");
  }
  sendUbloxM10(sizeof(ubx_cfg_valset_dyn6), ubx_cfg_valset_dyn6);

  return true; //temporary, the checking was faulty
}



uBloxChecksum ublox_calc_checksum(const uint8_t msgClass, const uint8_t msgId, const uint8_t *message, uint16_t size) {
  uBloxChecksum ck = {0, 0};
  ck.ck_a += msgClass;
  ck.ck_b += ck.ck_a;
  ck.ck_a += msgId;
  ck.ck_b += ck.ck_a;
  ck.ck_a += size & 0xff;
  ck.ck_b += ck.ck_a;
  ck.ck_a += size >> 8;
  ck.ck_b += ck.ck_a;

  for (uint16_t i = 0; i < size; i++) {
    ck.ck_a += message[i];
    ck.ck_b += ck.ck_a;
  }
  return ck;
}

void sendUbloxOld(uint8_t msgClass, uint8_t msgId, uint8_t *payload, uint16_t payloadSize) {
  uBloxChecksum chksum = ublox_calc_checksum(msgClass, msgId, payload, payloadSize);

  gpsSerial.write(0xB5);
  gpsSerial.write(0x62);
  gpsSerial.write(msgClass);
  gpsSerial.write(msgId);
  gpsSerial.write((uint8_t)(payloadSize & 0xff));
  gpsSerial.write((uint8_t)(payloadSize >> 8));

  for (uint16_t i = 0; i < payloadSize; ++i) {
    gpsSerial.write(payload[i]);
  }
  gpsSerial.write(chksum.ck_a);
  gpsSerial.write(chksum.ck_b);
}

void sendUbloxOldPacket(uBloxPacket *packet) {
  sendUbloxOld(packet->header.messageClass, packet->header.messageId, (uint8_t*)&packet->data, packet->header.payloadSize);
}


// Function to select reading of a sensor and set its state (on/off)
void selectSensorBoom(int sensorNum, int state) {
    // Ensure state is either 0 (off) or 1 (on), return if invalid
    if (state != 0 && state != 1) {
        return;  // Invalid state, do nothing
    }
    
    // Set powerState based on the valid state
    int powerState = (state == 1) ? HIGH : LOW;

    switch(sensorNum) {
        case 0:  // All sensors
            digitalWrite(SPST1, powerState);  // Reference 1
            digitalWrite(SPST2, powerState);  // Reference 2
            digitalWrite(SPST3, powerState);  // External Heater Temp
            digitalWrite(SPST4, powerState);  // Main Temperature
            digitalWrite(PULLUP_TM, powerState);  // Common PULLUP for all
            break;

        case 1:  // Reference 1 92khz (SPST1 and PULLUP_TM)
            digitalWrite(SPST1, powerState);
            digitalWrite(PULLUP_TM, powerState);
            break;

        case 2:  // Reference 2 62khz (SPST2 and PULLUP_TM)
            digitalWrite(SPST2, powerState);
            digitalWrite(PULLUP_TM, powerState);
            break;

        case 3:  // External Heater Temperature (SPST3 and PULLUP_TM)
            digitalWrite(SPST3, powerState);
            digitalWrite(PULLUP_TM, powerState);
            break;

        case 4:  // Main Temperature (SPST4 and PULLUP_TM)
            digitalWrite(SPST4, powerState);
            digitalWrite(PULLUP_TM, powerState);
            break;

        default:
            // Invalid sensor number, do nothing
            break;
    }
}

float getSensorBoomPeriod(int sensorNum) {
    #define READ_PIN() (GPIOA->IDR & (1 << 1))  // PA1 corresponds to bit 1 in GPIOA IDR

    selectSensorBoom(sensorNum, 1);
    delay(100);

    unsigned long long measStartTime = micros(); // Start time for the timeout
    unsigned long long measTimeoutDuration = 500000; // .5 seconds in microseconds

    // Wait for the signal to go low
    while (READ_PIN() != 0) {
        if (micros() - measStartTime > measTimeoutDuration) { // Check for timeout
            selectSensorBoom(sensorNum, 0); // Ensure sensor is deselected
            return -1; // Return an error value (can be adjusted as needed)
        }
    }

    // Wait for the signal to go high
    while (READ_PIN() == 0) {
        if (micros() - measStartTime > measTimeoutDuration) { // Check for timeout
            selectSensorBoom(sensorNum, 0); // Ensure sensor is deselected
            return -1; // Return an error value (can be adjusted as needed)
        }
    }

    unsigned long long measFirstEdgeTime = micros();

    for (int i = 0; i < 30000; i++) { // 30000 measurements averaged
        // Wait for the signal to go low (falling edge)
        while (READ_PIN() != 0) {
            if (micros() - measStartTime > measTimeoutDuration) { // Check for timeout
                selectSensorBoom(sensorNum, 0); // Ensure sensor is deselected
                return -1; // Return an error value (can be adjusted as needed)
            }
        }

        // Wait for the signal to go high (rising edge)
        while (READ_PIN() == 0) {
            if (micros() - measStartTime > measTimeoutDuration) { // Check for timeout
                selectSensorBoom(sensorNum, 0); // Ensure sensor is deselected
                return -1; // Return an error value (can be adjusted as needed)
            }
        }
    }

    unsigned long long measLastEdgeTime = micros();

    selectSensorBoom(sensorNum, 0);

    return ((measLastEdgeTime - measFirstEdgeTime) / 30000.0);
}

float getSensorBoomFreq(int sensorNum) {
    float period = getSensorBoomPeriod(sensorNum);
    if (period > 0) {
        return 1000000 / period;
    } else {
        return 0;
    }
}

// Function to calibrate the sensor using the two calibration resistors
float calibrateTempSensorBoom() {
  // Get the frequencies for calibration resistors
  float freq750 = getSensorBoomFreq(1);  // Get frequency for 750Ω resistor
  float freq1010 = getSensorBoomFreq(2); // Get frequency for 1010Ω resistor

  // Calibration constant k: R * f (frequency-to-resistance ratio)
  // We use an average to balance between the two calibration resistors.
  float k = (750.0 * freq750 + 1010.0 * freq1010) / 2.0;

  return k;  // Return the calibration constant
}

float calculateSensorBoomResistance(float freq, float k) {
  return k / freq;
}

// Function to convert PT1000 resistance to temperature in Celsius
float convertPt1000ResToTemp(float resistance) {
    const float R0 = 1000.0;  // Resistance at 0°C
    const float alpha = 0.00385;  // Temperature coefficient of resistance

    // Calculate temperature using the formula
    float temperature = (resistance - R0) / (R0 * alpha);

    return temperature;
}

void humiditySensorHeaterControl(int heaterPower) { //0 - OFF, 1 - small power, 2 - full power
  if(heaterPower == 0) {
    digitalWrite(HEAT_HUM1, LOW);
    digitalWrite(HEAT_HUM2, LOW);
  }
  else if(heaterPower == 1) {
    digitalWrite(HEAT_HUM1, LOW);
    digitalWrite(HEAT_HUM2, HIGH);
  }
  else if(heaterPower == 2) {
    digitalWrite(HEAT_HUM1, HIGH);
    digitalWrite(HEAT_HUM2, HIGH);
  }
  else {
    digitalWrite(HEAT_HUM1, LOW);
    digitalWrite(HEAT_HUM2, LOW);
  }
}

void sensorBoomHandler() {
  static int callCount = 0;
  if(sensorBoomEnable) {
    // Calibrate the sensor and get the calibration factor
    tempSensorBoomCalibrationFactor = calibrateTempSensorBoom();
    
    // Get main temperature frequency
    mainTemperatureFrequency = getSensorBoomFreq(4);
    if(mainTemperatureFrequency <= 0) {
      sensorBoomMainTempError = true;  // Error if frequency is invalid
      if(xdataPortMode == 1) {
        xdataSerial.println("[WARN]: Main temperature hook sensor boom error! MEAS frequency invalid. The hook may have snapped.");
      }
    } else {
      sensorBoomMainTempError = false;  // No error
      mainTemperatureResistance = calculateSensorBoomResistance(mainTemperatureFrequency, tempSensorBoomCalibrationFactor);
      mainTemperatureValue = convertPt1000ResToTemp(mainTemperatureResistance) + mainTemperatureCorrectionC;
    }

    // Get external heater temperature frequency
    extHeaterTemperatureFrequency = getSensorBoomFreq(3);
    if(extHeaterTemperatureFrequency <= 0) {
      sensorBoomHumidityModuleError = true;  // Error if frequency is invalid
      if(xdataPortMode == 1) {
        xdataSerial.println("[WARN]: External humidity heater temperature sensor error! MEAS frequency invalid.");
      }
    } else {
      sensorBoomHumidityModuleError = false;  // No error
      extHeaterTemperatureResistance = calculateSensorBoomResistance(extHeaterTemperatureFrequency, tempSensorBoomCalibrationFactor);
      extHeaterTemperatureValue = convertPt1000ResToTemp(extHeaterTemperatureResistance) + extHeaterTemperatureCorrectionC;      
    }

    // Check overall sensor status
    sensorBoomGeneralError = sensorBoomMainTempError || sensorBoomHumidityModuleError;

    if(sensorBoomMainTempError && sensorBoomHumidityModuleError) {
      if(xdataPortMode == 1) {
        xdataSerial.println("[ERR]: The sensor boom seems disconnected!");
      }
    }
  }
}





void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);  
  pinMode(PSU_SHUTDOWN_PIN, OUTPUT);
  pinMode(CS_RADIO_SPI, OUTPUT);
  if(heaterPinControlAvail) {
    pinMode(HEAT_REF, OUTPUT);
  }
  pinMode(PULLUP_TM, OUTPUT);
  pinMode(PULLUP_HYG, OUTPUT);
  pinMode(SPST1, OUTPUT);
  pinMode(SPST2, OUTPUT);
  pinMode(SPST3, OUTPUT);
  pinMode(SPST4, OUTPUT);
  pinMode(MEAS_OUT, INPUT);

  pinMode(HEAT_HUM1, OUTPUT);
  pinMode(HEAT_HUM2, OUTPUT);

  digitalWrite(PULLUP_TM, LOW);
  digitalWrite(PULLUP_HYG, LOW);
  digitalWrite(SPST1, LOW);
  digitalWrite(SPST2, LOW);
  digitalWrite(SPST3, LOW);
  digitalWrite(SPST4, LOW);
  
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, HIGH);

  if(xdataPortMode == 1) {
    xdataSerial.begin(115200);
  }
  else if(xdataPortMode == 3) {
    xdataSerial.begin(9600);
  }
  
  if(rsm4x4){
    gpsSerial.begin(gpsBaudRate);
  }
  else if(rsm4x2) {
    gpsSerial.begin(gpsBaudRate);
  }
  
  if(xdataPortMode == 1) {
    xdataSerial.println("[info]: Serial interfaces initialized");
  }

  if(rsm4x4) {
    analogReadResolution(12);
    if(xdataPortMode == 1) {
      xdataSerial.println("[info]: ADC resolution set to 12 bits");
    }
  }
  

  SPI_2.begin();
  digitalWrite(CS_RADIO_SPI, HIGH);  // Deselect the SI4432 CS pin
  if(xdataPortMode == 1) {
    xdataSerial.println("[info]: SPI_2 interface initialized");
  }

  digitalWrite(CS_RADIO_SPI, LOW);
  init_RFM22();
  if(xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 radio register initialization complete");
  }
  //digitalWrite(CS_RADIO_SPI, HIGH); //no need to disable cs because no other spi devices on the bus

  setRadioPower(radioPwrSetting);
  if(xdataPortMode == 1) {
    xdataSerial.print("[info]: Si4032 PA power set to (pre-config): ");
    xdataSerial.println(radioPwrSetting);
  }
  
  writeRegister(0x72, 0x07);  //deviation ~12khz, currently no use because of manual cw modes (rtty morse pip)

  if(xdataPortMode == 1) {
    xdataSerial.println("[info]: Si4032 deviation set to 0x07, not used for now?...");
  }
  
  fsk4_bitDuration = (uint32_t)1000000/horusBdr; //horus 100baud delay calculation


  disableRefHeaterRing(); //turn off reference heating
  humiditySensorHeaterControl(0); //turn off humidity sensor heater
  selectSensorBoom(0, 0); //turn off all sensor boom measurement circuits


  if(ubloxGpsAirborneMode) { //sending twice as an experimental way, the value reading wasn't figured out yet
    if(rsm4x4) {
      if(xdataPortMode == 1) {
        xdataSerial.println("[info] Setting the Airborne 1G (6) GPS dynamic model for uBlox M10...");
      }      

      if(!ubloxM10DynMod()) {
        for(int i = 0; i < 5; i++) {
          digitalWrite(RED_LED_PIN, LOW);
          delay(250);
          digitalWrite(RED_LED_PIN, HIGH);
          delay(250);
        }
      }
            
    }
    else if(rsm4x2) {
      uBloxPacket msgcfgnav5;
      msgcfgnav5.header.sync1 = 0xb5;
      msgcfgnav5.header.sync2 = 0x62;
      msgcfgnav5.header.messageClass = 0x06;
      msgcfgnav5.header.messageId = 0x24;
      msgcfgnav5.header.payloadSize = sizeof(uBloxCFGNAV5Payload);

      msgcfgnav5.data.cfgnav5.mask = 0b00000001111111111;
      msgcfgnav5.data.cfgnav5.dynModel = 6;
      msgcfgnav5.data.cfgnav5.fixMode = 2;
      msgcfgnav5.data.cfgnav5.fixedAlt = 0;
      msgcfgnav5.data.cfgnav5.fixedAltVar = 10000;
      msgcfgnav5.data.cfgnav5.minElev = 5;
      msgcfgnav5.data.cfgnav5.drLimit = 0;
      msgcfgnav5.data.cfgnav5.pDop = 100;
      msgcfgnav5.data.cfgnav5.tDop = 100;
      msgcfgnav5.data.cfgnav5.pAcc = 100;
      msgcfgnav5.data.cfgnav5.tAcc = 200;
      msgcfgnav5.data.cfgnav5.staticHoldThresh = 0;
      msgcfgnav5.data.cfgnav5.dgpsTimeOut = 2;
      msgcfgnav5.data.cfgnav5.reserved2 = 0;
      msgcfgnav5.data.cfgnav5.reserved3 = 0;
      msgcfgnav5.data.cfgnav5.reserved4 = 0;

      sendUbloxOldPacket(&msgcfgnav5);
      delay(750);
      sendUbloxOldPacket(&msgcfgnav5);
    }
    
  }

  digitalWrite(GREEN_LED_PIN, LOW);
  delay(50);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(200);
  digitalWrite(GREEN_LED_PIN, LOW);
  delay(50);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(200);

  if(xdataPortMode == 1) {
    xdataSerial.println("[info]: Exiting setup...");
  }

}


void loop() {
  buttonHandler();
  deviceStatusHandler(); 
  serialStatusHandler();
  refHeaterHandler();
  powerHandler();
  refHeaterHeightActivator();
  sensorBoomHandler();
  

  if(xdataPortMode == 3) {
    xdataInstrumentHandler();
  }

  if (pipEnable) {
    if(xdataPortMode == 1) {
      xdataSerial.println("[info]: PIP mode enabled");
    }
    
    if (radioEnablePA) {
      setRadioModulation(0);
      setRadioFrequency(pipFrequencyMhz);

      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: PIP on (MHz): ");
        xdataSerial.println(pipFrequencyMhz);
        xdataSerial.println("[info]: Transmitting PIP...");
      }      

      for (txRepeatCounter; txRepeatCounter < pipRepeat; txRepeatCounter++) {
        radioEnableTx();

        if(xdataPortMode == 1) {
          xdataSerial.print("pip ");
        }
        
        delay(pipLengthMs);
        buttonHandler();
        deviceStatusHandler();

        radioDisableTx();
        delay(pipLengthMs);
        buttonHandler();
        deviceStatusHandler();
      }

      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: PIP TX done");
      }
      
      txRepeatCounter = 0;
    }
    else {
      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
    }

    if(modeChangeDelay == 0) {}
    else if(modeChangeDelay != 0 && modeChangeDelay < 1000) {
      delay(modeChangeDelay);
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
        xdataSerial.println(modeChangeDelay);
      }
    }
    else {
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
        xdataSerial.println(modeChangeDelay);
      }
      
      if(radioSleep) {
        radioInhibitTx();

        if(xdataPortMode == 1) {
          xdataSerial.println("[info]: Radio sleep");
        }

      }

      delay(modeChangeDelay);
    }
  }

  buttonHandler();
  deviceStatusHandler();

  if (morseEnable) {
    if(xdataPortMode == 1) {
      xdataSerial.println("[info]: Morse mode enabled");
    }

    if (radioEnablePA) {
      morseMsg = createRttyShortPayload();
      const char* morseMsgCstr = morseMsg.c_str();
      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: Morse payload created: ");
        xdataSerial.println(morseMsg);
        xdataSerial.println();
      }
      
      setRadioModulation(0);  // CW modulation
      setRadioFrequency(morseFrequencyMhz);

      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: Morse transmitting on (MHz): ");
        xdataSerial.println(morseFrequencyMhz);
      }

      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: Transmitting morse...");
      }
      
      transmitMorseString(morseMsgCstr, morseUnitTime);
      radioDisableTx();

      if(xdataPortMode == 1) {
      xdataSerial.println("[info]: Morse TX done");
      }
      
    }
    else {
      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won't transmit");
      }
      
    }


    if(modeChangeDelay == 0) {}
      else if(modeChangeDelay != 0 && modeChangeDelay < 1000) {
        delay(modeChangeDelay);
        if(xdataPortMode == 1) {
          xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
          xdataSerial.println(modeChangeDelay);
        }
      }
      else {
        if(xdataPortMode == 1) {
          xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
          xdataSerial.println(modeChangeDelay);
        }
        
        if(radioSleep) {
          radioInhibitTx();

          if(xdataPortMode == 1) {
            xdataSerial.println("[info]: Radio sleep");
          }

        }

        delay(modeChangeDelay);
      }

  }

  buttonHandler();
  deviceStatusHandler();


  if (rttyEnable) {
    if(xdataPortMode == 1) {
      xdataSerial.println("[info]: RTTY mode enabled");
    }
    
    if (radioEnablePA) {
      if(rttyShortenMsg) {
        rttyMsg = createRttyShortPayload();
      }
      else {
        rttyMsg = createRttyPayload();
      }
      
      const char* rttyMsgCstr = rttyMsg.c_str();
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: RTTY payload created: ");
        xdataSerial.println(rttyMsg);
        xdataSerial.println("");
      }
      

      setRadioModulation(0);  // CW modulation
      setRadioFrequency(rttyFrequencyMhz);
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: RTTY frequency set to (MHz): ");
        xdataSerial.println(rttyFrequencyMhz);
      }
      

      radioEnableTx();

      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: Transmitting RTTY");
      }
      
      setRadioSmallOffset(RTTY_RADIO_MARK_OFFSET);
      delay(250); //mark character idle
      sendRTTYPacket(rttyMsgCstr);
      sendRTTYPacket("\n\n");
      radioDisableTx();

      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: RTTY TX done");
      }
      
    }
    else {
      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won' transmit");
      }
    }
  
    if(modeChangeDelay == 0) {}
    else if(modeChangeDelay != 0 && modeChangeDelay < 1000) {
      delay(modeChangeDelay);
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
        xdataSerial.println(modeChangeDelay);
      }
    }
    else {
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
        xdataSerial.println(modeChangeDelay);
      }
      
      if(radioSleep) {
        radioInhibitTx();

        if(xdataPortMode == 1) {
          xdataSerial.println("[info]: Radio sleep");
        }

      }

      delay(modeChangeDelay);
    }
  }

  buttonHandler();
  deviceStatusHandler();

  if (horusEnable) {
    if(xdataPortMode == 1) {
      xdataSerial.println("[info]: HORUS mode enabled");
    }
    
    if (radioEnablePA) {
      int pkt_len = build_horus_binary_packet_v2(rawbuffer);
      int coded_len = horus_l2_encode_tx_packet((unsigned char*)codedbuffer,(unsigned char*)rawbuffer,pkt_len);

      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: HORUS payload created.");

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
      

      setRadioModulation(0);  // CW modulation
      setRadioFrequency(horusFrequencyMhz);
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: HORUS frequency set to (MHz): ");
        xdataSerial.println(horusFrequencyMhz);
      }
      
      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: Transmitting HORUS");
      }
      
      radioEnableTx();

      fsk4_idle();
      delay(1000);
      fsk4_preamble(8);
      fsk4_write(codedbuffer, coded_len);

      radioDisableTx();

      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: HORUS TX done");
      }
      
    }
    else {
      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: radioEnablePA false, won' transmit");
      }
    }
    if(modeChangeDelay == 0) {}
    else if(modeChangeDelay != 0 && modeChangeDelay < 1000) {
      delay(modeChangeDelay);
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
        xdataSerial.println(modeChangeDelay);
      }
    }
    else {
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
        xdataSerial.println(modeChangeDelay);
      }
      
      if(radioSleep) {
        radioInhibitTx();

        if(xdataPortMode == 1) {
          xdataSerial.println("[info]: Radio sleep");
        }

      }

      delay(modeChangeDelay);
    }
  }

  buttonHandler();
  deviceStatusHandler();

  if (bufTxEnable) {
    if(xdataPortMode == 1) {
      xdataSerial.println("[info]: bufTx mode enabled");
    }
    
    if (radioEnablePA) {
      setRadioModulation(2);
      setRadioFrequency(fskFrequencyMhz);
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: Transmitting on (MHz): ");
        xdataSerial.println(fskFrequencyMhz);
      }
      
      txBuf();
      
      if(xdataPortMode == 1) {
        xdataSerial.println("[info]: TX done.");
      }
      
    }
  }

  if(radioSleep) {
    radioInhibitTx();

    if(xdataPortMode == 1) {
      xdataSerial.println("[info]: Radio sleep");
    }

    if(modeChangeDelay == 0) {}
    else if(modeChangeDelay != 0 && modeChangeDelay < 1000) {
      delay(modeChangeDelay);
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
        xdataSerial.println(modeChangeDelay);
      }
    }
    else {
      if(xdataPortMode == 1) {
        xdataSerial.print("[info]: modeChangeDelay enabled, waiting for: ");
        xdataSerial.println(modeChangeDelay);
      }
      
      if(radioSleep) {
        radioInhibitTx();

        if(xdataPortMode == 1) {
          xdataSerial.println("[info]: Radio sleep");
        }

      }

      delay(modeChangeDelay);
    }

  }

  buttonHandler();
  deviceStatusHandler();

  updateGpsData();

  if(xdataPortMode == 1) { //only for development
    /*xdataSerial.println("[info]: DEBUG DATA: ");
    xdataSerial.print("Lat, long: ");
    xdataSerial.print(String(gpsLat, 6));
    xdataSerial.print(" ,  ");
    xdataSerial.print(String(gpsLong, 6));
    xdataSerial.print("\nSat: ");
    xdataSerial.print(gpsSats);
    xdataSerial.print("\nTime: ");
    xdataSerial.print(gpsTime);
    xdataSerial.print("\nSpeed: ");
    xdataSerial.print(gpsSpeed);
    xdataSerial.print("\nAltitude: ");
    xdataSerial.print(gpsAlt);
    xdataSerial.print("\n");

    xdataSerial.print(readBatteryVoltage());
    xdataSerial.print("\n");
    xdataSerial.print(readThermistorTemp());
    xdataSerial.print("\n");
    xdataSerial.print(readRadioTemp());
    xdataSerial.print("\n\n");

    xdataSerial.println(String(rttyMsg));

    xdataSerial.println(analogRead(VBTN_PIN));
    xdataSerial.println(analogRead(VBAT_PIN));
    xdataSerial.println("\n\n\n");*/
  }
  
  

  if (!radioEnablePA) {
    delay(500);

    if(xdataPortMode == 1) {
      xdataSerial.println("[info]: Radio PA not enabled, sleep for 500ms");
    }
  }

  if(xdataPortMode == 1) {
    xdataSerial.println("\n\n");
  }
  
}
