#ifndef __UBXGNSS_H
#define __UBXGNSS_H

// ------------------------------------------------------------------------
// [RS41-NFW-SA] Source-Available Module - this ENTIRE file is NOT under GPL-3.0.
// It is covered by the RS41-NFW Source-Available License. See LICENSING.md and
// LICENSE.source-available.
//
// RS41-NFW native u-blox UBX GNSS driver
//
// Replaces the former TinyGPS++ NMEA parser. The firmware talks to the GPS
// entirely in the binary UBX protocol now - no NMEA is parsed or emitted.
//
//   RSM4x4 / RSM4x5 (u-blox M10) : UBX-NAV-PVT  (one message = everything)
//   RSM4x2 / RSM4x1 (u-blox 6)   : UBX-NAV-POSLLH + NAV-VELNED + NAV-SOL
//                                  + NAV-TIMEUTC (M6 predates NAV-PVT)
//   jamming/interference         : UBX-MON-RF (M10) / UBX-MON-HW (M6)
//
// The public accessor surface (gps.time.*, gps.location.lat()/lng(),
// gps.altitude.meters(), gps.speed.mps()/kmph(), gps.satellites.value(),
// gps.hdop.hdop()) is kept identical to the old parser so the rest of the
// firmware is unchanged. New in this driver: a true GPS-computed vertical
// velocity (from velD), fix-validity flags, satellite count beyond the old
// NMEA cap of 12, position/velocity accuracy estimates and a hardware
// jamming indicator.
//
// NOTE: gps.hdop now carries pDOP (position DOP), not hDOP - NAV-PVT reports
// pDOP natively. See CONFIG.h / OPERATION_MANUAL.
// ------------------------------------------------------------------------

#include <inttypes.h>
#include "Arduino.h"
#include <limits.h>

#define _GPS_VERSION        "2.0.0-RS41-UBX"
#define _GPS_FEET_PER_METER 3.2808399

class UbxGnss;   // forward declaration for friendship

// --- common freshness/age bookkeeping shared by every value holder ---------
struct GpsField {
  friend class UbxGnss;
public:
  bool     isValid()   const { return valid; }
  bool     isUpdated() const { return updated; }
  uint32_t age()       const { return valid ? millis() - lastCommitTime : (uint32_t)ULONG_MAX; }
protected:
  bool     valid = false, updated = false;
  uint32_t lastCommitTime = 0;
  void     touch() { lastCommitTime = millis(); valid = updated = true; }
};

// --- UTC time of day, stored as HHMMSScc to match the old value() layout ---
struct GpsTime : GpsField {
  friend class UbxGnss;
public:
  uint32_t value()       { updated = false; return t; }                 // HHMMSScc
  uint8_t  hour()        { updated = false; return  t / 1000000UL; }
  uint8_t  minute()      { updated = false; return (t / 10000UL) % 100; }
  uint8_t  second()      { updated = false; return (t / 100UL) % 100; }
  uint8_t  centisecond() { updated = false; return  t % 100; }
private:
  uint32_t t = 0;
  void set(uint8_t h, uint8_t m, uint8_t s) {
    t = (uint32_t)h * 1000000UL + (uint32_t)m * 10000UL + (uint32_t)s * 100UL;
    touch();
  }
};

// --- geodetic position (decimal degrees) -----------------------------------
struct GpsLocation : GpsField {
  friend class UbxGnss;
public:
  double lat() { updated = false; return latDeg; }
  double lng() { updated = false; return lngDeg; }
private:
  double latDeg = 0, lngDeg = 0;
  void set(int32_t lat1e7, int32_t lng1e7) {           // UBX lat/lon are 1e-7 deg
    latDeg = lat1e7 / 1.0e7;
    lngDeg = lng1e7 / 1.0e7;
    touch();
  }
};

// --- altitude above mean sea level -----------------------------------------
struct GpsAltitude : GpsField {
  friend class UbxGnss;
public:
  double meters() { updated = false; return mm / 1000.0; }
  double feet()   { updated = false; return _GPS_FEET_PER_METER * mm / 1000.0; }
private:
  int32_t mm = 0;
  void setMillimeters(int32_t v) { mm = v; touch(); }
};

// --- ground speed ----------------------------------------------------------
struct GpsSpeed : GpsField {
  friend class UbxGnss;
public:
  double mps()  { updated = false; return mmps / 1000.0; }
  double kmph() { updated = false; return mmps * 3.6 / 1000.0; }
private:
  int32_t mmps = 0;                                    // mm/s
  void setMillimetersPerSec(int32_t v) { mmps = v; touch(); }
};

// --- satellites used in the solution (can exceed the old NMEA cap of 12) ----
struct GpsSats : GpsField {
  friend class UbxGnss;
public:
  uint32_t value() { updated = false; return n; }
private:
  uint8_t n = 0;
  void set(uint8_t v) { n = v; touch(); }
};

// --- dilution of precision (carries pDOP, 0.01 units on the wire) -----------
struct GpsDop : GpsField {
  friend class UbxGnss;
public:
  double hdop() { updated = false; return raw / 100.0; }   // name kept; value is pDOP
private:
  uint16_t raw = 0;
  void setRaw(uint16_t v) { raw = v; touch(); }
};

// ---------------------------------------------------------------------------
class UbxGnss {
public:
  UbxGnss();

  // Feed one received byte. Returns true when a fresh position solution
  // (NAV-PVT on M10, NAV-POSLLH on M6) was just committed.
  bool encode(uint8_t b);

  // Abandon any partially-received frame and hunt for the next sync. Call after
  // flushing the UART (e.g. after a long blocking TX) so the next frame parses cleanly.
  void resetParse() { st = S_SYNC1; }

  // Value holders (names/methods kept from the old API)
  GpsTime     time;
  GpsLocation location;
  GpsAltitude altitude;
  GpsSpeed    speed;
  GpsSats     satellites;
  GpsDop      hdop;                 // holds pDOP

  // Sub-second fraction of the UTC time-of-day (nanoseconds, signed) from NAV-PVT /
  // NAV-TIMEUTC. Added to the whole-second time for sub-second clock synchronisation.
  int32_t  timeNano         = 0;

  // Extra solution data now available from UBX
  float    verticalVelocity = 0.0f; // m/s, positive = climbing (from -velD)
  float    headingDeg       = 0.0f; // motion heading
  uint8_t  fixType          = 0;    // UBX fixType / gpsFix (3 = 3D)
  bool     gnssFixOK        = false;// receiver flags the fix as usable
  uint32_t hAccMm           = 0;    // horizontal accuracy estimate (mm)
  uint32_t vAccMm           = 0;    // vertical accuracy estimate (mm)
  uint32_t sAccMmS          = 0;    // speed accuracy estimate (mm/s)

  // Hardware interference: raw CW jamming indicator (UBX-MON-RF / MON-HW)
  uint8_t  jamIndicator     = 0;    // 0..255 CW jamming indicator

  // Spoofing detection (UBX-NAV-STATUS flags2)
  uint8_t  spoofState       = 0;    // 0 unknown, 1 none, 2 spoofing indicated, 3 multiple

  // Power-save mode state, from UBX-NAV-PVT flags (M10). Lets us see whether cyclic tracking
  // (PSMCT) actually engaged: 0 = PSM not active (continuous), 1 = enabled (intermediate),
  // 2 = acquisition, 3 = tracking, 4 = power optimized tracking (saving power), 5 = inactive.
  uint8_t  psmState         = 0;

  // Set true whenever a position solution is committed; caller clears it.
  bool     navUpdated       = false;

  // Per-constellation count of satellites used in the fix (from UBX-NAV-SAT)
  uint8_t satGps = 0, satGal = 0, satBds = 0, satGlo = 0, satSbas = 0, satQzss = 0;
  bool    navSatUpdated = false;    // set when a NAV-SAT frame committed; caller clears

  // Major-GNSS support/enable bitmasks from UBX-MON-GNSS (bit0 GPS, bit1 GLONASS,
  // bit2 BeiDou, bit3 Galileo). Lets us log what the receiver actually supports and has
  // enabled after configuration. (Augmentation systems SBAS/QZSS are not reported here.)
  uint8_t  gnssSupported    = 0;
  uint8_t  gnssEnabled      = 0;
  bool     monGnssSeen      = false;// a UBX-MON-GNSS reply has been decoded

  // Parser diagnostics
  uint16_t frameErrors = 0;         // UBX frames that failed the checksum

  // UBX message class/id constants (also used by the TX/config side)
  static const uint8_t CLS_NAV = 0x01, CLS_MON = 0x0A, CLS_CFG = 0x06;
  static const uint8_t NAV_POSLLH = 0x02, NAV_STATUS = 0x03, NAV_SOL = 0x06,
                       NAV_PVT = 0x07, NAV_VELNED = 0x12, NAV_TIMEUTC = 0x21, NAV_SAT = 0x35;
  static const uint8_t MON_HW = 0x09, MON_RF = 0x38, MON_GNSS = 0x28;

private:
  // UBX frame state machine
  enum { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CKA, S_CKB };
  uint8_t  st = S_SYNC1;
  uint8_t  msgClass = 0, msgId = 0;
  uint16_t payloadLen = 0, payloadIdx = 0;
  uint8_t  payload[100];            // large enough for NAV-PVT (92) and MON-HW (68)
  uint8_t  ckA = 0, ckB = 0, rxCkA = 0;
  bool     dispatch();             // decode a checksum-valid frame

  // NAV-SAT is far larger than the payload buffer (8 + 12*numSvs bytes), so it is
  // decoded on the fly: each 12-byte satellite block is assembled here and its
  // constellation counted, without ever storing the whole message.
  uint8_t  nsBlk[12];
  uint8_t  nsGps = 0, nsGal = 0, nsBds = 0, nsGlo = 0, nsSbas = 0, nsQzss = 0;
  void     navSatByte(uint8_t b);  // feed one NAV-SAT payload byte
};

#endif
