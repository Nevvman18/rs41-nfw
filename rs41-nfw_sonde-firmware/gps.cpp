// ------------------------------------------------------------------------
// [RS41-NFW-SA] Source-Available Module - this ENTIRE file is NOT under GPL-3.0.
// It is covered by the RS41-NFW Source-Available License. See LICENSING.md and
// LICENSE.source-available.
//
// RS41-NFW native u-blox UBX GNSS driver - parser implementation.
// See gps.h for the message set and the rationale.
// ------------------------------------------------------------------------

#include "gps.h"

// little-endian field readers over the payload buffer
static inline uint16_t rdU2(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t rdI2(const uint8_t *p) { return (int16_t)rdU2(p); }
static inline uint32_t rdU4(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t rdI4(const uint8_t *p) { return (int32_t)rdU4(p); }

UbxGnss::UbxGnss() {
  payload[0] = 0;
}

// Feed one byte through the UBX frame state machine. Verifies the 8-bit
// Fletcher checksum and, on a good frame, decodes it. Returns true only when
// a fresh position solution was committed (so the read loop can stop early).
bool UbxGnss::encode(uint8_t b) {
  switch (st) {
    case S_SYNC1:
      if (b == 0xB5) st = S_SYNC2;
      break;

    case S_SYNC2:
      st = (b == 0x62) ? S_CLASS : (b == 0xB5 ? S_SYNC2 : S_SYNC1);
      break;

    case S_CLASS:
      msgClass = b; ckA = b; ckB = b; st = S_ID;          // checksum starts at class
      break;

    case S_ID:
      msgId = b; ckA += b; ckB += ckA; st = S_LEN1;
      break;

    case S_LEN1:
      payloadLen = b; ckA += b; ckB += ckA; st = S_LEN2;
      break;

    case S_LEN2: {
      payloadLen |= (uint16_t)b << 8; ckA += b; ckB += ckA;
      payloadIdx = 0;
      bool isNavSat = (msgClass == CLS_NAV && msgId == NAV_SAT);
      if (isNavSat) { nsGps = nsGal = nsBds = nsGlo = nsSbas = nsQzss = 0; }  // reset counts
      // Robustness: a corrupted length (e.g. after a UART RX overrun during a long
      // radio TX) must not make us sit and swallow thousands of "payload" bytes -
      // that stalls the parser and freezes every reading. NAV-SAT is streamed (not
      // buffered) and can be large: 8 + 12*numSvs bytes, which with several constellations
      // enabled (GPS + Galileo + BeiDou + GLONASS + SBAS) runs to ~700+ bytes, so it needs
      // a generous cap. Everything else must fit the payload buffer. A length past the cap
      // is corruption - bail to the sync hunt.
      if (payloadLen > (isNavSat ? 1024 : sizeof(payload))) { st = S_SYNC1; break; }
      st = (payloadLen == 0) ? S_CKA : S_PAYLOAD;
      break;
    }

    case S_PAYLOAD:
      ckA += b; ckB += ckA;
      if (msgClass == CLS_NAV && msgId == NAV_SAT) navSatByte(b);   // stream-decode, do not buffer
      else if (payloadIdx < sizeof(payload))       payload[payloadIdx] = b;  // keep what fits
      if (++payloadIdx >= payloadLen) st = S_CKA;
      break;

    case S_CKA:
      rxCkA = b; st = S_CKB;
      break;

    case S_CKB: {
      bool navPos = false;
      if (rxCkA == ckA && b == ckB) {
        if (msgClass == CLS_NAV && msgId == NAV_SAT) {
          satGps = nsGps; satGal = nsGal; satBds = nsBds;          // commit streamed counts
          satGlo = nsGlo; satSbas = nsSbas; satQzss = nsQzss;
          navSatUpdated = true;
        } else if (payloadLen <= sizeof(payload)) {
          navPos = dispatch();
        }
      } else {
        if (frameErrors < 65535) frameErrors++;                    // checksum failure
      }
      st = S_SYNC1;
      return navPos;
    }
  }
  return false;
}

// Decode a checksum-valid frame into the value holders.
// Returns true if a position solution was committed.
bool UbxGnss::dispatch() {
  const uint8_t *p = payload;

  if (msgClass == CLS_NAV) {
    switch (msgId) {

      // ---- u-blox M8/M9/M10: everything in one 92-byte message ----------
      case NAV_PVT: {
        if (payloadLen < 92) return false;
        uint8_t validFlags = p[11];
        fixType   = p[20];
        gnssFixOK = (p[21] & 0x01);
        psmState  = (p[21] >> 2) & 0x07;       // flags bits 4..2 = power-save mode state
        satellites.set(p[23]);                 // numSV (used-in-fix), can be > 12
        uint16_t pdop = rdU2(p + 76);
        hdop.setRaw(pdop);

        if ((validFlags & 0x02)) {             // validTime
          time.set(p[8], p[9], p[10]);
          timeNano = rdI4(p + 16);             // sub-second fraction (ns, signed)
        }

        if (gnssFixOK && fixType >= 3) {        // 3D fix -> trust position/velocity
          location.set(rdI4(p + 28), rdI4(p + 24));   // lat, lon (1e-7 deg)
          altitude.setMillimeters(rdI4(p + 36));      // hMSL (mm)
          speed.setMillimetersPerSec(rdI4(p + 60));   // gSpeed (mm/s)
          verticalVelocity = -rdI4(p + 56) / 1000.0f; // velD (mm/s) -> +up m/s
          headingDeg       = rdI4(p + 64) / 1.0e5f;   // headMot (1e-5 deg)
          hAccMm  = rdU4(p + 40);
          vAccMm  = rdU4(p + 44);
          sAccMmS = rdU4(p + 68);
        }
        navUpdated = true;
        return true;
      }

      // ---- u-blox 6 legacy set -----------------------------------------
      case NAV_POSLLH: {                        // position + altitude (28 bytes)
        if (payloadLen < 28) return false;
        if (gnssFixOK) {                         // gated by the last NAV-SOL
          location.set(rdI4(p + 8), rdI4(p + 4));     // lat, lon (1e-7 deg)
          altitude.setMillimeters(rdI4(p + 16));      // hMSL (mm)
          hAccMm = rdU4(p + 20);
          vAccMm = rdU4(p + 24);
          navUpdated = true;
          return true;                           // this is the M6 position anchor
        }
        return false;
      }

      case NAV_VELNED: {                         // velocity (36 bytes, cm/s)
        if (payloadLen < 36) return false;
        if (gnssFixOK) {
          verticalVelocity = -rdI4(p + 12) / 100.0f;  // velD (cm/s) -> +up m/s
          speed.setMillimetersPerSec(rdU4(p + 20) * 10); // gSpeed (cm/s) -> mm/s
          headingDeg = rdI4(p + 24) / 1.0e5f;          // heading (1e-5 deg)
          sAccMmS    = rdU4(p + 28) * 10;              // sAcc (cm/s) -> mm/s
        }
        return false;
      }

      case NAV_SOL: {                            // fix status, numSV, pDOP (52 bytes)
        if (payloadLen < 52) return false;
        fixType   = p[10];                        // gpsFix
        gnssFixOK = (p[11] & 0x01);
        hdop.setRaw(rdU2(p + 44));                // pDOP
        satellites.set(p[47]);                    // numSV
        return false;
      }

      case NAV_TIMEUTC: {                        // UTC time of day (20 bytes)
        if (payloadLen < 20) return false;
        // Accept on validTOW (bit0) or validUTC (bit2): some u-blox 6 units set
        // validUTC late, which left the clock reading as unset. validTOW is set as
        // soon as there is a time solution and is good to ~1 s (refined once UTC
        // fully resolves), which is fine for the scheduler and display.
        if (p[19] & 0x05) {
          time.set(p[16], p[17], p[18]);
          timeNano = rdI4(p + 8);               // sub-second fraction (ns, signed)
        }
        return false;
      }

      case NAV_STATUS: {                         // fix + spoofing state (16 bytes)
        if (payloadLen < 16) return false;
        spoofState = (p[7] >> 3) & 0x03;          // flags2 bits 3-4 = spoofDetState
        return false;
      }
    }
    return false;
  }

  if (msgClass == CLS_MON) {
    // Both MON-RF (M10) and MON-HW (u-blox 6) carry the raw 0..255 CW jamming
    // indicator - that is the only interference value we report.
    if (msgId == MON_RF) {                        // M10: one 24-byte block per RF path
      if (payloadLen >= 4 + 24) jamIndicator = p[4 + 16];
      return false;
    }
    if (msgId == MON_HW) {                         // u-blox 6: 68-byte block
      if (payloadLen >= 68) jamIndicator = p[45];
      return false;
    }
    if (msgId == MON_GNSS) {                        // M10: major-GNSS support/enable masks
      if (payloadLen >= 4) { gnssSupported = p[1]; gnssEnabled = p[3]; monGnssSeen = true; }
      return false;
    }
  }

  return false;
}

// Stream one NAV-SAT payload byte. Header is 8 bytes; then 12-byte satellite
// blocks. gnssId is block byte 0, and the svUsed flag is bit 3 of the 4-byte
// flags at block bytes 8..11. We never store the whole (large) message.
void UbxGnss::navSatByte(uint8_t b) {
  if (payloadIdx < 8) return;                     // iTOW, version, numSvs, reserved
  uint8_t inBlk = (uint8_t)((payloadIdx - 8) % 12);
  nsBlk[inBlk] = b;
  if (inBlk == 11) {                              // block complete
    uint8_t qual   = nsBlk[8] & 0x07;             // flags bits0-2 = qualityInd (>=4 locked)
    bool    svUsed = nsBlk[8] & 0x08;             // flags bit3    = used in the fix
    switch (nsBlk[0]) {                            // gnssId
      case 0: if (svUsed && nsGps  < 255) nsGps++;  break; // GPS
      // SBAS is a correction source, not a ranging source, so u-blox almost never sets
      // its svUsed bit - counting "used in fix" left this permanently 0. Count SBAS that
      // are actually locked (qualityInd >= 4) instead, so the tile shows SBAS reception.
      case 1: if (qual >= 4 && nsSbas < 255) nsSbas++; break; // SBAS
      case 2: if (svUsed && nsGal  < 255) nsGal++;  break; // Galileo
      case 3: if (svUsed && nsBds  < 255) nsBds++;  break; // BeiDou
      case 5: if (svUsed && nsQzss < 255) nsQzss++; break; // QZSS
      case 6: if (svUsed && nsGlo  < 255) nsGlo++;  break; // GLONASS
    }
  }
}
