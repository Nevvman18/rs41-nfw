#!/usr/bin/env python
#
#   RS41 factory-calibration subframe handling for the Sounding Software.
#
#   Derived from dependencies/rs41cal.py (the upstream SondeHub subframe utility).
#   This vendored copy keeps the web app self-contained and adds:
#     - extract_firmware_cal(): maps the parsed subframe to the exact coefficient
#       names the RS41-NFW firmware expects (sensorCalibrationMode = 2 / factory*),
#       reproducing the rs1729 (zilog80) PTU conversion variables.
#     - validate_cal(): plausibility checks so we never paste garbage into a build.
#
#   The conversion the firmware performs with these values (per channel):
#     g  = (f2 - f1) / (refResistorHigh - refResistorLow)
#     Rb = (f1*refResistorHigh - f2*refResistorLow) / (f2 - f1)
#     Rc = f/g - Rb ;  R = Rc * calT
#     T  = (taylorT[0] + taylorT[1]*R + taylorT[2]*R^2 + polyT[0]) * (1 + polyT[1])
#   and the analogous TU set for the heater/RH-sensor temperature, plus the
#   empirical rs1729 RH model using calibU.
#
import base64
import math
import struct
import requests


class RS41Subframe(object):
    def __init__(self, filename=None, raw_bytes=None):
        self.data = {}
        self.raw_data = b''

        if filename:
            with open(filename, 'rb') as _f:
                self.raw_data = _f.read()
        if raw_bytes:
            self.raw_data = raw_bytes
        if (filename is None) and (raw_bytes is None):
            raise IOError("Either a filename, or raw_bytes must be provided!")
        if (len(self.raw_data) != 800) and (len(self.raw_data) != 816):
            raise IOError("Subframe data must be either 800 or 816 bytes in length!")

        self.parse_subframe()

    def parse_subframe(self):
        # Only the fields the firmware needs are extracted here (the upstream
        # script parses many more that we do not use).
        self.data['serial'] = self.extract_string(0x00D, 8).rstrip(b'\x00').decode(errors='replace')
        self.data['firmwareVersion'] = self.extract_uint32(0x015)
        # Main PTU calibration
        self.data['refResistorLow']  = self.extract_float(0x03D)
        self.data['refResistorHigh'] = self.extract_float(0x041)
        self.data['refCapLow']  = self.extract_float(0x045)
        self.data['refCapHigh'] = self.extract_float(0x049)
        self.data['taylorT']  = self.extract_float_array(0x04D, 3)
        self.data['calT']     = self.extract_float(0x059)
        self.data['polyT']    = self.extract_float_array(0x05D, 6)
        self.data['calibU']   = self.extract_float_array(0x075, 2)
        self.data['matrixU']  = self.extract_float_array(0x07D, 42)  # 7x6 RH calibration matrix
        self.data['taylorTU'] = self.extract_float_array(0x125, 3)
        self.data['calTU']    = self.extract_float(0x131)
        self.data['polyTrh']  = self.extract_float_array(0x135, 6)
        # MCU temperature-sensor voltage at 25 C (optional, repeatable ~1.424 V)
        self.data['cpuTempSensorVoltageAt25deg'] = self.extract_float(0x255)
        # Board identification (handy for the library / UI)
        self.data['variant']            = self.extract_string(0x218, 10).rstrip(b'\x00').decode(errors='replace')
        self.data['mainboard_version']  = self.extract_string(0x222, 10).rstrip(b'\x00').decode(errors='replace')
        self.data['mainboard_serial']   = self.extract_string(0x22C, 9).rstrip(b'\x00').decode(errors='replace')

    def extract_uint32(self, address):
        return struct.unpack('<I', self.raw_data[address:address + 4])[0]

    def extract_float(self, address):
        return struct.unpack('<f', self.raw_data[address:address + 4])[0]

    def extract_float_array(self, address, x):
        return list(struct.unpack('<' + 'f' * x, self.raw_data[address:address + 4 * x]))

    def extract_string(self, address, length):
        return struct.unpack('<' + str(length) + 's', self.raw_data[address:address + length])[0]


def download_subframe_data(serial, timeout=20):
    """Download radiosonde telemetry for a serial and return the first frame
    carrying subframe data. Returns (subframe_bytes, telemetry_dict) or (None, None)."""
    url = "https://api.v2.sondehub.org/sonde/" + serial
    r = requests.get(url, timeout=timeout)
    r.raise_for_status()
    data = r.json()
    if not isinstance(data, list):
        return (None, None)
    for telem in data:
        if isinstance(telem, dict) and 'rs41_subframe' in telem:
            return (base64.b64decode(telem['rs41_subframe']), telem)
    return (None, None)


def extract_firmware_cal(d):
    """Map a parsed subframe dict to the exact RS41-NFW firmware field names."""
    taylorT, polyT   = d['taylorT'],  d['polyT']
    taylorTU, polyTrh = d['taylorTU'], d['polyTrh']
    calibU = d['calibU']
    cpuV = d.get('cpuTempSensorVoltageAt25deg', 1.424)
    if not (isinstance(cpuV, (int, float)) and math.isfinite(cpuV) and 0.8 <= cpuV <= 2.2):
        cpuV = 1.424   # implausible / missing -> firmware default
    return {
        'factoryRefResistorLow':  d['refResistorLow'],
        'factoryRefResistorHigh': d['refResistorHigh'],
        'factoryRefCapLow':       d['refCapLow'],
        'factoryRefCapHigh':      d['refCapHigh'],
        'factoryTaylorT0': taylorT[0],
        'factoryTaylorT1': taylorT[1],
        'factoryTaylorT2': taylorT[2],
        'factoryCalT':     d['calT'],
        'factoryPolyT0':   polyT[0],
        'factoryPolyT1':   polyT[1],
        'factoryTaylorTU0': taylorTU[0],
        'factoryTaylorTU1': taylorTU[1],
        'factoryTaylorTU2': taylorTU[2],
        'factoryCalTU':     d['calTU'],
        'factoryPolyTrh0':  polyTrh[0],
        'factoryPolyTrh1':  polyTrh[1],
        'factoryCalibU0':   calibU[0],
        'factoryCalibU1':   calibU[1],
        'factoryMatrixU':   list(d['matrixU']),
        # Optional MCU temperature-sensor calibration (firmware variable name).
        'cpuTempSensorVoltageAt25degC': cpuV,
    }


def _finite(*vals):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


def validate_cal(d):
    """Plausibility checks on the parsed coefficients. Returns (ok, message).
    SondeHub only stores subframes that auto_rx already validated, so this guards
    against a wrong/garbled frame rather than re-deriving the CRC."""
    try:
        rrl, rrh = d['refResistorLow'], d['refResistorHigh']
        rcl, rch = d['refCapLow'], d['refCapHigh']
        if not _finite(rrl, rrh, rcl, rch, d['calT'], d['calTU']):
            return (False, "Calibration data is not finite (corrupt subframe).")
        if not _finite(*d['taylorT'], *d['taylorTU'], *d['calibU']):
            return (False, "Calibration polynomials are not finite (corrupt subframe).")
        if len(d.get('matrixU', [])) != 42 or not _finite(*d['matrixU']):
            return (False, "Humidity matrix (matrixU) missing or not finite (corrupt subframe).")
        # Reference resistors are nominally 750 Ohm / 1100 Ohm, caps 0 / 47 pF.
        if not (600.0 <= rrl <= 900.0):
            return (False, f"refResistorLow {rrl:.1f} out of expected range (~750 Ohm).")
        if not (1000.0 <= rrh <= 1300.0):
            return (False, f"refResistorHigh {rrh:.1f} out of expected range (~1100 Ohm).")
        if not (-5.0 <= rcl <= 5.0):
            return (False, f"refCapLow {rcl:.1f} out of expected range (~0 pF).")
        if not (40.0 <= rch <= 55.0):
            return (False, f"refCapHigh {rch:.1f} out of expected range (~47 pF).")
        if not (0.5 <= d['calT'] <= 2.0):
            return (False, f"calT {d['calT']:.3f} out of expected range.")
        if not (30.0 <= d['calibU'][0] <= 60.0):
            return (False, f"calibU[0] {d['calibU'][0]:.2f} out of expected range.")
        return (True, "ok")
    except (KeyError, IndexError, TypeError):
        return (False, "Subframe is missing calibration fields.")
