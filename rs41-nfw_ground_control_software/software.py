# RS41-NFW Ground Control Software. Backend, version v12
# Released on GPL-3.0 license. Authors: Franek Łada (nevvman, SP5FRA)

import serial
import threading
import time
import sys
from flask import Flask, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
# Added async_mode for better stability on Windows
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Initialize ALL variables to default values to prevent UI errors on startup
telemetry = {
    # System
    "isRSM4x2": "1", "isRSM4x4": "0", "fwVer": "NFW, wait...", "millis": "0",
    "sys_h": "0", "sys_m": "0", "sys_s": "0", "autoResetEnable": "1",
    "buttonMode": "1", "ledStatusEnable": "1", "ledAutoDisableHeight": "1000",
    
    # GPS
    "gpsBaudRate": "9600", "ubloxGpsAirborneMode": "1", "gpsTimeoutWatchdog": "1800000",
    "improvedGpsPerformance": "1", "disableGpsImprovementInFlight": "1", "gpsOperationMode": "1",
    "gpsLat": "0.000000", "gpsLong": "0.000000", "gpsAlt": "0.00", "gpsSats": "0",
    "gpsHours": "0", "gpsMinutes": "0", "gpsSeconds": "0",
    "gpsSpeedKph": "0.00", "gpsHdop": "99.99", "vVCalc": "0.00",
    "currentGPSPowerMode": "1", "gpsTimerBegin": "0", "gpsResetCounter": "0", "gpsJamWarning": "0",
    
    # Radio General
    "radioEnablePA": "1", "callsign": "N0CALL", "radioTemp": "0.00", "currentRadioPwrSetting": "0",
    
    # PIP
    "pipEnable": "0", "pipFrequencyMhz": "432.500", "pipTimeSyncSeconds": "0",
    "pipTimeSyncOffsetSeconds": "0", "pipRadioPower": "6",
    
    # Horus
    "horusEnable": "1", "horusFrequencyMhz": "437.600", "horusTimeSyncSeconds": "15",
    "horusTimeSyncOffsetSeconds": "0", "horusPayloadId": "737", "horusRadioPower": "0",
    
    # Horus
    "horusV3Enable": "1", "horusV3FrequencyMhz": "437.600", "horusV3TimeSyncSeconds": "15",
    "horusV3TimeSyncOffsetSeconds": "0", "horus_v3_callsign": "737", "horusV3RadioPower": "0",
    
    # APRS
    "aprsEnable": "1", "aprsFrequencyMhz": "432.500", "aprsTimeSyncSeconds": "40",
    "aprsTimeSyncOffsetSeconds": "0", "aprsCall": "N0CALL", "aprsComment": "",
    "aprsSsid": "11", "aprsOperationMode": "1", "aprsRadioPower": "0", "aprsToneCalibrationMode": "0",
    
    # RTTY & Morse
    "rttyEnable": "0", "rttyFrequencyMhz": "434.600", "rttyTimeSyncSeconds": "0",
    "rttyTimeSyncOffsetSeconds": "0", "rttyRadioPower": "7",
    "morseEnable": "0", "morseFrequencyMhz": "434.600", "morseTimeSyncSeconds": "0",
    "morseTimeSyncOffsetSeconds": "0", "morseRadioPower": "7",
    
    # Power
    "batV": "0.00", "vBatWarnValue": "0.00", "batteryCutOffVoltage": "0.00", "ultraPowerSaveAfterLanding": "0",
    
    # Flight Stats
    "maxAlt": "0", "maxSpeed": "0", "maxAscentRate": "0", "maxDescentRate": "0",
    "maxMainTemperature": "0", "minMainTemperature": "0", "maxInternalTemp": "0", "minInternalTemp": "0",
    "beganFlying": "0", "burstDetected": "0", "hasLanded": "0",
    "flightDetectionAltitude": "800", "burstDetectionThreshold": "500", "lowAltitudeFastTxThreshold": "1000",
    
    # Sensors - Main Temperature
    "mainTemperatureValue": "0.00", "mainTemperatureCorrectionC": "0", "mainTemperatureFrequency": "0",
    "mainTemperaturePeriod": "0", "mainTemperatureResistance": "0", "tempSensorBoomCalibrationFactor": "0",
    
    # Sensors - Humidity
    "humidityValue": "0", "humidityFrequency": "0", "maxHumidityFrequency": "0",
    "zeroHumidityCalibration": "1", "humidityRangeDelta": "1100", "zeroHumidityFrequency": "0",
    
    # Sensors - Pressure
    "pressureValue": "0", "enablePressureEstimation": "0", "seaLevelPressure": "101325", 
    
    # Sensors - Internal Temps
    "thermistorTemp": "0.00", "THERMISTOR_R25": "10400", "THERMISTOR_B": "4100",
    
    # Sensors - External Heater Temp
    "extHeaterTemperatureValue": "0.00", "extHeaterTemperatureFrequency": "0",
    "extHeaterTemperaturePeriod": "0", "extHeaterTemperatureResistance": "0", 
    "extHeaterTemperatureCorrectionC": "0",
    
    # Sensors - Configuration
    "sensorBoomEnable": "1", "humidityModuleEnable": "1",
    "autoTemperatureCalibration": "1", "autoTemperatureCalibrationMethod": "1",
    "environmentStartupAirTemperature": "24.00", "autoHumidityModuleTemperatureCorrection": "1",
    "calibrationError": "1",
    
    # Sensor Errors
    "sensorBoomMainTempError": "0", "sensorBoomHumidityModuleError": "0",
    
    # Heating
    "referenceHeaterStatus": "0", "referenceAreaTargetTemperature": "32",
    "extHeaterPwmStatus": "0", "humidityModuleHeating": "0", 
    "humidityModuleHeatingAmount": "4", "heatingTemperatureThreshold": "2", "heatingHumidityThreshold": "90",
    
    # Recorder
    "dataRecorderEnable": "1", "dataRecorderInterval": "600000",
    "dataRecorderFlightNoiseFiltering": "1", "recorderInitialized": "0",
    
    # Status
    "stage": "01", "ok": "1", "warn": "0", "err": "0"
}

ser = None
stop_thread = threading.Event()

STAGE_MAP = {
    "01": ("Initial hardware setup", "Initializing hardware..."),
    "02": ("GPS initialization", "Enabling and initializing the onboard u-blox GPS..."),
    "03": ("Sensor Boom and Heaters Initialization", "Initializing sensor boom circuitry and heatres in process..."),
    "04": ("Hardware setup complete", "Hardware setup has finished"),
    "11": ("Temp Calibration (Method 1)", "Method 1 - Calibrating via constant environment temperature. Please place the sonde in the specified environment (temperature must exactly match the one specified in firmware config under 'environmentStartupAirTemperature')"),
    "12": ("Temp Calibration (Method 2)", "Estimating temperature via PCB polynomial model. Please wait..."),
    "15": ("Humidity module temperature correction", "Automatic humidity module temperature sensor correction in process..."),
    "20": ("Reconditioning Phase", "Reconditining phase is in progress now! The glass-ceramic humidity module located on the sensor boom will now get very hot for a short period of time (~135*C!), to remove any contaminants. Don't touch it or place any objects close nearby, both to not get burned and to not leave any contaminants."),
    "21": ("Zero Humidity Check", "Zero humidity check is in progress now! The glass-ceramic humidity module located on the sensor boom will now get very hot for a short while (~135*C)! Don't touch it or place any objects close nearby. The sonde will take a few measurements and in under a minute finish the calibration process. Make sure the environment is stable, with little to no wind, and in a suitable temperature and humidity range (0-30*C, 0-60%RH). After the calibration, please do not touch the sensor, it may still be hot and you could leave contaminants on it."),
    "25": ("Humidity Delta Calibration: Starting...", "Entering high-precision humidity calibration, please wait..."),
    "26": ("Humidity Delta Calibration: Ready", "humidityRangeDelta calibration process has started. For the successful calibration, please switch now to the 'Sensors' tab in this software and observe the 'humidityRangeDelta' value. While looking at it, place the sensor boom in a simulated 100%RH environment and observe the 'humidityRangeDelta' readings. Look around for the (by average) highest number, write it down, add +25 to it and power off the sonde. Recompile the firmware with this mode disabled and manually entered 'humidityRangeDelta' value for your sensor boom readings."),
    "27": ("Humidity Delta Calibration: Complete, power OFF", "Sonde turning OFF. Please reprogram with debug mode disabled and manually entered 'humidityRangeDelta' value for your sensor boom readings."),
    "31": ("GPS Performance Improvement - waiting for fix", "Radio silence, transmissions halted. The 'Improved GPS Performance' mode is working on quickly gathering a stable fix. Radio transmitter is quiet while there is no fix. Place the sonde outside, with a clear, unobstructed view of the sky.  Searching for GPS satellites..."),
    "32": ("GPS Performance Improvement - GPS fix acquired", "Sonde GPS has a stable position fix, will exit soon and begin transmitting..."),
    "50": ("Sonde ready, waiting for fix...", "Sonde ready, place the sonde outside with a clear, unobstructed view of sky and wait for GPS fix...  If this process takes longer than expected, consider enabling 'improvedGpsPerformance'."),
    "59": ("READY FOR FLIGHT!", "Check for green LED and OK status. While connected, you can observe all parameters here.   The sonde is ready and waiting to be launched...")
}

POWER_MAP = {
    "0": "-1 dBm (~0.8 mW)", "1": "2 dBm (~1.6 mW)", "2": "5 dBm (~3 mW)",
    "3": "8 dBm (~6 mW)", "4": "11 dBm (~12 mW)", "5": "14 dBm (~25 mW)",
    "6": "17 dBm (~50 mW)", "7": "20 dBm (~100 mW)"
}


def serial_worker(port, baud):
    global telemetry, ser
    print(f"DEBUG: Attempting to connect to {port} at {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"DEBUG: Serial connection established on {port}")
        while not stop_thread.is_set():
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if ":" in line:
                    key, val = line.split(":", 1)
                    telemetry[key.strip()] = val.strip()
                    socketio.emit('telemetry_update', telemetry)
            time.sleep(0.01)
    except Exception as e:
        print(f"DEBUG ERROR: Serial failed: {e}")
        socketio.emit('serial_error', str(e))

@app.route('/')
def index():
    return render_template('index.html', STAGE_MAP=STAGE_MAP, POWER_MAP=POWER_MAP)

@socketio.on('connect_serial')
def handle_connect(data):
    global stop_thread
    print(f"DEBUG: UI Requesting connection to {data['port']}")
    stop_thread.set()
    time.sleep(0.2)
    stop_thread.clear()
    threading.Thread(target=serial_worker, args=(data['port'], data['baud']), daemon=True).start()

@socketio.on('refresh_data')
def handle_refresh():
    if ser and ser.is_open:
        for _ in range(5):
            ser.write(b'\r\n')
            time.sleep(0.05)

if __name__ == '__main__':
    # Print a clear message so you know the script is alive
    print("-----------------------------------------")
    print("RS41-NFW Ground Control Station Starting")
    print("Navigate to: http://127.0.0.1:4141")
    print("-----------------------------------------")
    
    try:
        socketio.run(app, host='0.0.0.0', port=4141, debug=True)
    except Exception as e:
        print(f"CRITICAL ERROR: {e}")
