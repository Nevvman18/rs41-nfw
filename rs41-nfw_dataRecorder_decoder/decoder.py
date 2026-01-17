import sys

# ANSI escape codes for coloring text
GREEN = '\033[32m'
RED = '\033[31m'
YELLOW = '\033[33m'
ORANGE = '\033[38;5;214m'  # ANSI escape code for orange
RESET = '\033[0m'

def decode_nfw_packet(packet):
    # Define the expected structure and number of fields, adding GPS signal anomalies and GPS HDOP
    expected_fields = [
        "maxAlt (m)",
        "maxSpeed (km/h)",
        "maxAscentRate (m/s)",
        "maxDescentRate (m/s)",
        "maxMainTemperature (\u00b0C)",
        "minMainTemperature (\u00b0C)",
        "maxInternalTemperature (\u00b0C)",
        "minInternalTemperature (\u00b0C)",
        "ledsEnable (1 - yes, 0 - no)",
        "status (0 - ok, 1 - warn, 2 - err)",
        "gpsResetCounter",
        "beganFlying (1 - yes, 0 - no)",
        "burstDetected (1 - yes, 0 - no)",
        "radioPwrSetting (0 to 7, power levels of SI4032 in dBm and mW)",
        "currentGPSPowerMode (0 - disabled, 1 - max performance, 2 - powersaving)",
        "radioTemp (\u00b0C)",
        "sensorBoomErr (0 - Clear, 1 - ERR)",
        "zeroHumidityCapacitance (uF)",  # New field
        "humidityCapacitanceRangeDelta (uF)",     # New field
        "heatingPwmStatus",            # New field
        "referenceHeaterStatus",       # New field
        "mvBatU (mV)",                 # New field
        "referenceAreaTemperature (\u00b0C)",  # New field
        "gpsSignalAnomalies (1 - true, 0 - false)",  # New field for GPS signal anomalies
        "gpsHDOP"  # New field for GPS HDOP
    ]

    # Remove the last semicolon if present
    if packet.endswith(';'):
        packet = packet[:-1]

    # Split the packet into parts
    parts = packet.split("NFW;")
    if len(parts) < 2:
        print("Error: No NFW structure detected.")
        return

    # Extract the NFW part and split it by semicolons
    nfw_part = parts[1].strip()
    fields = nfw_part.split(";")

    # Check for the exact number of fields
    if len(fields) != len(expected_fields):
        print(f"Error: Expected {len(expected_fields)} values in NFW packet but got {len(fields)}.")
        return

    # Decode the fields into a dictionary
    decoded = {expected_fields[i]: fields[i] for i in range(len(expected_fields))}

    # Print the decoded meaning with color formatting
    print("Decoded NFW Packet:")
    for field, value in decoded.items():
        if "ledsEnable" in field:
            value_color = GREEN if value == "1" else RED
            value_text = "YES" if value == "1" else "NO"
            print(f"  {field.split(' ')[0]}: {value_color}{value_text} ({value}){RESET}")
        elif "status" in field:
            if value == "0":
                value_text = "OK"
                value_color = GREEN
            elif value == "1":
                value_text = "Warning"
                value_color = YELLOW
            else:
                value_text = "Error"
                value_color = RED
            print(f"  {field.split(' ')[0]}: {value_color}{value_text} ({value}){RESET}")
        elif "beganFlying" in field:
            value_color = GREEN if value == "1" else RED
            value_text = "YES" if value == "1" else "NO"
            print(f"  {field.split(' ')[0]}: {value_color}{value_text} ({value}){RESET}")
        elif "burstDetected" in field:
            value_color = GREEN if value == "1" else RED
            value_text = "YES" if value == "1" else "NO"
            print(f"  {field.split(' ')[0]}: {value_color}{value_text} ({value}){RESET}")
        elif "currentGPSPowerMode" in field:
            if value == "0":
                value_text = "Disabled"
                value_color = RED
            elif value == "1":
                value_text = "Max Performance"
                value_color = GREEN
            else:
                value_text = "Power Saving"
                value_color = YELLOW
            print(f"  {field.split(' ')[0]}: {value_color}{value_text} ({value}){RESET}")
        elif "radioPwrSetting" in field:
            # Power levels for SI4032 chip in dBm and mW
            power_levels_dBm_mW = {
                "0": "-1 dBm (~0.8 mW)",
                "1": "2 dBm (~1.6 mW)",
                "2": "5 dBm (~3 mW)",
                "3": "8 dBm (~6 mW)",
                "4": "11 dBm (~12 mW)",
                "5": "14 dBm (~25 mW)",
                "6": "17 dBm (~50 mW)",
                "7": "20 dBm (~100 mW)"
            }
            value_color = GREEN  # Here we just use yellow for power levels
            value_text = power_levels_dBm_mW.get(value, "Unknown Power Level")
            print(f"  {field.split(' ')[0]}: {value_color}{value_text} ({value}){RESET}")
        elif "Temperature" in field or "radioTemp" in field:
            # For temperature fields, append °C
            print(f"  {field.split(' ')[0]}: {value}°C")
        elif "maxSpeed" in field:
            # For speed append kph
            print(f"  {field.split(' ')[0]}: {value} km/h")
        elif "maxAscentRate" in field or "maxDescentRate" in field:
            # For speed and rates, append m/s
            print(f"  {field.split(' ')[0]}: {value} m/s")
        elif "maxAlt" in field:
            # For maxAlt, append m (meters)
            print(f"  {field.split(' ')[0]}: {value} m")
        elif "sensorBoomErr" in field:
            # Handle the sensorBoomErr field
            if value == "1":
                value_color = RED
                value_text = "ERR"
            else:
                value_color = GREEN
                value_text = "Clear"
            print(f"  {field.split(' ')[0]}: {value_color}{value_text} ({value}){RESET}")
        elif "zeroHumidityFrequency" in field:
            # Handle the zeroHumidityFrequency field (red if 0)
            value_color = RED if value == "0" else RESET
            print(f"  {field.split(' ')[0]}: {value_color}{value} Hz{RESET}")
        elif "humidityRangeDelta" in field:
            # Handle the humidityRangeDelta field (in Hz)
            print(f"  {field.split(' ')[0]}: {value} Hz")
        elif "heatingPwmStatus" in field:
            # Decode heating PWM status with color
            pwm_value = int(value)
            if 0 < pwm_value <= 255:
                print(f"  Humidity heating power: {ORANGE}L: ON {pwm_value}/255{RESET} | {GREEN}H: OFF{RESET}")
            elif 256 <= pwm_value <= 500:
                high_power = pwm_value - 255
                print(f"  Humidity heating power: {ORANGE}L: ON 255/255{RESET} | {RED}H: ON {high_power}/245{RESET}")
            else:
                print(f"  Humidity heating power: {GREEN}L: OFF | H: OFF{RESET}")
        elif "referenceHeaterStatus" in field:
            # Decode reference heater status with color
            heater_status_map = {
                "0": (GREEN, "OFF"),
                "1": (YELLOW, "Low Power"),
                "2": (ORANGE, "Medium Power"),
                "3": (RED, "High Power")
            }
            color, status = heater_status_map.get(value, (RESET, "Unknown"))
            print(f"  {field.split(' ')[0]}: {color}{status} ({value}){RESET}")
        elif "mvBatU" in field:
            # Display battery voltage in mV
            print(f"  {field.split(' ')[0]}: {value} mV")
        elif "referenceAreaTemperature" in field:
            # Handle the reference area temperature
            print(f"  {field.split(' ')[0]}: {value}°C")
        elif "gpsSignalAnomalies" in field:
            # Handle GPS signal anomalies
            value_color = RED if value == "1" else GREEN
            value_text = "Anomalies Detected" if value == "1" else "No Anomalies"
            print(f"  {field.split(' ')[0]}: {value_color}{value_text} ({value}){RESET}")
        elif "gpsHDOP" in field:
            # Handle GPS HDOP
            print(f"  {field.split(' ')[0]}: {value}")
        else:
            print(f"  {field.split(' ')[0]}: {value}")

if __name__ == "__main__":
    # Read input from the terminal
    print("Enter the dataRecorder packet (example: 'NFW;28651;20;-2;0;18;0;28;23;1;0;0;0;0;0;1;24;0;39631;950;16;0;2524;21;0;4'): ")
    packet = input().strip()

    # Call the decoder
    decode_nfw_packet(packet)

