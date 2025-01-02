"""
    Author: Deep Das
    Time: 2021-07-25 20:00:00

    This script is a part of the Autonomous Drone Development Challenge (ADDC) - 2025 by SAE India southern section.
    The task is to create a drone that can take off, fly to a target location, scan a QR code, and drop a payload if the QR code is verified.
    Additional features include scanning QR code from an image file and sending QR data to the Ground Control Station (GCS).
"""

import cv2
import numpy as np
from pyzbar.pyzbar import decode
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import argparse
import math
import os

# Option parsing ke liye command-line arguments ko set karo
parser = argparse.ArgumentParser(description='Vehicle ko control karta hai using vehicle.simple_goto aur object track karta hai.')
parser.add_argument('--connect', default='127.0.0.1:14550',
                    help="Vehicle connection string. Agar specify nahi kiya toh SITL automatically start hoga aur use kiya jayega.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Agar connection string nahi diya toh SITL start karo
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Vehicle se connect ho rahe hain aur HEARTBEAT ka wait karte hain
def connect_vehicle(connection_string):
    print(f'Connecting to vehicle on: {connection_string}')
    vehicle = connect(connection_string, wait_ready=True)

    # Ensure the vehicle is sending heartbeats
    print("Waiting for heartbeat...")
    while not vehicle.last_heartbeat:
        time.sleep(1)
    print("Heartbeat received!")

    # Increase telemetry stream rates
    print("Configuring MAVLink stream rates...")
    vehicle.parameters['SR0_EXT_STATUS'] = 10  # System status updates
    vehicle.parameters['SR0_EXTRA1'] = 10      # Extended statuses
    vehicle.parameters['SR0_EXTRA2'] = 10      # Position updates
    vehicle.parameters['SR0_EXTRA3'] = 10      # Additional data
    vehicle.flush()
    return vehicle

vehicle = connect_vehicle(connection_string)

# Drone ko arm karo aur specified altitude tak le jao
def arm_and_takeoff(vehicle, target_altitude):
    while not vehicle.is_armable:
        print("Drone ko arm hone ka wait kar rahe hain...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Drone ko arm hone ka wait kar rahe hain...")
        time.sleep(1)

    print("Takeoff kar rahe hain!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Target altitude reach ho gaya")
            break
        time.sleep(1)

# Haversine formula distance calculate karne ke liye
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371  # Earth radius kilometers mein
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c * 1000  # Meters mein convert kar rahe hain

# QR code data ko GCS (Ground Control Station) ko bhejna
def send_qr_to_gcs(vehicle, qr_data):
    try:
        # Truncate or adjust QR data length to meet MAVLink message requirements
        truncated_data = qr_data[:50] if len(qr_data) > 50 else qr_data
        message = vehicle.message_factory.statustext_encode(
            mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity level
            truncated_data.encode('utf-8')     # QR data
        )
        vehicle.send_mavlink(message)
        vehicle.flush()
        print(f"GCS message sent: {truncated_data}")
    except Exception as e:
        print(f"Error sending message to GCS: {e}")

# QR code ko image file se scan karna
def scan_qr_from_image_file(image_path):
    if not os.path.exists(image_path):
        print(f"Error: File {image_path} exist nahi karta.")
        return None

    image = cv2.imread(image_path)
    decoded_objects = decode(image)
    for obj in decoded_objects:
        qr_data = obj.data.decode('utf-8')
        print(f"QR Code Image se detect kiya gaya: {qr_data}")
        return str(qr_data)

    print("QR code nahi mila image mein.")
    return None

# QR code ko live webcam se scan karna
def scan_qr_code(webcam_index=0):
    cap = cv2.VideoCapture(webcam_index)
    if not cap.isOpened():
        print("Error: Webcam open nahi ho rahi.")
        return None

    print("QR code ko scan kar rahe hain...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Webcam se read nahi ho raha.")
            continue

        decoded_objects = decode(frame)
        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')
            send_qr_to_gcs(vehicle, qr_data)
            cap.release()
            cv2.destroyAllWindows()
            return str(qr_data)

        cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

# Target location par jana aur QR scan karna
def go_to_location_with_qr_scan(vehicle, latitude, longitude, altitude, initial_qr_data):
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)

    qr_verified = False
    while True:
        current_location = vehicle.location.global_relative_frame
        current_lat = current_location.lat
        current_lon = current_location.lon

        distance = haversine_distance(current_lat, current_lon, latitude, longitude)
        print(f"Target se distance: {distance:.2f} meters")

        if 0 <= distance <= 2:
            print("QR code ko scan kar rahe hain target location ke paas...")
            time.sleep(2.5)  # Hover for 2.5 seconds for more precision
            target_qr_data = scan_qr_code()
            
            # Check for any errors or discrepancies in QR code detection
            if target_qr_data != initial_qr_data:
                print("QR Code mismatch detected, retrying...")
                continue  # Retry scanning if QR doesn't match

            print(f"QR Code verified ho gaya target location par: {target_qr_data}")
            qr_verified = True
            break

        if distance < 1:
            print("Target location reach ho gaya.")
            break

        time.sleep(2)

    return qr_verified

# Servo ko control karna
def set_servo(vehicle, servo_number, pwm_value):
    pwm_value_int = int(pwm_value)
    msg = vehicle.message_factory.command_long_encode(
        0, 0, 
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_number,
        pwm_value_int,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Payload drop karne ka mechanism
def drop_payload():
    print("Payload drop kar rahe hain...")
    for _ in range(3):
        set_servo(vehicle, 6, 1100)
        time.sleep(1)
    print("Payload drop ho gaya!")
    set_servo(vehicle, 6, 1500)

# ** New Feature: Log Vehicle Path to File **
def log_vehicle_path(vehicle, log_file="vehicle_path_log.txt"):
    with open(log_file, "a") as file:
        while True:
            location = vehicle.location.global_relative_frame
            file.write(f"{time.ctime()},{location.lat},{location.lon},{location.alt}\n")
            print(f"Logged Location: {location.lat}, {location.lon}, {location.alt}")
            time.sleep(5)

# Main mission ko run karna
def main():
    try:
        arm_and_takeoff(vehicle, 10)
        print("Initial QR code scan karo:")
        initial_qr_data = scan_qr_from_image_file("/home/bodhini/Documents/QrCode/QR.png")
        if not initial_qr_data:
            print("Launch site par QR code nahi mila. Mission abort kar rahe hain.")
            return 

        target_lat = -35.36266700
        target_lon = 149.16403966
        qr_verified = go_to_location_with_qr_scan(vehicle, target_lat, target_lon, 10, initial_qr_data)

        if qr_verified:
            drop_payload()
        else:
            print("QR Code verification fail ho gayi. Payload drop nahi hoga.")

        # Start logging vehicle path
        log_vehicle_path(vehicle)

    except Exception as e:
        print(f"Ek error aayi hai: {e}")

    finally:
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")
        vehicle.close()

if __name__ == "__main__":
    main()