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

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto and tracks an object.')
parser.add_argument('--connect', default='127.0.0.1:14550',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print(f'Connecting to vehicle on: {connection_string}')
vehicle = connect(connection_string, wait_ready=True)

# Drone takeoff to target altitude
def arm_and_takeoff(vehicle, target_altitude):
    while not vehicle.is_armable:
        print("Waiting for the drone to become armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for the drone to arm...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Target altitude reached")
            break
        time.sleep(1)

# Haversine formula for distance
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371  # Earth radius in kilometers
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c * 1000  # Convert to meters

# Send QR data to GCS
def send_qr_to_gcs(vehicle, qr_data):
    message = vehicle.message_factory.statustext_encode(
        mavutil.mavlink.MAV_SEVERITY_INFO, qr_data.encode('utf-8'))
    vehicle.send_mavlink(message)
    vehicle.flush()

# Scan QR code from an image file
def scan_qr_from_image_file(image_path):
    if not os.path.exists(image_path):
        print(f"Error: File {image_path} does not exist.")
        return None

    image = cv2.imread(image_path)
    decoded_objects = decode(image)
    for obj in decoded_objects:
        qr_data = obj.data.decode('utf-8')
        print(f"QR Code Detected from Image: {qr_data}")
        return str(qr_data)

    print("No QR code found in the image.")
    return None

# Scan QR code live
def scan_qr_code(webcam_index=0):
    cap = cv2.VideoCapture(webcam_index)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return None

    print("Scanning for QR code...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read from webcam.")
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

# Go to location with QR scanning
def go_to_location_with_qr_scan(vehicle, latitude, longitude, altitude, initial_qr_data):
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)

    qr_verified = False
    while True:
        current_location = vehicle.location.global_relative_frame
        current_lat = current_location.lat
        current_lon = current_location.lon

        distance = haversine_distance(current_lat, current_lon, latitude, longitude)
        print(f"Distance to target: {distance:.2f} meters")

        if 0 <= distance <= 2:
            print("Scanning QR code within range...")
            target_qr_data = scan_qr_code()
            if target_qr_data == initial_qr_data:
                print(f"QR Code Verified at Target Location: {target_qr_data}")
                qr_verified = True
                break

        if distance < 1:
            print("Target location reached.")
            break

        time.sleep(2)

    return qr_verified

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

# Payload dropping mechanism
def drop_payload():
    print("Dropping payload...")
    for _ in range(3):
        set_servo(vehicle, 6, 1100)
        time.sleep(1)
    print("Payload dropped successfully!")
    set_servo(vehicle, 6, 1500)

# Main mission
def main():
    try:
        arm_and_takeoff(vehicle, 10)
        print("Scan the initial QR code:")
        initial_qr_data = scan_qr_from_image_file("/home/bodhini/Documents/QrCode/QR.png")
        if not initial_qr_data:
            print("No QR code detected at the launch site. Aborting mission.")
            return 

        target_lat = -35.36266700
        target_lon = 149.16403966
        qr_verified = go_to_location_with_qr_scan(vehicle, target_lat, target_lon, 10, initial_qr_data)

        if qr_verified:
            drop_payload()
        else:
            print("QR Code verification failed. Payload not dropped.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")
        vehicle.close()

if _name_ == "_main_":
    main()