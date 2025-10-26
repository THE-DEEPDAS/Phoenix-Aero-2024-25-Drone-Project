'''
Author: Deep Das
Time: 2021-07-25 20:00:00
'''

import cv2
import numpy as np
from pyzbar.pyzbar import decode
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import argparse
import math
import os

parser = argparse.ArgumentParser(description="Precision payload dropping with QR code verification.")
parser.add_argument('--connect', default='127.0.0.1:14550', help="Vehicle connection string.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

def connect_vehicle(connection_string):
    print(f"Connecting to vehicle on: {connection_string}")
    vehicle = connect(connection_string, wait_ready=True)
    print("Connected to vehicle.")
    return vehicle

vehicle = connect_vehicle(connection_string)

def arm_and_takeoff(vehicle, target_altitude):
    while not vehicle.is_armable:
        print("Waiting for drone to become armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for drone to arm...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Target altitude reached.")
            break
        time.sleep(1)

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c * 1000  # convert kiya hai 'm' mai

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

def drop_payload():
    print("Dropping payload...")
    set_servo(vehicle, 6, 1100)
    time.sleep(2)
    set_servo(vehicle, 6, 1500)
    print("Payload dropped.")

def scan_qr_code():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot access webcam.")
        return None

    qr_data = None
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading from webcam.")
            continue

        decoded_objects = decode(frame)
        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')
            print(f"QR Code Detected: {qr_data}")
            break

        if qr_data or cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return qr_data

def search_radius(vehicle, radius_m):
    print(f"Searching within {radius_m}m radius.")
    origin = vehicle.location.global_relative_frame
    for angle in range(0, 360, 45):  #45 degree ke ++ me sweep karega drone
        target_lat = origin.lat + (radius_m / 111320.0) * math.cos(math.radians(angle))
        target_lon = origin.lon + (radius_m / (111320.0 * math.cos(math.radians(origin.lat)))) * math.sin(math.radians(angle))
        target_location = LocationGlobalRelative(target_lat, target_lon, origin.alt)
        vehicle.simple_goto(target_location)
        time.sleep(5)  # Allow time to scan
        qr_data = scan_qr_code()
        if qr_data:
            return qr_data
    return None

def mission():
    try:
        arm_and_takeoff(vehicle, 10)
        target_lat = -35.36266700
        target_lon = 149.16403966
        target_alt = 10

        target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)
        vehicle.simple_goto(target_location)
        print("Flying to target location.")

        qr_data = scan_qr_code()
        if qr_data:
            print("First QR scan detected.")
            time.sleep(2)  # ye hovering hai for wind and all
            qr_confirm = scan_qr_code()
            if qr_confirm == qr_data:
                print("QR verified on second scan. Dropping payload.")
                drop_payload()
                return

        print("Scanning within 20m radius for QR.")
        radius_qr = search_radius(vehicle, 20)
        if radius_qr:
            print("QR found within radius. Dropping payload.")
            drop_payload()
        else:
            print("QR not found. Returning to Launch.")
    except Exception as e:
        print(f"Error during mission: {e}")
    finally:
        print("Returning to Launch (RTL).")
        vehicle.mode = VehicleMode("RTL")
        vehicle.close()

if __name__ == "__main__":
    mission()
