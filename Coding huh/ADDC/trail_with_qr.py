from os import sys, path

sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time

import math

import argparse

import cv2

import numpy as np

from dronekit import connect, VehicleMode, LocationGlobalRelative

from pymavlink import mavutil

from opencv.library import *

from pyzbar.pyzbar import decode



parser = argparse.ArgumentParser()

parser.add_argument('--connect', default='/dev/ttyACM0')

args = parser.parse_args()



# Connect to vehicle

print('Connecting...')

vehicle = connect(args.connect, wait_ready=True, baud=921600)

vehicle.home_location = vehicle.location.global_frame



# Constants

rad_2_deg = 180.0 / math.pi

deg_2_rad = 1.0 / rad_2_deg

id_to_find = 72

marker_size = 25  # cm

freq_send = 1  # Hz

land_alt_cm = 200.0

angle_descend = 15 * deg_2_rad

land_speed_cms = 40.0



time_0 = time.time()



# Load camera calibration

cwd = path.dirname(path.abspath(__file__))

calib_path = cwd + "/../opencv/"

camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_raspi.txt', delimiter=',')

camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_raspi.txt', delimiter=',')

aruco_tracker = ArucoSingleTracker(id_to_find, marker_size, False, camera_matrix, camera_distortion)



def scan_qr_code(image):

    decoded_objects = decode(image)

    for obj in decoded_objects:

        qr_data = obj.data.decode("utf-8")

        print("QR Code Data:", qr_data)

        return qr_data

    return None



def arm_and_takeoff(aTargetAltitude):

    print("Basic pre-arm checks")

    while not vehicle.is_armable:

        print(" Waiting for vehicle to initialise...")

        time.sleep(1)

    print("Arming motors")

    vehicle.mode = VehicleMode("GUIDED")

    vehicle.armed = True

    while not vehicle.armed:

        print(" Waiting for arming...")

        time.sleep(1)

    print("Taking off!")

    vehicle.simple_takeoff(aTargetAltitude)

    while True:

        print(" Altitude: ", vehicle.location.global_relative_frame.alt)

        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:

            print("Reached target altitude")

            break

        time.sleep(1)



def land_on_marker():

    global time_0

    while True:

        marker_found, x_cm, y_cm, z_cm, frame = aruco_tracker.track(loop=False)  # Updated to return frame

        if _found:

            vehicle.mode = VehicleMode("GUIDED")

            x_cm, y_cm = -y_cm, x_cm

            uav_location = vehicle.location.global_relative_frame

            if uav_location.alt >= 5.0:

                z_cm = uav_location.alt * 100.0

            angle_x, angle_y = math.atan2(x_cm, z_cm), math.atan2(y_cm, z_cm)

            if time.time() >= time_0 + 1.0 / freq_send:

                time_0 = time.time()

                north, east = x_cm * 0.01, y_cm * 0.01

                marker_lat, marker_lon = uav_location.lat + north, uav_location.lon + east

                if math.sqrt(angle_x**2 + angle_y**2) <= angle_descend:

                    location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt - (land_speed_cms * 0.04 / freq_send))

                else:

                    location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)

                vehicle.simple_goto(location_marker)

                print("Commanding to:", marker_lat, marker_lon)

            if z_cm <= land_alt_cm:

                vehicle.mode = VehicleMode("LAND")

                time.sleep(10)

                print("Scanning QR Code...")

                qr_data = scan_qr_code(frame)

                if qr_data:

                    print("QR Code Detected:", qr_data)

                break

        time.sleep(1)



arm_and_takeoff(8)

print("Starting mission")

vehicle.mode = VehicleMode("AUTO")

land_on_marker()

print('Return to launch')

vehicle.mode = VehicleMode("RTL")

vehicle.armed = False

vehicle.close()

