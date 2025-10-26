from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time
import math
import argparse
import subprocess

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Ensure necessary modules are installed
def install_dependencies():
    try:
        import dronekit
        import pymavlink
    except ImportError:
        print("Installing required dependencies...")
        subprocess.call(["pip", "install", "dronekit", "pymavlink"])
install_dependencies()

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/serial0')  # Use serial port for RPi
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------    

def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while vehicle.location.global_relative_frame.alt < aTargetAltitude * 0.95:
        time.sleep(1)

    print("Reached target altitude")

# Takeoff height in meters
arm_and_takeoff(8)
print("Take off complete")

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point...")
original_location = vehicle.location.global_relative_frame
point1 = LocationGlobalRelative(-35.36326170, 149.16462709, 10)
vehicle.simple_goto(point1, groundspeed=3)
time.sleep(20)

# Land directly instead of descending manually
print("Landing...")
vehicle.mode = VehicleMode("LAND")
while vehicle.location.global_relative_frame.alt > 1.0:
    time.sleep(1)

print("Landed successfully.")
time.sleep(5)

# Take off again
arm_and_takeoff(8)
print("Take off complete")

# Return to original location
print("Returning to original location...")
vehicle.simple_goto(original_location, groundspeed=3)
time.sleep(20)

# Land at original location
print("Landing at original location...")
vehicle.mode = VehicleMode("LAND")
while vehicle.location.global_relative_frame.alt > 0.5:
    time.sleep(1)

print("Mission complete.")

