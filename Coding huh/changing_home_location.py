#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2024-25, Bodhi
all rights are reserved with Jain_empire.pvt
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',default='127.0.0.1:14550',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(10)
Home=vehicle.location.global_frame
Home.alt=10
print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards point B")
point1 = LocationGlobalRelative(-35.36254347,149.16457651, 10)
vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(30)
vehicle.mode=VehicleMode("LAND")
time.sleep(30)

while vehicle.armed:
        print(" Waiting for disarming...")
        time.sleep(2)
        
arm_and_takeoff(10)

print("Going towards point C")
point2 = LocationGlobalRelative(-35.36327333,149.16292425 , 10)
vehicle.simple_goto(point2, groundspeed=20)

# sleep so we can see the change in map
time.sleep(30)
vehicle.mode=VehicleMode("LAND")
time.sleep(30)

while vehicle.armed:
        print(" Waiting for disarming...")
        time.sleep(2)
        
'''arm_and_takeoff(10)
print("Landing towards initial home")
point3 = LocationGlobalRelative(Home.lat,Home.lon,10)
vehicle.simple_goto(point3, groundspeed=20)
time.sleep(30)
vehicle.mode = VehicleMode("LAND")
time.sleep(30)'''
arm_and_takeoff(10)
vehicle.home_location = Home

print("Now rtl to intial home")
vehicle.mode = VehicleMode("RTL")


# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
