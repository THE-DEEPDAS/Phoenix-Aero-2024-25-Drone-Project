from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse

# Argument parser to get connection string
parser = argparse.ArgumentParser(description='Simple drone movement using DroneKit.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL is started.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

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

# Start mission
arm_and_takeoff(8)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards target point for 30 seconds ...")
target_location = LocationGlobalRelative(-35.361354, 149.165218, 8)
vehicle.simple_goto(target_location)

# Wait to reach destination
time.sleep(30)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle
print("Closing vehicle connection")
vehicle.close()
