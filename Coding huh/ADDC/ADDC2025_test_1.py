import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Connect to the Vehicle
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--connect' , default = '/dev/ttyACM0')
args = parser.parse_args()

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    mode = VehicleMode("GUIDED")
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

arm_and_takeoff(10)
Home = vehicle.location.global_frame
Home.alt = 10
print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards point B")
point1 = LocationGlobalRelative(-35.36254347, 149.16457651, 10)
vehicle.simple_goto(point1)
time.sleep(30)
vehicle.mode = VehicleMode("LAND")
time.sleep(30)

while vehicle.armed:
    print(" Waiting for disarming...")
    time.sleep(2)

arm_and_takeoff(10)
print("Going towards point C")
point2 = LocationGlobalRelative(-35.36327333, 149.16292425, 10)
vehicle.simple_goto(point2, groundspeed=50)
time.sleep(30)
vehicle.mode = VehicleMode("LAND")
time.sleep(30)

while vehicle.armed:
    print(" Waiting for disarming...")
    time.sleep(2)

arm_and_takeoff(10)
vehicle.home_location = Home

print("Now RTL to initial home")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

