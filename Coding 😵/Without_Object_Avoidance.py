#!/usr/bin/env python3

'''
1) drone scans for initial qr to get target location
2) drone takes off to a specified altitude
3) drone goes to the target location
4) camera feed begins 5m before reaching the coordinate
5) drone looks for a specific QR code
6) if QR code is not found, drone moves 5m ahead to search for QR code
7) if QR code is not found, drone lands at target location and RTLs to home
8) if QR code is found, drone uses PID to center itself above the QR code
9) once centered, drone descends and lands on the QR code
10) after landing, drone RTLs to original home location
'''


# IMPORT REQUIRED LIBRARIES
import time
import math
import argparse
import cv2
from pyzbar.pyzbar import decode
from PIL import Image
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil


# DEFINE CONSTANTS
FACTOR = 1.113195e5
EARTH_RADIUS = 6371
OG_HOME = None  # Original home location (to be set later)
DETECTED = False


# CONNECT THE VEHICLE
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.",
                    default='127.0.0.1:14550')
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


# ARM AND TAKEOFF FUNCTION
def arm_and_takeoff(target_altitude):
    """Arm the vehicle and fly to target_altitude (m)."""

    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    
    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt} m")
        if alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# SEND BODY NED VELOCITY FUNCTION
def send_local_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    velocity_x: velocity in m/s along North direction
    velocity_y: velocity in m/s along East direction
    velocity_z: velocity in m/s along Down direction (positive downward)
    """

    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                                     # time_boot_ms (not used)
        0, 0,                                  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,    # frame
        0b0000111111000111,                    # type_mask (only speeds enabled)
        0, 0, 0,                               # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,    # x, y, z velocity in m/s
        0, 0, 0,                               # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)                                  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    vehicle.send_mavlink(msg)
    vehicle.flush()



# YAW FUNCTION
def condition_yaw(heading, is_relative,direction,duration=2):
    
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    heading: yaw in degrees, 0-360
    is_relative: 1 for relative yaw, 0 for absolute angle
    direction: -1 for counter-clockwise, 1 for clockwise, 0 for shortest path
    """

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,                                  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,                                     #confirmation
        heading,                               # param 1, yaw in degrees
        0,                                     # param 2, yaw speed deg/s
        direction,                             # param 3, direction -1 ccw, 1 cw
        is_relative,                           # param 4, relative offset 1, absolute angle 0
        0, 0, 0)                               # param 5 ~ 7 not used

    for i in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
    
    vehicle.flush()


# PID CLASS

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.sum_error = 0

    def pid_control(self, error, dt):
        error = -error
        self.sum_error += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = (self.kp * error) + (self.ki * self.sum_error) + (self.kd * derivative)
        self.prev_error = error

        return output

pid_x = PID(kp=0.003, ki=0.0, kd=0.004)
pid_y = PID(kp=0.003, ki=0.0, kd=0.004)



# INITIALISE QR
print("Initialising QR detection...")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not detect camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Could not capture frame.")
        continue
    qrs = decode(frame)

    if qrs:
        og_data = qrs[0].data.decode('utf-8')
        data = qrs[0].data.decode('utf-8').strip()
        print(f"Initialized QR code with data: {qrs[0].data.decode('utf-8')}")
        try:
            # Remove parentheses and spaces
            data = data.replace('(', '').replace(')', '').replace(' ', '')
            lat_str, lon_str = data.split(',')
            target_lat = float(lat_str)
            target_lon = float(lon_str)
            print(f"Extracted target coordinates -> Latitude: {target_lat}, Longitude: {target_lon}")
            break
        except Exception as e:
            print(f"Error decoding QR data: {e}")
            continue

    cv2.imshow("QR Initialization", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


# ARM AND TAKEOFF TO A SPECIFIED ALTITUDE
arm_and_takeoff(10)
vehicle.groundspeed = 3

# GO TO A SPECIFIC COORDINATE
point1 = LocationGlobalRelative(target_lat, target_lon, 10)
vehicle.simple_goto(point1)
print("Searching for QR code...")

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not detect camera.")
    vehicle.mode = VehicleMode("RTL")
    vehicle.close()
    exit()

qrs = None

while not qrs and not DETECTED:        
    ret, frame = cap.read()

    if not ret:
        print("Could not capture frame.")
        continue

    qrs = decode(frame)

    if qrs:
        if qrs[0].data.decode('utf-8') == og_data:
            print(f"Detected QR code with data: {qrs[0].data.decode('utf-8')}")
            print("QR code detected. Centering above QR code...")
        DETECTED = True
        break

    current_location = vehicle.location.global_relative_frame
    dist = FACTOR*((current_location.lat-point1.lat)**2+(current_location.lon-point1.lon)**2)**0.5
    
    print(f"distance to point 1: {round(dist,2)} m")

    if dist <= 5:
        break

    cv2.imshow("PID QR Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


print("Waiting to set home location...")
while vehicle.home_location is None:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if vehicle.home_location is not None:
        break
OG_HOME = vehicle.home_location  # Set the original home location


# LOOK FOR QR AROUND
# IF QR SPOTTED MAKE THE DRONE FLY ABOVE THE QR

while not qrs and not DETECTED:        
    ret, frame = cap.read()

    if not ret:
        print("Could not capture frame.")
        continue

    qrs = decode(frame)

    if qrs:
        if qrs[0].data.decode('utf-8') == og_data:
            print(f"Detected QR code with data: {qrs[0].data.decode('utf-8')}")
            print("QR code detected. Centering above QR code...")
        DETECTED = True
        break

    send_local_ned_velocity(3, 0, 0, 1)
    print(f"distance to point 1: {round(dist,2)} m")
    current_location = vehicle.location.global_relative_frame
    dist = FACTOR*((current_location.lat-point1.lat)**2+(current_location.lon-point1.lon)**2)**0.5

    if dist > 5:                         # if drone has moved more than 5m ahead of target point without finding QR
        cap.release()
        cv2.destroyAllWindows()
        print("Could not find QR code, landing at target location and returning home...")
        vehicle.simple_goto(point1)
        time.sleep(5)
        while vehicle.location.global_relative_frame.alt > 3:
            send_local_ned_velocity(0,0,1,1)
        vehicle.mode = VehicleMode("LAND")
        time.sleep(15)
        vehicle.home_location = OG_HOME
        vehicle.mode = VehicleMode("RTL")
        time.sleep(2)
        vehicle.close()

        exit()

    else:
        cv2.imshow("PID QR Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

cv2.destroyAllWindows()

then = time.time()
vehicle.mode = VehicleMode("GUIDED")
condition_yaw(vehicle.heading, is_relative=0, direction=0)

while True:

    ret, frame = cap.read()
    if not ret:
        print("Could not capture frame.")
        continue

    qrs = decode(frame)

    if qrs:
        if qrs[0].data.decode('utf-8') == og_data:
            vertices = qrs[0].polygon

            cx , cy = frame.shape[1]//2, frame.shape[0]//2   # centre of frame, x is height, y is width
            x = int((vertices[0][0] + vertices[1][0]) // 2)  # centre of detected qr
            y = int((vertices[0][1] + vertices[3][1]) // 2)  # centre of detected qr
            error_x = x - cx
            error_y = y - cy

            if math.sqrt(error_x**2 + error_y**2) < 50:
                break

            now = time.time()
            dt = now - then
            then = now

            vx = -pid_x.pid_control(error_x, dt)    # change in velocity left/right
            vy = pid_y.pid_control(error_y, dt)     # change in velocity forward/backward
            vz = 0                                  # no change in altitude

            vx = np.clip(vx, -1, 1)
            vy = np.clip(vy, -1, 1)

            send_local_ned_velocity(vy, vx, vz, 1)

            # visualise

            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(frame, (cx, cy), (x, y), (255, 0, 0), 2)
            cv2.rectangle(frame, (int(vertices[0][0]), int(vertices[0][1])), (int(vertices[2][0]), int(vertices[2][1])), (0, 255, 0), 2)
            cv2.putText(frame,f"Error: {math.sqrt(error_x**2 + error_y**2)}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
            

        else:
            send_local_ned_velocity(0, 0, 0, 1)  # hover in place if no QR detected
        
    else:
        send_local_ned_velocity(0, 0, 0, 1)  # hover in place if no QR detected

        

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.imshow("PID QR Tracking", frame)

cap.release()
cv2.destroyAllWindows()

# MAKE THE DRONE LAND ON THE QR
print("QR centered, descending...")
while vehicle.location.global_relative_frame.alt > 3:
    send_local_ned_velocity(0,0,1,1)

vehicle.mode = VehicleMode("LAND")
time.sleep(15)

# RETURN TO ORIGINAL HOME LOCATION
print("Returning to original home location...")
vehicle.home_location = OG_HOME
vehicle.mode = VehicleMode("RTL")
time.sleep(2)

# CLEAN UP THE CODE
vehicle.close()