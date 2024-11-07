from __future__ import print_function
import time
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import argparse

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
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,    
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  
        0b0000111111000111,   
        0, 0, 0,  
        velocity_x, velocity_y, velocity_z,  
        0, 0, 0,  
        0, 0)  
    vehicle.send_mavlink(msg)
    vehicle.flush()

# HSV range for red color detection
lower_red_1 = np.array([0, 100, 100])
upper_red_1 = np.array([10, 255, 255])
lower_red_2 = np.array([170, 100, 100])
upper_red_2 = np.array([180, 255, 255])

# Frame properties
frame_width = 640
frame_height = 480

# Center of the frame
center_x_frame = frame_width // 2
center_y_frame = frame_height // 2

# Tolerances
x_tolerance = 20  # Horizontal tolerance for movement
y_tolerance = 20  # Vertical tolerance for movement

# Initialize global variables for trace points and timing
trace_points = []
point_times = []

def draw_grid(frame):
    for i in range(1, 8):
        cv2.line(frame, (i * (frame_width // 8), 0), (i * (frame_width // 8), frame_height), (0, 255, 0), 1)
        cv2.line(frame, (0, i * (frame_height // 8)), (frame_width, i * (frame_height // 8)), (0, 255, 0), 1)

def process_frame(frame):
    global trace_points, point_times
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = mask1 | mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest_contour) > 500:
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2
            center = (center_x, center_y)

            trace_points.append(center)
            point_times.append(time.time())

            current_time = time.time()
            while point_times and (current_time - point_times[0] >= 3):
                trace_points.pop(0)
                point_times.pop(0)

            x_offset = center_x - center_x_frame
            y_offset = center_y - center_y_frame

            forward_back_velocity = 0
            right_left_velocity = 0
            down_up_velocity = 0  # Initialize vertical velocity

            # Adjust right-left velocity based on the direction of the red object's movement
            if abs(x_offset) > x_tolerance:
                if x_offset < 0:
                    right_left_velocity = -0.5  # Move left
                else:
                    right_left_velocity = 0.5  # Move right

            # Keep pitch adjustment as is
            if abs(y_offset) > y_tolerance:
                forward_back_velocity = -0.5 if y_offset < 0 else 0.5  # Forward or backward
                down_up_velocity = -0.5 if y_offset < 0 else 0.5  # Pitch up or down
            else:
                down_up_velocity = 0  # No vertical movement

            send_ned_velocity(forward_back_velocity, right_left_velocity, down_up_velocity)

            for i in range(len(trace_points) - 1):
                cv2.line(frame, trace_points[i], trace_points[i + 1], (0, 255, 0), 2)

            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

    draw_grid(frame)
def set_servo(vehicle, servo_number, pwm_value):
    pwm_value_int = int(pwm_value)
    print("Procedure has started")
    msg = vehicle.message_factory.command_long_encode(
        0, 0, 
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_number,
        pwm_value_int,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()  # Ensure message is sent
    print(f"Sent servo command: Servo {servo_number}, PWM {pwm_value_int}")

# Arm the drone and take off
arm_and_takeoff(10)

# Set airspeed
print("Set default/target airspeed to 3")
vehicle.airspeed = 3

# Fly to the target location (waypoint)
print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(-35.36180508,149.16519710, 20)
vehicle.simple_goto(point1)

cap = cv2.VideoCapture(0)  # Webcam for testing, replace with drone camera feed

start_time = time.time()
while time.time() - start_time < 60:  # Track the red object for 30 seconds
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    process_frame(frame)

    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if time.time() - start_time > 300:
        print("Stopping camera feed after 5 minutes...")
        break

# Sleep to allow time for the drone to reach the waypoint
time.sleep(30)

# Start the object tracking process


# After tracking for 30 seconds, descend to 1m and hover
print("Descending to 1 meter altitude")
vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat,
                                           vehicle.location.global_frame.lon, 1))

# Wait for 30 seconds at 1 meter altitude
set_servo(vehicle, 6, 1100)  
time.sleep(10)
print("Servo khul gyi")
set_servo(vehicle, 6, 1100) 
print("Servo band ho gayi bs ab rtl ")
time.sleep(10)
# Perform RTL (Return to Launch)
print("Returning to Launch (RTL)")
vehicle.mode = VehicleMode("RTL")

# Close vehicle and camera feed
cap.release()
cv2.destroyAllWindows()
vehicle.close()

# Shut down simulator if it was started
if sitl:
    sitl.stop()