###########DEPENDENCIES################
import time
import socket
import math
import argparse

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil

import cv2
import cv2.aruco as aruco
import numpy as np

from imutils.video import WebcamVideoStream
import imutils
from pyzbar.pyzbar import decode

#######VARIABLES####################
##Aruco
aruco_id_to_find = 72  # ArUco ID to search for
aruco_marker_size = 25  # Marker size in cm
takeoff_height = 10
velocity = .5

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()
##

##Camera
horizontal_res = 640
vertical_res = 480
# For Gazebo, we'll use a different approach to get camera feed
# We'll initialize this in main when needed
cap = None

horizontal_fov = 62.2 * (math.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

##Camera calibration
calib_path="/home/bodhini/Documents/opencv/"

# Load camera matrix from the provided file
try:
    cameraMatrix = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
    cameraDistortion = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')
except:
    # Default camera matrix for a 640x480 camera
    cameraMatrix = np.array([[500, 0, horizontal_res/2],
                            [0, 500, vertical_res/2],
                            [0, 0, 1]])
    cameraDistortion = np.zeros((1, 5))
    print("Warning: Using default camera calibration parameters")

##Counters and script triggers
found_count = 0
notfound_count = 0

first_run = 0 #Used to set initial time of function to determine FPS
start_time = 0
end_time = 0
script_mode = 1 ##1 for arm and takeoff, 2 for manual LOITER to GUIDED land 
ready_to_land = 0 ##1 to trigger landing

manualArm = False ##Set to False for simulation

# Landing parameters
qr_detect_altitude = 2.5  # Altitude to search for QR code
use_qr_for_landing = True  # Flag to enable QR code landing instead of second ArUco
qr_code_detected = False  # Flag to track QR code detection

# Add QR code validation parameter
expected_qr_code = "LANDING_PAD_1"  # Set your expected QR code value here

#########FUNCTIONS#################

def connectMyCopter():
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
    
    print(f"Connecting to vehicle on: {connection_string}")
    vehicle = connect(connection_string, wait_ready=True)
    
    return vehicle

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable.")
        time.sleep(1)
    print("Vehicle is now armable")
    
    vehicle.mode = VehicleMode("GUIDED")
            
    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")
    
    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    
    print("Look out! Props are spinning!!")
            
    vehicle.simple_takeoff(targetHeight) ##meters

    while True:
        print(f"Current Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        if vehicle.location.global_relative_frame.alt >= .95 * targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")

    return None

def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
def send_land_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0,)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to detect QR code and perform precision landing
def detect_qr_and_land():
    global qr_code_detected, cap
    current_alt = vehicle.location.global_relative_frame.alt
    
    # Only start QR detection at right altitude
    if current_alt > qr_detect_altitude + 0.2:  # Add small buffer
        print(f"Too high for QR detection: {current_alt:.2f}m")
        return False
        
    print("Searching for QR code...")
    start_time = time.time()
    timeout = 30  # Detection timeout in seconds
    
    while not qr_code_detected and (time.time() - start_time) < timeout:
        frame = cap.read()
        frame = cv2.resize(frame, (horizontal_res, vertical_res))
        display_frame = frame.copy()
        
        # QR code detection using pyzbar
        detected_codes = decode(frame)
        
        # Add drone status to display
        cv2.putText(display_frame, f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}m", 
                   (10, vertical_res - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        if detected_codes:
            for qr_code in detected_codes:
                # Get QR code data and position
                qr_data = qr_code.data.decode('utf-8')
                
                # Validate QR code content
                if qr_data != expected_qr_code:
                    print(f"Wrong QR code detected: {qr_data}")
                    continue
                
                print(f"Correct QR Code Detected: {qr_data}")
                
                # Get QR code position
                points = qr_code.polygon
                if len(points) == 4:
                    # Calculate center of QR code
                    center_x = sum(point.x for point in points) / 4
                    center_y = sum(point.y for point in points) / 4
                    
                    # Calculate normalized coordinates (-1 to 1)
                    norm_x = (center_x - horizontal_res/2) / (horizontal_res/2)
                    norm_y = (center_y - vertical_res/2) / (vertical_res/2)
                    
                    # Draw detection visualization
                    pts = np.array([(p.x, p.y) for p in points], np.int32)
                    cv2.polylines(display_frame, [pts], True, (0, 255, 0), 2)
                    cv2.putText(display_frame, qr_data, (int(center_x), int(center_y)), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    qr_code_detected = True
                    send_land_message(norm_x, norm_y)
                    
                    print(f"QR Code position: x={norm_x:.2f}, y={norm_y:.2f}")
                    break
        
        cv2.imshow("Drone Camera Feed", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        if qr_code_detected:
            print("Beginning precision landing sequence...")
            vehicle.mode = VehicleMode("LAND")
            while vehicle.armed:
                # Continue tracking QR code during landing
                frame = cap.read()
                detected_codes = decode(frame)
                if detected_codes:
                    points = detected_codes[0].polygon
                    if len(points) == 4:
                        center_x = sum(point.x for point in points) / 4
                        center_y = sum(point.y for point in points) / 4
                        norm_x = (center_x - horizontal_res/2) / (horizontal_res/2)
                        norm_y = (center_y - vertical_res/2) / (horizontal_res/2)
                        send_land_message(norm_x, norm_y)
                time.sleep(0.1)
            return True
    
    if not qr_code_detected:
        print("QR code detection timed out. Switching to regular landing.")
        vehicle.mode = VehicleMode("LAND")
    
    cv2.destroyAllWindows()
    return False

# Function to handle initial ArUco marker detection and vehicle navigation
def lander():
    global first_run, notfound_count, found_count, start_time, cap
    
    if first_run == 0:
        print("First run of lander function!")
        first_run = 1
        start_time = time.time()
        cv2.namedWindow("Drone Camera Feed", cv2.WINDOW_NORMAL)

    frame = cap.read()
    frame = cv2.resize(frame, (horizontal_res, vertical_res))
    display_frame = frame.copy()
    current_altitude = vehicle.location.global_relative_frame.alt
    
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
    frame_with_markers = aruco.drawDetectedMarkers(display_frame, corners, ids)
    
    # Add drone status display
    cv2.putText(frame_with_markers, f"Altitude: {current_altitude:.2f}m", 
               (10, vertical_res - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame_with_markers, f"Mode: {vehicle.mode.name}", 
               (10, vertical_res - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    if ids is not None:
        for i, id in enumerate(ids):
            if id[0] == aruco_id_to_find:
                found_count += 1
                ret = aruco.estimatePoseSingleMarkers(corners[i], aruco_marker_size, cameraMatrix, cameraDistortion)
                (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                
                marker_center = np.mean(corners[i][0], axis=0)
                norm_x = (marker_center[0] - horizontal_res/2) / (horizontal_res/2)
                norm_y = (marker_center[1] - vertical_res/2) / (vertical_res/2)
                
                # Calculate descent rate based on current altitude
                if current_altitude > 5:
                    descent_rate = -0.5  # Faster descent at higher altitudes
                else:
                    descent_rate = -0.3  # Slower descent when closer to ground
                
                # IMPORTANT: Calculate vertical velocity based on current altitude
                current_alt = vehicle.location.global_relative_frame.alt
                vertical_velocity = 0
                
                # Add descent velocity whenever marker is detected
                if current_alt > qr_detect_altitude + 0.5:  # Add buffer
                    vertical_velocity = 0.3  # Descend at 0.3 m/s
                elif current_alt > qr_detect_altitude:
                    vertical_velocity = 0.1  # Slower descent near target altitude
                
                if use_qr_for_landing:
                    if abs(norm_x) < 0.2 and abs(norm_y) < 0.2:
                        if current_alt > qr_detect_altitude:
                            # Key change: Added negative vertical_velocity for descent
                            send_local_ned_velocity(-norm_y * velocity, norm_x * velocity, vertical_velocity)
                            print(f"Descending to QR altitude. Current height: {current_alt:.2f}m")
                        else:
                            print("Reached QR detection altitude, switching to QR detection...")
                            detect_qr_and_land()
                            return True
                    else:
                        # Move horizontally and descend simultaneously
                        send_local_ned_velocity(-norm_y * velocity, norm_x * velocity, vertical_velocity)
                        print(f"Aligning with ArUco and descending. Offset: x={norm_x:.2f}, y={norm_y:.2f}")
                else:
                    # ArUco-only landing with controlled descent
                    send_land_message(norm_x, norm_y)
                    send_local_ned_velocity(-norm_y * velocity, norm_x * velocity, descent_rate)
                    
                    if current_altitude < 0.5:  # Final landing phase
                        vehicle.mode = VehicleMode("LAND")
                        return True
                
                # Update display
                cv2.putText(frame_with_markers, f"TRACKING ARUCO: x={norm_x:.2f}, y={norm_y:.2f}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.imshow("Drone Camera Feed", frame_with_markers)
                cv2.waitKey(1)
                return False
    
    notfound_count += 1
    cv2.putText(frame_with_markers, "SEARCHING FOR ARUCO", (10, 60), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.imshow("Drone Camera Feed", frame_with_markers)
    cv2.waitKey(1)
    
    # If no marker found, maintain altitude while searching
    send_local_ned_velocity(0, 0, 0)
    return False

######################################################

#######################MAIN###########################

######################################################

try:
    # Connect to the Vehicle
    vehicle = connectMyCopter()
    print("Connected to vehicle!")
    
    # Initialize camera
    print("Initializing camera...")
    cap = WebcamVideoStream(src=0).start()
    time.sleep(2)  # Give camera time to initialize
    
    # Check if camera is working
    test_frame = cap.read()
    if test_frame is None:
        print("ERROR: Could not read from camera. Check camera connection.")
        raise Exception("Camera initialization failed")
    else:
        print("Camera initialized successfully!")
    
    # Setup parameters to enable precision landing
    print("Setting precision landing parameters...")
    vehicle.parameters['PLND_ENABLED'] = 1
    vehicle.parameters['PLND_TYPE'] = 1  # 1 for companion computer
    vehicle.parameters['PLND_EST_TYPE'] = 0  # 0 for raw sensor, 1 for kalman filter pos estimation
    vehicle.parameters['LAND_SPEED'] = 20  # Descent speed of 20cm/s
    
    if script_mode == 1:
        # Arm and takeoff
        arm_and_takeoff(takeoff_height)
        print(str(time.time()))
        # Optional: offset drone from target
        #send_local_ned_velocity(velocity, velocity, 0)
        time.sleep(1)
        ready_to_land = 1
    elif script_mode == 2:
        # Wait for manual mode change
        while vehicle.mode != 'GUIDED':
            time.sleep(1)
            print(f"Waiting for manual change from mode {vehicle.mode} to GUIDED")
        ready_to_land = 1
    
    if ready_to_land == 1:
        print("Beginning landing sequence...")
        landing_complete = False
        
        while vehicle.armed and not landing_complete:
            landing_complete = lander()
            time.sleep(0.1)  # Small delay to prevent CPU hogging
        
        # Calculate stats
        end_time = time.time()
        total_time = end_time - start_time
        total_time = abs(int(total_time))
        
        total_count = found_count + notfound_count
        if total_time > 0:
            freq_lander = total_count / total_time
        else:
            freq_lander = 0
            
        print("Total iterations: " + str(total_count))
        print("Total seconds: " + str(total_time))
        print("------------------")
        print("lander function had frequency of: " + str(freq_lander))
        print("------------------")
        print("Vehicle has landed")
        print("------------------")

except Exception as e:
    print(f"An error occurred: {e}")
    
finally:
    # Clean up
    if cap is not None:
        cap.stop()
    cv2.destroyAllWindows()
    print("Script completed!")
