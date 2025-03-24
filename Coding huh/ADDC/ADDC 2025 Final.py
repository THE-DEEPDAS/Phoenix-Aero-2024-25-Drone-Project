from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from pyzbar.pyzbar import decode
import time
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

#-------------- SETUP AND INITIALIZATION --------------#
# Setup command line argument parser for vehicle connection
parser = argparse.ArgumentParser(description='Controls vehicle and tracks objects using ArUco markers')
parser.add_argument('--connect', 
                   default='127.0.0.1:14550',
                   help="Vehicle connection string. If not specified, SITL simulator will be used.")
args = parser.parse_args()

# Initialize connection variables
connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

#-------------- UTILITY FUNCTIONS --------------#

def get_location_metres(original_location, dNorth, dEast):
    # Define Earth's radius in meters (WGS84 spherical approximation)
    earth_radius=6378137.0 
    
    # Convert North/East offset distances to changes in latitude/longitude
    # Uses the great circle approximation for small distances
    dLat = dNorth/earth_radius  # Convert distance to radians
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))  # Compensate for Earth's curvature
    print("dlat, dlon", dLat, dLon)  # Debug output for position calculations
    
    # Convert the offsets to actual latitude/longitude coordinates
    # Uses the equirectangular projection approximation valid for small distances
    newlat = original_location.lat + (dLat * 180/math.pi)  # Convert back to degrees
    newlon = original_location.lon + (dLon * 180/math.pi)  # Convert back to degrees
    return(newlat, newlon)

def set_servo(vehicle, servo_number, pwm_value):
    # Convert PWM value to integer as required by MAVLink protocol
    pwm_value_int = int(pwm_value)
    
    # Create MAVLink command message for servo control
    # Parameters: target system(0), target component(0), command(DO_SET_SERVO),
    # confirmation(0), servo number, PWM value, and empty parameters
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_number,
        pwm_value_int,
        0,0,0,0,0
    )
    # Send the command to the vehicle
    vehicle.send_mavlink(msg)

def marker_position_to_angle(x, y, z):
    # Calculate the angles using arctangent (atan2)
    # atan2 is used instead of atan as it handles all quadrants correctly
    angle_x = math.atan2(x,z)  # Calculate horizontal angle from z-axis
    angle_y = math.atan2(y,z)  # Calculate vertical angle from z-axis
    return (angle_x, angle_y)

def camera_to_uav(x_cam, y_cam):
    # Convert camera frame coordinates to UAV frame coordinates
    # Camera and UAV have different coordinate systems, need to transform between them
    x_uav = -y_cam    # Camera's y-axis becomes UAV's negative x-axis
    y_uav = x_cam     # Camera's x-axis becomes UAV's y-axis
    return(x_uav, y_uav)

def uav_to_ne(x_uav, y_uav, yaw_rad):
    # Convert UAV frame coordinates to North-East coordinates
    # Uses rotation matrix to transform coordinates based on UAV's yaw angle
    c = math.cos(yaw_rad)    # Cosine of yaw angle for rotation matrix
    s = math.sin(yaw_rad)    # Sine of yaw angle for rotation matrix
    # Apply rotation matrix transformation
    north = x_uav*c - y_uav*s    # Calculate North component using rotation matrix
    east = x_uav*s + y_uav*c     # Calculate East component using rotation matrix
    return(north, east)

def check_angle_descend(angle_x, angle_y, angle_desc):
    # Check if the drone is within acceptable angles for descent
    # Uses Pythagorean theorem to calculate total angular deviation
    # Returns True if the total angle is less than the descent threshold
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)

#-------------- CONNECTION
print('Connecting...')
vehicle = connect(args.connect, wait_ready=True, baud=921600)
print('Home is Defined')
vehicle.home_location = vehicle.location.global_frame

#-------------- PARAMETERS
rad_2_deg = 180.0/math.pi
deg_2_rad = 1.0/rad_2_deg

#-------------- LANDING MARKER
# ArUco marker ID to look for
id_to_find = 72                    # Specific ArUco marker ID to track
marker_size = 25                   # Physical size of marker in centimeters
freq_send = 1                      # Update frequency in Hz
land_alt_cm = 100.0               # Target landing altitude in centimeters
angle_descend = 15*deg_2_rad      # Maximum acceptable angle for descent
land_speed_cms = 70.0             # Landing speed in centimeters per second

# Camera calibration variables
camera_matrix = np.loadtxt(...)    # Camera intrinsic parameters
camera_distortion = np.loadtxt(...) # Camera distortion coefficients
aruco_tracker = ArucoSingleTracker(...) # ArUco marker detection object

time_0 = time.time()              # Initialize time for frequency control

#--- Get the camera calibration path
cwd = path.dirname(path.abspath(__file__))
calib_path = cwd+"/../opencv/"
camera_matrix = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')
aruco_tracker = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False,
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)

time_0 = time.time()

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()  # Wait until download is complete.

download_mission(vehicle)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    # Wait until the vehicle reaches a safe height
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(8)
print("Starting mission")
# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
# Access the waypoints and their count
waypoints = vehicle.commands
waypoint_count = waypoints.count

#-------------- MAIN FLIGHT LOOP --------------#
while True:
    # Check if reached final waypoint
    if vehicle.commands.next == waypoint_count:
        # Attempt to detect ArUco marker
        marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
        
        if marker_found:
            # Switch to precision landing mode
            vehicle.mode = VehicleMode("GUIDED")
            
            # Transform coordinates and calculate position
            x_cm, y_cm = camera_to_uav(x_cm, y_cm)
            uav_location = vehicle.location.global_relative_frame
            
            # Altitude handling logic
            if uav_location.alt >= 5.0:
                z_cm = uav_location.alt*100.0  # Use barometer for high altitudes
            
            # Calculate approach angles
            angle_x, angle_y = marker_position_to_angle(x_cm, y_cm, z_cm)
            
            # Update control at specified frequency
            if time.time() >= time_0 + 1.0/freq_send:
                time_0 = time.time()
                print(" ")
                print("Altitude = %.0fcm"%z_cm)
                print("Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg))
                north, east = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
                print("Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg))
                marker_lat, marker_lon = get_location_metres(uav_location, north*0.01, east*0.01)
                #-- If angle is good, descend
                if check_angle_descend(angle_x, angle_y, angle_descend):
                    print("Low error: descending")
                    location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.03/freq_send))
                else:
                    location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                vehicle.simple_goto(location_marker)
                print("UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
                print("Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))
            # Final landing sequence
            if(z_cm <= land_alt_cm+150):            
                if(aruco_tracker.detect_qr_code(15,True)): 
                    print("QR Code dETECTED")
                    print(" -->>COMMANDING TO LAND<<")
                    vehicle.mode = "LAND"
                    time.sleep(10)
                    set_servo(vehicle, 6, 1100)
                    print("servo khuli")
                    time.sleep(3)
                break
time.sleep(1)
arm_and_takeoff(8)
print('Return to launch')
time.sleep(1)
vehicle.mode = VehicleMode("RTL")
vehicle.armed = False
vehicle.close()
