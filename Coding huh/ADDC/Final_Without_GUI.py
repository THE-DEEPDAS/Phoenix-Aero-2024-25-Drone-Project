from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
import time
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import cv2
import numpy as np
from pyzbar.pyzbar import decode

parser = argparse.ArgumentParser()
parser.add_argument('--connect' , default = '/dev/ttyACM0')
args = parser.parse_args()
    
#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------    
def get_location_metres(original_location, dNorth, dEast):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
    print ("dlat, dlon", dLat, dLon)
    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)
    
def set_servo(vehicle, servo_number, pwm_value):
	pwm_value_int = int(pwm_value)
	msg = vehicle.message_factory.command_long_encode(
		0, 0, 
		mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
		0,
		servo_number,
		pwm_value_int,
		0,0,0,0,0
		)
	vehicle.send_mavlink(msg)   
def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)
        
#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
print('Connecting...')
vehicle = connect(args.connect , wait_ready=True , baud = 921600)
print('Home is Defined')  
vehicle.home_location = vehicle.location.global_frame
#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 
#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
marker_size     = 25 #- [cm] - Size of the printed QR code
freq_send       = 1 #- Hz
land_alt_cm     = 300.0
angle_descend   = 15*deg_2_rad
land_speed_cms  = 50.0
#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')                                      
                
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
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(8)
print("Starting mission")
# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
# Access the waypoints and their count
waypoints = vehicle.commands
waypoint_count = waypoints.count
while True:            
	
    if vehicle.commands.next == waypoint_count:
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to capture frame")
            continue
            
        decoded_objects = decode(frame)
        marker_found = False
        
        if decoded_objects:
            marker_found = True
            qr_data = decoded_objects[0].data.decode('utf-8')
            print(f"QR Code detected. Data: {qr_data}")
            
            # Get QR code corners
            points = decoded_objects[0].polygon
            if len(points) == 4:
                # Calculate center
                center_x = sum(p.x for p in points) / 4
                center_y = sum(p.y for p in points) / 4
                
                # Draw QR detection visualization
                frame_height, frame_width = frame.shape[:2]
                x_cm = (center_x - frame_width/2) * marker_size / frame_width
                y_cm = (center_y - frame_height/2) * marker_size / frame_height
                z_cm = vehicle.location.global_relative_frame.alt * 100.0
                
                # Draw rectangle around QR code
                pts = np.array([(p.x, p.y) for p in points], np.int32)
                cv2.polylines(frame, [pts], True, (0,255,0), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0,0,255), -1)
                
                # Save debug frame occasionally
                if time.time() % 5 < 0.1:  # Save every 5 seconds
                    cv2.imwrite(f"qr_debug_{time.strftime('%H%M%S')}.jpg", frame)

        cap.release()
        if marker_found:
            vehicle.mode = VehicleMode("GUIDED")
            x_cm, y_cm = camera_to_uav(x_cm, y_cm)
            uav_location = vehicle.location.global_relative_frame
            
            #-- If high altitude, use baro rather than visual
            if uav_location.alt >= 5.0:
                print 
                z_cm = uav_location.alt*100.0
                
            angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)
            
            if time.time() >= time_0 + 1.0/freq_send:
                time_0 = time.time()
                # print ""
                print (" ")
                print ("Altitude = %.0fcm"%z_cm)
                print ("Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg))
                
                north, east             = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
                print ("Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg))
                
                marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)  
                #-- If angle is good, descend
                if check_angle_descend(angle_x, angle_y, angle_descend):
                    print ("Low error: descending")
                    location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.04/freq_send))
                else:
                    location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                    
                vehicle.simple_goto(location_marker)
                print ("UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
                print ("Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))
                
            #--- COmmand to land
            if z_cm <= land_alt_cm:
                if vehicle.mode == "GUIDED":
                    print (" -->>COMMANDING TO LAND<<")
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
