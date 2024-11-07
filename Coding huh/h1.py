"""
NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink

cd dronekit-python
git pull

sudo python setup.py build
sudo python setup.py install

Be sure the RASPI CAMERA driver is correctly acivated -> type the following
modprobe bcm2835-v4l2
"""

import threading
from playsound import playsound
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time
import math
import argparse

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------   

 

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
    print ("dlat, dlon", dLat, dLon)

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)

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
    
def actuate_servo(vehicle, channel, pwm_value):
    # Send MAVLink command to set servo PWM
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # Target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command ID
        0,      # Confirmation
        channel,  # Servo number
        pwm_value,  # PWM value
        0, 0, 0, 0, 0  # Parameters 4-9 (not used)
    )
    vehicle.send_mavlink(msg)
    print(f"Sent MAVLink command to set servo {channel} to {pwm_value}")
def play_sound():
    playsound('/home/bodhini/how_do_drones_work-master (copy)/scripts/countdown.mp3')
 
vehicle.home_location=vehicle.location.global_frame
# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):
  
  print ("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  
  sound_thread = threading.Thread(target=play_sound)
  sound_thread.start()
  time.sleep(5)
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
  time.sleep(3)
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    
    time.sleep(1)
  
  

  print ("Taking off!") 
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
  
  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt)
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

# Takeoff and set airspeed
# Takeoff and set airspeed


# Play the sound

arm_and_takeoff(8)
print("Take off complete")

print("Set default/target airspeed to 3")
vehicle.airspeed = 3
point1 = LocationGlobalRelative(-35.36326170, 149.16462709, 10)


vehicle.simple_goto(point1, groundspeed=50)
print("Moving towards point1...")


#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0 / math.pi
deg_2_rad   = 1.0 / rad_2_deg 

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
id_to_find      = 72
marker_size     = 10 # [cm]
freq_send       = 2  # Increased to 2 Hz for quicker response

# While testing in drone test as per platform height
land_alt_cm         = 50.0
angle_descend       = 20 * deg_2_rad
land_speed_cms      = 30.0

#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd + "/../opencv/"
camera_matrix       = np.loadtxt(calib_path + 'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path + 'cameraDistortion_raspi.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False, 
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)

time_0 = time.time()

# Sleep so we can see the change in map

# Start moving towards point1 right after takeoff


# Loop for marker detection and RTL
while True:
    # Check for the ArUco marker while moving towards point1
    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    
    if marker_found:
        print("Marker detected! ")
        vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat+1,vehicle.location.global_frame.lon, vehicle.location.global_frame.alt))
        vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon, 1))
        
        #vehicle.mode=VehicleMode("RTL") 
        break 
    tolerance = 0.0001  
    current_location = vehicle.location.global_frame

    if (abs(current_location.lat - point1.lat) < tolerance) and (abs(current_location.lon - point1.lon) < tolerance):
        print("I have reached")
        if not marker_found:
        	print("Marker not detected! ")
            vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon, 1))        	
        	#vehicle.mode=VehicleMode("RTL")
        	break
    # Calculate distance to point1
    current_location = vehicle.location.global_relative_frame
    dlat = point1.lat - current_location.lat
    dlong = point1.lon - current_location.lon
    distance_to_point1 = math.sqrt((dlat * 1.113195e5) ** 2 + (dlong * 1.113195e5 * math.cos(math.radians(current_location.lat))) ** 2)
    
    # Check distance to point1
    '''if distance_to_point1 <= 1.0:
        print("Reached point1. Continuing to look for ArUco marker.")
        break'''
  # Adjust based on your accuracy needs


print("Me ek m par ja rha hu")
time.sleep(5)
print("Now hovering for 30secs")
print("Actuating servo...")
actuate_servo(vehicle, 6, 1100)
time.sleep(30)       
vehicle.mode=VehicleMode("RTL")
# Return to Launch (RTL) if ArUco marker was detected
if vehicle.mode.name == "RTL":
    # Wait for RTL to complete
    while vehicle.location.global_relative_frame.alt > 1.0:
        print("Returning to launch... Altitude: %.2f" % vehicle.location.global_relative_frame.alt)
        time.sleep(1)

# Optionally disarm after landing
'''if vehicle.location.global_relative_frame.alt <= 1.0:
    print("Landed. Disarming...")
    vehicle.armed = False
    time.sleep(5)  '''  