from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from pyzbar.pyzbar import decode
import time
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

'''parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyACM0')
args = parser.parse_args()
vehicle = connect(args.connect, wait_ready=True)'''

class FakeDrone:
    def __init__(self, initial_lat=21.1600980, initial_lon=72.7867741, initial_alt=0.0):
        # Basic drone state
        self.is_armable = True
        self.armed = False
        self._mode = "GUIDED"
        self.commands = FakeCommands()
        
        # Drone location and movement
        self.location = type('Location', (), {})()
        self.location.global_relative_frame = type('GlobalRelativeFrame', (), {
            'lat': initial_lat,
            'lon': initial_lon,
            'alt': initial_alt
        })()
        
        # Drone parameters
        self.home_location = LocationGlobal(initial_lat, initial_lon, 0)
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self._target_location = None
        self._movement_thread = None
        self._movement_active = False
        
        # Create message factory for servo control
        self.message_factory = type('MessageFactory', (), {})()
        
        print(f"Fake drone initialized at: Lat={initial_lat}, Lon={initial_lon}, Alt={initial_alt}")
    
    @property
    def mode(self):
        return self._mode
        
    @mode.setter
    def mode(self, new_mode):
        if isinstance(new_mode, str):
            self._mode = new_mode
        else:
            self._mode = new_mode.name
        print(f"Drone mode changed to: {self._mode}")
        
        if self._mode == "RTL":
            # Handle Return to Launch
            self._target_location = LocationGlobalRelative(21.1600980, 72.7867741, 10)
            self._start_movement_thread()
            time.sleep(2)  # Simulate travel time
            # Then descend to ground
            self._target_location = LocationGlobalRelative(21.1600980, 72.7867741, 0)
            time.sleep(3)  # Simulate landing time
            self.location.global_relative_frame.alt = 0
            self.armed = False
            print("Simulated RTL completed - drone landed")
    
    def simple_takeoff(self, target_altitude):
        """Simulate a drone takeoff to the specified altitude."""
        if not self.armed:
            print("Cannot take off - drone is not armed")
            return
            
        print(f"Taking off to {target_altitude} meters...")
        
        # Simulate gradual climb to target altitude
        original_alt = self.location.global_relative_frame.alt
        climb_rate = 1.0  # 1 meter per second
        
        for alt in range(int(original_alt), int(target_altitude) + 1):
            self.location.global_relative_frame.alt = alt
            print(f"Altitude: {alt} meters")
            time.sleep(climb_rate)
            
        self.location.global_relative_frame.alt = target_altitude
        print(f"Reached target altitude of {target_altitude} meters")
        
        
    def download_mission(self):
        """
        Simulates downloading a mission from the drone.
        In a real scenario, this would fetch waypoints from the flight controller.
        """
        print("Downloading mission...")
        
        # Simulating mission waypoints (lat, lon, alt)
        self.mission = [
            {"seq": 0, "lat": 21.160500, "lon": 72.787000, "alt": 10},
            {"seq": 1, "lat": 21.161000, "lon": 72.787500, "alt": 20},
            {"seq": 2, "lat": 21.161500, "lon": 72.788000, "alt": 30},
        ]
        
        print("Mission downloaded successfully. Waypoints:")
        for wp in self.mission:
            print(f"  Seq {wp['seq']}: Lat={wp['lat']}, Lon={wp['lon']}, Alt={wp['alt']}")
    
    def simple_goto(self, target_location, groundspeed=None):
        """Simulate moving the drone to a target location."""
        if groundspeed:
            self.groundspeed = groundspeed
            
        self._target_location = target_location
        print(f"Moving to: Lat={target_location.lat}, Lon={target_location.lon}, Alt={target_location.alt}")
        
        # Start a thread to simulate the movement
        self._start_movement_thread()
    
    def _start_movement_thread(self):
        """Start a thread to handle simulated movement toward the target location."""
        if self._movement_thread and self._movement_active:
            self._movement_active = False
            self._movement_thread.join()
            
        '''self._movement_active = True
        self._movement_thread = threading.Thread(target=self._simulate_movement)
        self._movement_thread.daemon = True
        self._movement_thread.start()'''
    
    def _simulate_movement(self):
        """Simulate the drone moving toward the target location."""
        if not self._target_location:
            return
            
        move_rate = 0.5  # Update position 2 times per second
        speed = self.groundspeed if self.groundspeed > 0 else 5.0  # meters per second
        
        while self._movement_active and self._target_location:
            # Get current position
            current_lat = self.location.global_relative_frame.lat
            current_lon = self.location.global_relative_frame.lon
            current_alt = self.location.global_relative_frame.alt
            
            # Get target position
            target_lat = self._target_location.lat
            target_lon = self._target_location.lon
            target_alt = self._target_location.alt
            
            # Calculate distance (simplified)
            # Note: This is a very simplified distance calculation and not accurate for real GPS coords
            lat_diff = target_lat - current_lat
            lon_diff = target_lon - current_lon
            alt_diff = target_alt - current_alt
            
            # Simplified distance calculation (not geographically accurate but sufficient for simulation)
            distance = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Rough conversion to meters
            altitude_distance = abs(alt_diff)
            
            # If we're close enough to the target, end movement
            if distance < 1.0 and altitude_distance < 0.1:
                self.location.global_relative_frame.lat = target_lat
                self.location.global_relative_frame.lon = target_lon
                self.location.global_relative_frame.alt = target_alt
                print(f"Reached target location: Lat={target_lat}, Lon={target_lon}, Alt={target_alt}")
                break
                
            # Calculate movement in this time step
            move_distance = speed * move_rate  # meters to move in this step
            
            # Calculate the proportion of the total distance to move
            if distance > 0:
                proportion = min(move_distance / distance, 1.0)
                # Update position
                self.location.global_relative_frame.lat = current_lat + (lat_diff * proportion)
                self.location.global_relative_frame.lon = current_lon + (lon_diff * proportion)
            
            # Handle altitude separately
            if abs(alt_diff) > 0:
                alt_sign = 1 if alt_diff > 0 else -1
                alt_change = min(move_distance, abs(alt_diff))
                self.location.global_relative_frame.alt = current_alt + (alt_sign * alt_change)
            
            print(f"Current position: Lat={self.location.global_relative_frame.lat:.7f}, "
                  f"Lon={self.location.global_relative_frame.lon:.7f}, "
                  f"Alt={self.location.global_relative_frame.alt:.2f}")
                  
            time.sleep(move_rate)
    
    def command_long_encode(self, target_system, target_component, command, confirmation,
                           param1, param2, param3, param4, param5, param6, param7):
        """Simulate the message_factory.command_long_encode function."""
        print(f"Command sent: {command}, Params: {param1}, {param2}")
        return f"Command {command} with params {param1}, {param2}"
    
    def send_mavlink(self, msg):
        """Simulate sending a mavlink message."""
        print(f"Mavlink message sent: {msg}")
        
        # Handle specific commands
        if "183" in str(msg):  # DO_SET_SERVO command
            print("Servo actuation simulated")
    
    def flush(self):
        """Simulate the flush method."""
        pass
    
    def close(self):
        """Close the connection and stop any active threads."""
        if self._movement_active:
            self._movement_active = False
            if self._movement_thread:
                self._movement_thread.join()
        print("Fake drone connection closed")
def connect(connection_string=None, wait_ready=False):
    """
    Fake implementation of dronekit's connect function.
    Returns a FakeDrone instance instead of an actual Vehicle.
    """
    print(f"Connecting to fake drone (simulating: {connection_string})...")
    time.sleep(1)  # Simulate connection delay
    print("Fake drone connected!")
    return FakeDrone()



#-------------- FUNCTIONS

def get_location_metres(original_location, dNorth, dEast):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    print("dlat, dlon", dLat, dLon)
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
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav*c - y_uav*s
    east = x_uav*s + y_uav*c
    return(north, east)

def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)

#-------------- CONNECTION
print('Connecting...')
#vehicle = connect(args.connect, wait_ready=True, baud=921600)
vehicle = connect()
print('Home is Defined')
#vehicle.home_location = vehicle.location.global_frame

#-------------- PARAMETERS
rad_2_deg = 180.0/math.pi
deg_2_rad = 1.0/rad_2_deg

#-------------- LANDING MARKER
#--- Define Tag
id_to_find = 72
marker_size = 25 #- [cm]
freq_send = 1 #- Hz
land_alt_cm = 100.0
angle_descend = 15*deg_2_rad
land_speed_cms = 70.0

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

while True:
    if vehicle.commands.next == waypoint_count:
        marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
        if marker_found:
            vehicle.mode = VehicleMode("GUIDED")
            x_cm, y_cm = camera_to_uav(x_cm, y_cm)
            uav_location = vehicle.location.global_relative_frame
            #-- If high altitude, use baro rather than visual
            if uav_location.alt >= 5.0:
                print
                z_cm = uav_location.alt*100.0
            angle_x, angle_y = marker_position_to_angle(x_cm, y_cm, z_cm)
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
