# Import required libraries for drone control, timing, and computer vision
from dronekit import connect, VehicleMode, LocationGlobalRelative  # Drone control library
import time  # For timing and delays
from pyzbar.pyzbar import decode  # QR code detection library
import cv2  # OpenCV for image processing

# Connect to the vehicle
import argparse

# Setup command line argument parser
parser = argparse.ArgumentParser()  # Create argument parser object
parser.add_argument('--connect', default='/dev/ttyACM0')  # Add connection argument with default USB port
args = parser.parse_args()  # Parse the arguments
vehicle = connect(args.connect, wait_ready=True)  # Connect to the drone
# Define the expected QR code data for verification
expected_qr_code_data = "https://images.app.goo.gl/sJGSui5jnby8THM48"

def arm_and_takeoff(aTargetAltitude):
    """Function to arm the drone and take off to a specified altitude"""
    print("Basic pre-arm checks")
    # Wait for vehicle to be armable
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    # Switch to GUIDED mode and arm the vehicle
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    # Wait for arming to complete
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    # Command the drone to take off
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    # Monitor the altitude during takeoff
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude:.2f}m")
        if current_altitude >= aTargetAltitude * 0.95:  # 95% of target altitude
            print("Reached target altitude")
            break
        time.sleep(1)

def wait_for_arrival(target_location, tolerance=0.00005, timeout=60):
    """Function to wait until drone reaches target location within tolerance"""
    start_time = time.time()
    
    while True:
        # Get current position
        current_lat = vehicle.location.global_relative_frame.lat
        current_lon = vehicle.location.global_relative_frame.lon
        # Calculate distance to target
        distance_lat = abs(current_lat - target_location.lat)
        distance_lon = abs(current_lon - target_location.lon)

        # Print current position for monitoring
        print(f"Current position -> Lat: {current_lat:.6f}, Lon: {current_lon:.6f}")
        time.sleep(1)

        # Check if within tolerance
        if distance_lat < tolerance and distance_lon < tolerance:
            print("Arrived at target location.")
            break

        time.sleep(2)

def descend_and_hover(target_altitude):
    """Function to descend to specified altitude and hover"""
    print(f"Descending to {target_altitude} meters...")
    location = LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        target_altitude
    )
    vehicle.simple_goto(location)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude:.2f}m")
        if current_altitude <= target_altitude + 0.1:
            print("Hovering at target altitude.")
            break
        time.sleep(1)

def actuate_servo(vehicle, channel, pwm_value):
    """Function to control servo for payload release"""
    print(f"Setting servo on channel {channel} to PWM {pwm_value}.")
    # Create MAVLink message for servo control
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        183,  # MAVLink command ID for DO_SET_SERVO
        0, 
        channel,
        pwm_value,
        0, 0, 0, 0, 0
    )
    # Send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def detect_qr_position(frame):
    """Detect QR code and return its position data"""
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2
    
    detected = decode(frame)
    for obj in detected:
        qr_data = obj.data.decode('utf-8')
        if qr_data == expected_qr_code_data:
            points = obj.polygon
            if len(points) == 4:
                qr_x = sum(p.x for p in points) // 4
                qr_y = sum(p.y for p in points) // 4
                offset_x = qr_x - center_x
                offset_y = qr_y - center_y
                qr_size = abs((points[0].x - points[2].x) * (points[0].y - points[2].y))
                return True, (offset_x, offset_y, qr_size)
    return False, None

def precision_landing(vehicle, camera):
    """Execute precision landing using QR code visual feedback"""
    print("Starting precision landing sequence...")
    start_time = time.time()
    qr_found = False
    
    while vehicle.location.global_relative_frame.alt > 0.3:  # Until 30cm above ground
        # Check for timeout
        if time.time() - start_time > 30 and not qr_found:
            print("QR detection timeout - No QR code found")
            return False
            
        ret, frame = camera.read()
        if not ret:
            continue
            
        found, position = detect_qr_position(frame)
        if found:
            qr_found = True
            offset_x, offset_y, qr_size = position
            
            # Calculate movement adjustments (pixels to meters conversion)
            move_north = -offset_y * 0.01  # Convert pixels to meters
            move_east = offset_x * 0.01
            
            # Get current position
            current_pos = vehicle.location.global_relative_frame
            
            # Calculate new position
            target_lat = current_pos.lat + (move_north / 111111)  # Convert meters to degrees
            target_lon = current_pos.lon + (move_east / (111111 * cos(radians(current_pos.lat))))
            
            # Calculate descent rate based on QR size
            # Larger QR size means we're closer, so descend faster
            target_alt = max(current_pos.alt * 0.8, 0.3)
            
            # Move to new position
            target = LocationGlobalRelative(target_lat, target_lon, target_alt)
            vehicle.simple_goto(target)
            print(f"Adjusting position: N: {move_north:.2f}m, E: {move_east:.2f}m, Alt: {target_alt:.2f}m")
            
            time.sleep(1)  # Allow time for movement
        else:
            print("Searching for QR code...")
            time.sleep(0.1)
    
    if qr_found:
        print("Final landing phase")
        vehicle.mode = VehicleMode("LAND")
        return True
    return False

# Remove the main execution code from here as it will be controlled by GUI
if __name__ == "__main__":
    print("This script is meant to be run through the GUI interface")