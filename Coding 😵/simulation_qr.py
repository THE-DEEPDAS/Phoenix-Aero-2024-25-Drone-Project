# Import required libraries for drone control, timing, and computer vision
from dronekit import connect, VehicleMode, LocationGlobalRelative  # Drone control library
import time  # For timing and delays
from pyzbar.pyzbar import decode  # QR code detection library
import cv2  # OpenCV for image processing
from math import radians, cos  # Required for GPS adjustments 

# Connect to the vehicle
import argparse

# Setup command line argument parser
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyACM0')
args = parser.parse_args()

# Global variables
original_home = None

def initialize_vehicle(connection_string):
    """Initialize vehicle connection"""
    global original_home
    print(f'Connecting to vehicle on: {connection_string}')
    try:
        vehicle = connect(connection_string, wait_ready=True, baud=57600)  # Fixed baud for Pixhawk
        original_home = vehicle.location.global_frame
        print(f"Connected to Pixhawk. Home location: {original_home}")
        return vehicle
    except Exception as e:
        print(f"Connection failed: {str(e)}")
        raise

# Define the expected QR code data for verification
expected_qr_code_data = "https://qrfy.io/lIHOlbM_zI"

def arm_and_takeoff(vehicle, aTargetAltitude):
    """Function to arm the drone and take off to a specified altitude"""
    print("Basic pre-arm checks...")
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude:.2f}m")
        if current_altitude >= aTargetAltitude * 0.95:  
            print("Reached target altitude")
            break
        time.sleep(1)

def wait_for_arrival(vehicle, target_location, tolerance=0.00005, timeout=60):
    """Function to wait until drone reaches target location within tolerance"""
    start_time = time.time()
    
    while True:
        current_lat = vehicle.location.global_relative_frame.lat
        current_lon = vehicle.location.global_relative_frame.lon

        distance_lat = abs(current_lat - target_location.lat)
        distance_lon = abs(current_lon - target_location.lon)

        print(f"Current position -> Lat: {current_lat:.6f}, Lon: {current_lon:.6f}")
        
        if distance_lat < tolerance and distance_lon < tolerance:
            print("Arrived at target location.")
            break

        if time.time() - start_time > timeout:
            print("Timeout reached. Landing at current location.")
            vehicle.mode = VehicleMode("LAND")
            break

        time.sleep(2)

def descend_and_hover(vehicle, target_altitude):
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

def precision_landing(vehicle, camera):
    """Execute precision landing using QR code visual feedback"""
    print("Starting precision landing sequence...")
    start_time = time.time()
    qr_found = False
    
    while vehicle.location.global_relative_frame.alt > 0.3:  # Until 30cm above ground
        ret, frame = camera.read()
        if not ret:
            continue
            
        found, position = detect_qr_position(frame)
        if found:
            qr_found = True
            offset_x, offset_y, qr_size = position
            
            move_north = -offset_y * 0.01  
            move_east = offset_x * 0.01
            
            current_pos = vehicle.location.global_relative_frame
            
            target_lat = current_pos.lat + (move_north / 111111)  
            target_lon = current_pos.lon + (move_east / (111111 * cos(radians(current_pos.lat))))
            
            target_alt = max(current_pos.alt * 0.8, 0.3)
            
            target = LocationGlobalRelative(target_lat, target_lon, target_alt)
            vehicle.simple_goto(target)
            print(f"Adjusting position: N: {move_north:.2f}m, E: {move_east:.2f}m, Alt: {target_alt:.2f}m")
            
            time.sleep(1)
        else:
            print("Searching for QR code...")
            time.sleep(0.1)
        
        if time.time() - start_time > 30 and not qr_found:
            print("QR detection timeout - No QR code found")
            break
    
    print("Final landing phase")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(1)
    print("Landed successfully.")
    return qr_found

def detect_qr_position(frame):
    """Detect QR code position in frame and adjust orientation"""
    detected = decode(frame)
    for obj in detected:
        qr_data = obj.data.decode('utf-8')
        if qr_data == expected_qr_code_data:
            points = obj.polygon
            if len(points) == 4:
                qr_x = sum(p.x for p in points) // 4
                qr_y = sum(p.y for p in points) // 4
                height, width = frame.shape[:2]
                center_x, center_y = width // 2, height // 2
                offset_x = qr_x - center_x
                offset_y = qr_y - center_y
                qr_size = abs((points[0].x - points[2].x) * (points[0].y - points[2].y))
                return True, (offset_x, offset_y, qr_size)
    return False, None

def verify_qr_code(frame, gui_callback=None):
    """Verify QR code and notify GUI"""
    detected = decode(frame)
    for obj in detected:
        qr_data = obj.data.decode('utf-8')
        print(f"Detected QR Code: {qr_data}")
        if qr_data == expected_qr_code_data:
            if gui_callback:
                gui_callback(f"QR Status: Verified\nData: {qr_data}")
            return True, obj.polygon
    return False, None

def adjust_and_descend_to_qr(vehicle, camera, target_altitude=1.0, gui_callback=None):
    """Adjusts drone position based on QR code position"""
    print("Starting QR detection and pose estimation...")
    timeout = 60
    start_time = time.time()
    qr_detected_once = False

    while True:
        ret, frame = camera.read()
        if not ret:
            print("Failed to capture frame")
            continue

        # Verify QR code and get position
        qr_found, points = verify_qr_code(frame, gui_callback)
        
        if qr_found and points and len(points) == 4:
            qr_detected_once = True
            
            # Calculate center position
            cx = sum(p.x for p in points) // 4
            cy = sum(p.y for p in points) // 4
            frame_center_x = frame.shape[1] // 2
            frame_center_y = frame.shape[0] // 2
            
            # Calculate offsets
            offset_x = cx - frame_center_x
            offset_y = cy - frame_center_y
            
            # Calculate pose adjustments
            movement_lat = vehicle.location.global_relative_frame.lat
            movement_lon = vehicle.location.global_relative_frame.lon
            
            # Fine-tune position
            adjustment_factor = 0.000001
            movement_lat -= offset_y * adjustment_factor
            movement_lon += offset_x * adjustment_factor
            
            # Move drone while maintaining visual on QR
            current_altitude = vehicle.location.global_relative_frame.alt
            descent_step = 0.3
            target_location = LocationGlobalRelative(
                movement_lat, 
                movement_lon, 
                max(current_altitude - descent_step, target_altitude)
            )
            vehicle.simple_goto(target_location)
            
            # Log adjustments
            print(f"Pose Estimation - X:{offset_x} Y:{offset_y} Alt:{current_altitude:.2f}")
            
            # Check if positioned correctly
            if abs(offset_x) < 10 and abs(offset_y) < 10:
                if gui_callback:
                    gui_callback("QR Status: Centered & Ready for Landing")
                print("Position locked - Ready for precision landing")
                return True

        elif qr_detected_once:
            # Continue descent if QR was seen before
            current_altitude = vehicle.location.global_relative_frame.alt
            if current_altitude > target_altitude:
                vehicle.simple_goto(LocationGlobalRelative(
                    vehicle.location.global_relative_frame.lat,
                    vehicle.location.global_relative_frame.lon,
                    max(current_altitude - 0.3, target_altitude)
                ))
                time.sleep(1)
            else:
                return True

        if time.time() - start_time > timeout:
            if gui_callback:
                gui_callback("QR Status: Detection Timeout")
            print("QR detection timeout")
            break

    return False

def actuate_servo(vehicle, channel, pwm_value):
    """Actuate the servo to drop the payload."""
    print(f"Setting servo on channel {channel} to PWM {pwm_value}.")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        183,  # command ID, DO_SET_SERVO
        0,  # confirmation
        channel,  # servo number
        pwm_value,  # PWM value
        0, 0, 0, 0, 0  # unused parameters
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def actuate_servo_and_drop(vehicle, channel=6, pwm_value=1100):
    """Actuate servo to drop payload with confirmation"""
    print("Initiating payload drop...")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,        # target system, target component
        183,         # command ID (DO_SET_SERVO)
        0,           # confirmation
        channel,     # servo number
        pwm_value,   # PWM value
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    time.sleep(2)  # Wait for servo actuation
    return True

def run_complete_mission(target_lat, target_lon, target_alt, connection_string, camera, gui_callback=None):
    """Run the complete mission sequence with GUI updates"""
    vehicle = None
    try:
        # Ensure all parameters are proper type
        target_lat = float(target_lat)
        target_lon = float(target_lon)
        target_alt = float(target_alt)
        
        print(f"Starting mission with parameters:")
        print(f"Latitude: {target_lat}")
        print(f"Longitude: {target_lon}")
        print(f"Altitude: {target_alt}")
        print(f"Connection: {connection_string}")
        
        vehicle = initialize_vehicle(connection_string)
        print("Starting mission...")
        
        # Takeoff and navigation
        arm_and_takeoff(vehicle, float(target_alt))
        vehicle.airspeed = 2
        
        # Navigate to target
        target_location = LocationGlobalRelative(float(target_lat), float(target_lon), float(target_alt))
        vehicle.simple_goto(target_location)
        wait_for_arrival(vehicle, target_location)
        
        # Descend and verify QR
        descend_and_hover(vehicle, 3.0)
        qr_verified = False
        
        # QR code detection with pose estimation and GUI updates
        if adjust_and_descend_to_qr(vehicle, camera, target_altitude=1.0, gui_callback=gui_callback):
            print("QR code verified and pose estimation completed")
            qr_verified = True
            if gui_callback:
                gui_callback("QR Status: Verified & Positioned")
        else:
            print("QR code verification failed - Proceeding with normal landing")
            if gui_callback:
                gui_callback("QR Status: Not Detected")

        # Precision landing at target
        print("Executing precision landing...")
        vehicle.mode = VehicleMode("LAND")
        while vehicle.armed:
            print(f"Landing... Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
            time.sleep(1)
        
        # Drop payload after landing
        print("Landing complete - Dropping payload")
        if actuate_servo_and_drop(vehicle):
            print("Payload dropped successfully")
            if gui_callback:
                gui_callback("Status: Payload Dropped")
        
        # Take off for RTL
        print("Taking off for RTL...")
        arm_and_takeoff(vehicle, 10)  # Safe altitude for RTL
        
        # Reset home to original and RTL
        print("Setting home location to original position...")
        vehicle.home_location = original_home
        print(f"Home location reset to: {original_home}")
        
        vehicle.mode = VehicleMode("RTL")
        while vehicle.armed:
            print(f"RTL in progress... Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
            time.sleep(1)
        
        print("Mission completed successfully")
        return True, qr_verified

    except Exception as e:
        print(f"Mission encountered an error: {str(e)}")
        return False, False
    finally:
        if vehicle:
            vehicle.close()

print("Drone control script is ready for SITL execution.")

if __name__ == "__main__":
    # Setup camera with 800x600 resolution
    cap = cv2.VideoCapture(1)  # Pi camera or USB camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
    run_complete_mission('21.1598244', '72.7863403', '10', args.connect, cap)
    cap.release()
