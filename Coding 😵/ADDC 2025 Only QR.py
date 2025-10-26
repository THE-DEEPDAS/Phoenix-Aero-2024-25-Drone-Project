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
    # Create location object at current lat/lon but new altitude
    location = LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        target_altitude
    )
    vehicle.simple_goto(location)

    # Monitor descent
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude:.2f}m")
        if current_altitude <= target_altitude + 0.1:
            print("Hovering at target altitude.")
            break
        time.sleep(1)

    # Check for QR code but continue regardless
    if detect_qr_code():
        print("Expected QR Code detected. Proceeding...")
    else:
        print("No expected QR code detected. Continuing mission...")

def detect_qr_code():
    """Function to detect and validate QR code"""
    cap = cv2.VideoCapture(0)  # Initialize camera
    print("Opening camera for QR code detection...")

    timeout = 30  # Detection timeout in seconds
    start_time = time.time()
    qr_detected = False

    # Detection loop
    while time.time() - start_time < timeout:
        ret, frame = cap.read()  # Read frame from camera
        if not ret:
            print("Failed to capture frame from camera.")
            break

        # Attempt to detect QR codes in frame
        detected_objects = decode(frame)
        for obj in detected_objects:
            qr_data = obj.data.decode('utf-8')
            print(f"Detected QR Code: {qr_data}")
            if qr_data == expected_qr_code_data:
                print("Expected QR code detected!")
                qr_detected = True
                break

        # Display camera feed (debug)
        cv2.imshow('Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    return qr_detected

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

# Main mission execution
arm_and_takeoff(8)
print("Takeoff complete.")
vehicle.airspeed = 1.5

# Move towards the target location and wait for arrival
target_location = LocationGlobalRelative(21.1598244, 72.7863403, 10)
vehicle.simple_goto(target_location, groundspeed=2)
print("Moving towards target location...")
wait_for_arrival(target_location)

# Descend and hover
descend_and_hover(1.0)

# Actuate servo
print("Actuating servo...")
actuate_servo(vehicle, 6, 1100)
time.sleep(5)

# Return to Launch
print("Returning to Launch (RTL)...")
vehicle.mode = VehicleMode("RTL")

while vehicle.location.global_relative_frame.alt > 0.1:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
    time.sleep(1)

print("Landed. Disarming...")
vehicle.armed = False
vehicle.close()