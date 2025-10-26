from dronekit import connect, VehicleMode, LocationGlobalRelative
from pyzbar.pyzbar import decode
import time
import subprocess
import cv2
import numpy as np

# Extract expected QR code data from image
qr_image_path = "WhatsApp Image 2025-06-03 at 5.56.32 PM.jpeg"
def extract_qr_code_data(image_path):
    """
    Extracts QR code data from the given image.
    """
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found: {image_path}")
    decoded_objects = decode(image) #ketla QR che te find kare
    if not decoded_objects:
        raise ValueError("No QR code found in the provided image.")
    for obj in decoded_objects:
        qr_data = obj.data.decode('utf-8') # utf-8 ek encoding che jenathi bytes ne string ma convert kariye che
        print(f"QR Code Content: {qr_data}")
    return decoded_objects[0].data.decode('utf-8')

try:
    expected_qr_code_data = extract_qr_code_data(qr_image_path)
    print(f"Expected QR Code Data: {expected_qr_code_data}")
except Exception as e:
    print(f"Error: {e}")
    exit(1)

# Connect to the vehicle
vehicle = connect('127.0.0.1:14550', wait_ready=True)

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
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt #relative to the ground level at the drone's takeoff location
        print(f"Altitude: {current_altitude:.2f}")
        if current_altitude >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def actuate_servo(vehicle, channel, pwm_value):
    print(f"Setting servo on channel {channel} to PWM {pwm_value}.")
    msg = vehicle.message_factory.command_long_encode( # mavlink message banavo 
        0, 0,  # target system, target component, drone itself
        183,  # command ID, DO_SET_SERVO
        0,  # confirmation nahi joitu
        channel,  # servo number
        pwm_value,  # PWM value
        0, 0, 0, 0, 0  # unused parameters
    )
    vehicle.send_mavlink(msg)
    vehicle.flush() # flush the message queue to ensure the command is sent immediately

def verify_qr_code_data(qr_data):
    """
    Verifies if the detected QR code data matches the expected QR code data.
    """
    return qr_data == expected_qr_code_data

def start_camera_preview():
    """
    Starts the webcam preview using OpenCV
    Returns the video capture object
    """
    print("Starting camera preview...")
    cap = cv2.VideoCapture(0)  # 0 is usually the default webcam
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")
    return cap

def stop_camera_preview(cap):
    """
    Stops the webcam preview
    """
    print("Stopping camera preview...")
    if cap and cap.isOpened():
        cap.release()
        cv2.destroyAllWindows()

def capture_frame_with_webcam():
    """
    Captures a single frame using webcam and returns it as a numpy array.
    """
    print("Capturing frame using webcam...")
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("Could not open webcam")
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret or frame is None:
            raise RuntimeError("Failed to capture frame from webcam")
            
        return frame
    except Exception as e:
        print(f"Error during frame capture: {e}")
        raise

def adjust_and_descend_to_qr(vehicle, target_altitude=1.0):
    """
    Adjusts the drone's position while descending to hover over the QR code.
    If QR code is not detected within 60 seconds, performs a plus maneuver.
    """
    print("Opening webcam for QR code detection...")
    timeout = 60  # 60 seconds to detect a QR code
    start_time = time.time()

    qr_detected_once = False
    cap = cv2.VideoCapture(0)  # Initialize webcam
    if not cap.isOpened():
        print("Error: Could not open webcam")
        return False

    while True:
        try:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame from webcam")
                break
                
            # Decode QR codes in the frame
            detected_objects = decode(frame)
            qr_found = False
            for obj in detected_objects:
                qr_data = obj.data.decode('utf-8')
                print(f"Detected QR Code: {qr_data}")
                if verify_qr_code_data(qr_data):
                    print("Expected QR code detected!")
                    qr_found = True
                    qr_detected_once = True
                    # Get the QR code's bounding box center
                    points = obj.polygon
                    if len(points) > 0:
                        cx = int((points[0].x + points[2].x) / 2)
                        cy = int((points[0].y + points[2].y) / 2)
                        frame_center_x = frame.shape[1] // 2
                        frame_center_y = frame.shape[0] // 2

                        # Calculate offset
                        offset_x = cx - frame_center_x
                        offset_y = cy - frame_center_y

                        # Adjust drone's position based on offset
                        movement_lat = vehicle.location.global_relative_frame.lat
                        movement_lon = vehicle.location.global_relative_frame.lon

                        # Convert pixel offsets into lat/lon adjustments (simplified)
                        adjustment_factor = 0.000001
                        movement_lat -= offset_y * adjustment_factor
                        movement_lon += offset_x * adjustment_factor

                        # Descend while correcting position
                        current_altitude = vehicle.location.global_relative_frame.alt
                        descent_step = 0.3  # 30 cm per second
                        target_location = LocationGlobalRelative(
                            movement_lat, movement_lon, max(current_altitude - descent_step, target_altitude)
                        )
                        vehicle.simple_goto(target_location)
                        print(f"Adjusting position and descending: Offset X={offset_x}, Offset Y={offset_y}, Location={current_altitude:.2f}, {movement_lat:.9f}, {movement_lon:.9f}")

                        # Check if QR code is centered and at target altitude
                        if abs(offset_x) < 10 and abs(offset_y) < 10 and current_altitude <= target_altitude + 0.1:
                            print("QR code centered and target altitude reached. Hovering...")
                            cap.release()  # Release the webcam
                            return True

            if not qr_found and qr_detected_once:
                print("QR code disappeared. Continuing descent to fallback altitude...")
                current_altitude = vehicle.location.global_relative_frame.alt
                descent_step = 0.3
                if current_altitude > target_altitude:
                    target_location = LocationGlobalRelative(
                        vehicle.location.global_relative_frame.lat,
                        vehicle.location.global_relative_frame.lon,
                        max(current_altitude - descent_step, target_altitude)
                    )
                    vehicle.simple_goto(target_location)
                    print(f"Descending to 1 meter: Current Altitude = {current_altitude:.2f}")
                    time.sleep(1)
                elif current_altitude <= target_altitude:
                    print("Reached fallback altitude of 1 meter.")
                    cap.release()  # Release the webcam
                    return True

            # Check if timeout has been reached
            if time.time() - start_time > timeout:
                print("Timeout: No QR code detected after 60 seconds. Performing plus maneuver...")
                cap.release()  # Release the webcam

                # Perform plus maneuver
                maneuver_distance = 5.0  # 5 meters
                current_lat = vehicle.location.global_relative_frame.lat
                current_lon = vehicle.location.global_relative_frame.lon

                directions = [
                    (current_lat + 0.000045, current_lon),  # North
                    (current_lat - 0.000045, current_lon),  # South
                    (current_lat, current_lon + 0.000045),  # East
                    (current_lat, current_lon - 0.000045)   # West
                ]

                for lat, lon in directions:
                    target_location = LocationGlobalRelative(lat, lon, target_altitude)
                    print(f"Moving to: {lat:.9f}, {lon:.9f}, Altitude: {target_altitude:.2f}")
                    vehicle.simple_goto(target_location)
                    time.sleep(5)  # Wait for the drone to reach the location

                # Final payload drop
                final_lat = current_lat + 0.000045  # Offset by 5 meters North
                final_lon = current_lon
                target_location = LocationGlobalRelative(final_lat, final_lon, target_altitude)
                print(f"Dropping payload at: {final_lat:.9f}, {final_lon:.9f}, Altitude: {target_altitude:.2f}")
                actuate_servo(vehicle, 6, 1100)  # Example servo actuation
                return False

        except Exception as e:
            print(f"Error in QR detection loop: {e}")
            break

    cap.release()  # Make sure to release the webcam
    print("QR code adjustment failed or not detected.")
    return False




# Main script
arm_and_takeoff(8)
print("Take off complete")
vehicle.airspeed = 3

# Move towards the target location
target_location = LocationGlobalRelative(-35.36326170, 149.16462709, 10) #  latitude, longitude, and altitude
vehicle.simple_goto(target_location, groundspeed=50)
print("Moving towards target location...")

if adjust_and_descend_to_qr(vehicle):
    print("Position adjusted and descended over QR code.")
else:
    print("Failed to adjust position or detect QR code.")
    
target_location = LocationGlobalRelative(-35.36326170, 149.16462709, 1)
target_altitude = 1
descent_step = 0.3  # 30 cm per second
while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    if current_altitude > target_altitude:
        target_location = LocationGlobalRelative(
                    vehicle.location.global_relative_frame.lat,
                    vehicle.location.global_relative_frame.lon,
                    max(current_altitude - descent_step, target_altitude)
                )                
        vehicle.simple_goto(target_location)
        print(f"Descending to 1 meter: Current Altitude = {current_altitude:.2f}")
        time.sleep(1)
        continue
    elif current_altitude <= target_altitude:
        print("Reached fallback altitude of 1 meter.")
        break
 
time.sleep(15)   
# Actuate servo
print("Actuating servo...")
actuate_servo(vehicle, 6, 1100)
time.sleep(20)

# Return to Launch
print("Returning to Launch (RTL)...")
vehicle.mode = VehicleMode("RTL")

# Monitor altitude during RTL
while vehicle.location.global_relative_frame.alt > 0.1:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
    time.sleep(0.5)

print("Landed Back Safely. Disarming...")
vehicle.armed = False
vehicle.close()