import cv2
import numpy as np
from pyzbar.pyzbar import decode
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import argparse
import math
import os

# Option parsing ke liye command-line arguments ko set karo
parser = argparse.ArgumentParser(description='Vehicle ko control karta hai using vehicle.simple_goto aur object track karta hai.')
parser.add_argument('--connect', default='127.0.0.1:14550', help="Vehicle connection string. Agar specify nahi kiya toh SITL automatically start hoga aur use kiya jayega.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Agar connection string nahi diya toh SITL start karo
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Vehicle se connect ho rahe hain
print(f'Connecting to vehicle on: {connection_string}')
vehicle = connect(connection_string, wait_ready=True)

# Drone ko arm karo aur specified altitude tak le jao
def arm_and_takeoff(vehicle, target_altitude):
    while not vehicle.is_armable:
        print("Drone ko arm hone ka wait kar rahe hain...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Drone ko arm hone ka wait kar rahe hain...")
        time.sleep(1)

    print("Takeoff kar rahe hain!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Target altitude reach ho gaya")
            break
        time.sleep(1)

# Haversine formula distance calculate karne ke liye
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371  # Earth radius kilometers mein
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c * 1000  # Meters mein convert kar rahe hain

# QR code data ko GCS (Ground Control Station) ko bhejna
def send_qr_to_gcs(vehicle, qr_data):
    message = vehicle.message_factory.statustext_encode(
        mavutil.mavlink.MAV_SEVERITY_INFO, qr_data.encode('utf-8'))
    vehicle.send_mavlink(message)
    vehicle.flush()

# QR code ko image file se scan karna
def scan_qr_from_image_file(image_path):
    if not os.path.exists(image_path):
        print(f"Error: File {image_path} exist nahi karta.")
        return None

    image = cv2.imread(image_path)
    decoded_objects = decode(image)
    for obj in decoded_objects:
        qr_data = obj.data.decode('utf-8')
        print(f"QR Code Image se detect kiya gaya: {qr_data}")
        return str(qr_data)

    print("QR code nahi mila image mein.")
    return None

# QR code ko live webcam se scan karna
def scan_qr_code(webcam_index=0):
    cap = cv2.VideoCapture(webcam_index)
    if not cap.isOpened():
        print("Error: Webcam open nahi ho rahi.")
        return None

    print("QR code ko scan kar rahe hain...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Webcam se read nahi ho raha.")
            continue

        decoded_objects = decode(frame)
        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')
            send_qr_to_gcs(vehicle, qr_data)
            cap.release()
            cv2.destroyAllWindows()
            return str(qr_data)

        cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

def log_qr_location(vehicle, qr_data):
    """
    QR code ki location ko log karta hai jab first time detect hota hai
    Returns: Dictionary with location details
    """
    current = vehicle.location.global_relative_frame
    qr_location = {
        'lat': current.lat,
        'lon': current.lon,
        'alt': current.alt,
        'qr_data': qr_data,
        'timestamp': time.time()
    }
    print(f"QR location logged: Lat={qr_location['lat']}, Lon={qr_location['lon']}")
    return qr_location

def scan_qr_with_position(vehicle, webcam_index=0, scan_duration=30):
    """
    Drone ke movement ko compensate karne ke liye continuous scanning
    Returns: QR location dictionary ya None
    """
    start_time = time.time()
    cap = cv2.VideoCapture(webcam_index)
    if not cap.isOpened():
        print("Error: Webcam nahi khul rahi hai bhai!")
        return None

    while (time.time() - start_time) < scan_duration:
        ret, frame = cap.read()
        if not ret:
            continue

        decoded_objects = decode(frame)
        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')
            qr_location = log_qr_location(vehicle, qr_data)
            cap.release()
            cv2.destroyAllWindows()
            return qr_location

        cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

def precise_position_hold(vehicle, target_lat, target_lon, target_alt, tolerance=0.5):
    """
    Drone ko exact position par hold karta hai with specified tolerance
    """
    while True:
        current = vehicle.location.global_relative_frame
        distance = haversine_distance(current.lat, current.lon, target_lat, target_lon)
        alt_diff = abs(current.alt - target_alt)
        
        if distance < tolerance and alt_diff < tolerance:
            print("Precise position achieved!")
            return True
        
        target = LocationGlobalRelative(target_lat, target_lon, target_alt)
        vehicle.simple_goto(target)
        time.sleep(2)

def go_to_location_with_qr_scan(vehicle, latitude, longitude, altitude, initial_qr_data):
    """
    Two-pass QR scanning approach:
    1. Pehle scan karke location log karta hai
    2. Phir exact location par vapas jaake verify karta hai
    """
    # First pass - QR detection and location logging
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)
    
    qr_location = None
    while True:
        current = vehicle.location.global_relative_frame
        distance = haversine_distance(current.lat, current.lon, latitude, longitude)
        
        if distance < 2.0:  # 2 meter ke range mein scanning shuru
            print("First pass: QR code ko dhund rahe hain...")
            qr_location = scan_qr_with_orientation(vehicle)
            if qr_location:
                break
        
        if distance < 0.5:  # Agar bahut paas aa gaye aur QR nahi mila
            print("QR not found in first pass!")
            return False
            
        time.sleep(1)

    # Second pass - Return to exact QR location for verification
    print("Second pass: Exact location par vapas ja rahe hain verification ke liye")
    precise_position_hold(vehicle, qr_location['lat'], qr_location['lon'], qr_location['alt'])
    
    # Final verification
    print("Final QR verification kar rahe hain...")
    verification_result = scan_qr_with_orientation(vehicle, scan_duration=10)
    if verification_result and verification_result['qr_data'] == initial_qr_data:
        print("QR code successfully verified at exact location!")
        return True
    
    print("QR verification failed in second pass!")
    return False

# Servo ko control karna
def set_servo(vehicle, servo_number, pwm_value):
    pwm_value_int = int(pwm_value)
    msg = vehicle.message_factory.command_long_encode(
        0, 0, 
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_number,
        pwm_value_int,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Payload drop karne ka mechanism
def drop_payload():
    print("Payload drop kar rahe hain...")
    for _ in range(3):
        set_servo(vehicle, 6, 1100)
    time.sleep(30)
    print("Payload drop ho gaya!")
    set_servo(vehicle, 6, 1500)

# Main mission ko run karna
def main():
    try:
        arm_and_takeoff(vehicle, 10)
        print("Initial QR code scan kar rahe hain:")
        initial_qr_data = scan_qr_from_image_file("/home/bodhini/Documents/QrCode/QR.png")
        if not initial_qr_data:
            print("Launch site par QR nahi mila! Mission abort kar rahe hain.")
            return 

        target_lat = -35.36255470
        target_lon = 149.16386005
        qr_verified = go_to_location_with_qr_scan(vehicle, target_lat, target_lon, 10, initial_qr_data)

        if qr_verified:
            current = vehicle.location.global_relative_frame
            precise_position_hold(vehicle, current.lat, current.lon, 1)  # Payload drop ke liye low altitude
            drop_payload()
        else:
            print("QR verification fail ho gaya! Payload drop nahi karenge.")

    except Exception as e:
        print(f"Ek error aayi hai: {e}")

    finally:
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")
        vehicle.close()

if __name__ == "__main__":
    main()

def detect_qr_orientation(frame):
    """
    Detects QR code orientation using the finder patterns (position detection patterns)
    Returns: angle in degrees and finder pattern coordinates
    """
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Threshold the image
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find the three finder patterns (squares with inner squares)
    finder_patterns = []
    for contour in contours:
        # Approximate the contour to a polygon
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        
        # Check if it's a square
        if len(approx) == 4:
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            
            # Check aspect ratio
            if ar >= 0.9 and ar <= 1.1:
                finder_patterns.append((x + w//2, y + h//2))
                
    # We need exactly 3 finder patterns
    if len(finder_patterns) == 3:
        # Sort patterns by Y coordinate to find the top patterns
        sorted_patterns = sorted(finder_patterns, key=lambda x: x[1])
        top_patterns = sorted_patterns[:2]
        
        # Calculate angle between top patterns
        dx = top_patterns[1][0] - top_patterns[0][0]
        dy = top_patterns[1][1] - top_patterns[0][1]
        angle = math.degrees(math.atan2(dy, dx))
        
        return angle, finder_patterns
    
    return None, None

def set_drone_attitude(vehicle, desired_angle):
    """
    Sets the drone's yaw to match the QR code orientation
    """
    # Convert angle to positive yaw (0-360 degrees)
    yaw = desired_angle % 360
    
    # Create the CONDITION_YAW command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,       # confirmation
        yaw,     # param 1 - target angle
        25,      # param 2 - angular speed
        1,       # param 3 - direction (-1: ccw, 1: cw)
        1,       # param 4 - relative offset (0: absolute, 1: relative)
        0, 0, 0  # param 5-7 not used
    )
    
    # Send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def scan_qr_with_orientation(vehicle, webcam_index=0, scan_duration=30):
    """
    Modified QR scanning function that also aligns drone with QR orientation
    """
    start_time = time.time()
    cap = cv2.VideoCapture(webcam_index)
    if not cap.isOpened():
        print("Error: Webcam nahi khul rahi hai bhai!")
        return None

    while (time.time() - start_time) < scan_duration:
        ret, frame = cap.read()
        if not ret:
            continue

        # Detect QR orientation
        angle, finder_patterns = detect_qr_orientation(frame)
        
        if angle is not None and finder_patterns:
            # Draw finder patterns for visualization
            for (x, y) in finder_patterns:
                cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
            
            # Align drone with QR code
            set_drone_attitude(vehicle, angle)
            print(f"Aligning drone to QR orientation: {angle:.2f} degrees")

        decoded_objects = decode(frame)
        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')
            qr_location = log_qr_location(vehicle, qr_data)
            cap.release()
            cv2.destroyAllWindows()
            return qr_location

        cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None