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

def get_qr_position(frame, decoded_objects):
    if not decoded_objects:
        return None
    
    obj = decoded_objects[0]
    points = obj.polygon
    if len(points) != 4:
        return None
        
    # Calculate center of QR code
    center_x = sum(point.x for point in points) / 4
    center_y = sum(point.y for point in points) / 4
    
    frame_height, frame_width = frame.shape[:2]
    relative_x = (center_x - frame_width/2) / frame_width
    relative_y = (center_y - frame_height/2) / frame_height
    
    return relative_x, relative_y

def track_and_descend(vehicle, initial_altitude, target_altitude=2):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot open webcam")
        return False

    descent_rate = 0.5  # meters per second
    last_qr_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        decoded_objects = decode(frame)
        qr_position = get_qr_position(frame, decoded_objects)
        
        current_altitude = vehicle.location.global_relative_frame.alt
        
        if qr_position:
            last_qr_time = time.time()
            relative_x, relative_y = qr_position
            
            # Convert relative positions to movement commands
            forward_speed = relative_y * 2  # Scale factor for forward/backward
            right_speed = relative_x * 2    # Scale factor for left/right
            
            # Create velocity vector (North, East, Down, in m/s)
            velocity_x = forward_speed
            velocity_y = right_speed
            velocity_z = descent_rate
            
            # Send velocity command to vehicle
            msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,  # type_mask (only speeds enabled)
                0, 0, 0,  # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not used)
                0, 0)     # yaw, yaw_rate (not used)
            
            vehicle.send_mavlink(msg)
            vehicle.flush()
            
            # Check if reached target altitude
            if current_altitude <= target_altitude:
                cap.release()
                cv2.destroyAllWindows()
                return True
        else:
            # If QR not seen for 3 seconds, hover
            if time.time() - last_qr_time > 3:
                msg = vehicle.message_factory.set_position_target_local_ned_encode(
                    0, 0, 0,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0)
                vehicle.send_mavlink(msg)
                vehicle.flush()

        cv2.imshow("QR Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
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

# Target location par jana aur QR scan karna
def go_to_location_with_qr_scan(vehicle, latitude, longitude, altitude, initial_qr_data):
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)

    qr_verified = False
    while True:
        current_location = vehicle.location.global_relative_frame
        current_lat = current_location.lat
        current_lon = current_location.lon

        distance = haversine_distance(current_lat, current_lon, latitude, longitude)
        print(f"Target se distance: {distance:.2f} meters")
        print("QR code ko scan kar rahe hain target location ke paas...")
        if distance < 5:  # When within 5 meters of target
            print("Starting QR tracking and descent...")
            if track_and_descend(vehicle, altitude, 2):
                print("Successfully tracked and descended to QR code")
                qr_verified = True
                break

        if distance < 1:
            print("Target location reached but QR tracking failed")
            break

        time.sleep(20)

    return qr_verified

# Main mission ko run karna
def main():
    try:
        arm_and_takeoff(vehicle, 10)
        print("Initial QR code scan karo:")
        initial_qr_data = scan_qr_from_image_file("/home/bodhini/Documents/QrCode/QR.png")
        if not initial_qr_data:
            print("Launch site par QR code nahi mila. Mission abort kar rahe hain.")
            return 

        target_lat = -35.36255470  
        target_lon = 149.16386005 
        qr_verified = go_to_location_with_qr_scan(vehicle, target_lat, target_lon, 10, initial_qr_data)

        if qr_verified:
            drop_payload()
        else:
            print("QR Code verification fail ho gayi. Payload drop nahi hoga.")

    except Exception as e:
        print(f"Ek error aayi hai: {e}")

    finally:
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")
        vehicle.close()

if __name__ == "__main__":
    main()