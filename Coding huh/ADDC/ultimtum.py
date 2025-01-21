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

def get_qr_position_and_orientation(frame, decoded_objects):
    if not decoded_objects:
        return None, None, None
    
    obj = decoded_objects[0]
    points = obj.polygon
    if len(points) != 4:
        return None, None, None
        
    # Calculate center
    center_x = sum(point.x for point in points) / 4
    center_y = sum(point.y for point in points) / 4
    
    # Calculate orientation
    # Using first two points to determine angle
    dx = points[1].x - points[0].x
    dy = points[1].y - points[0].y
    angle = math.degrees(math.atan2(dy, dx))
    
    frame_height, frame_width = frame.shape[:2]
    relative_x = (center_x - frame_width/2) / (frame_width/2)  
    relative_y = (center_y - frame_height/2) / (frame_height/2)
    
    # More precise angle calculation using all corners
    angles = []
    for i in range(4):
        next_i = (i + 1) % 4
        dx = points[next_i].x - points[i].x
        dy = points[next_i].y - points[i].y
        angles.append(math.degrees(math.atan2(dy, dx)))
    angle = sum(angles) / 4  # Average angle for better stability
    
    return relative_x, relative_y, angle

def get_qr_3d_orientation(frame, decoded_objects):
    """Calculate 3D orientation of QR code relative to camera"""
    if not decoded_objects:
        return None, None, None, None
    
    obj = decoded_objects[0]
    points = np.float32([point for point in obj.polygon])
    
    # QR code size in meters (adjust based on your QR size)
    QR_SIZE = 0.2  
    
    # 3D model points of QR code corners
    model_points = np.float32([
        [0.0, 0.0, 0.0],
        [QR_SIZE, 0.0, 0.0],
        [QR_SIZE, QR_SIZE, 0.0],
        [0.0, QR_SIZE, 0.0]
    ])
    
    # Camera matrix (approximate values, should be calibrated)
    frame_height, frame_width = frame.shape[:2]
    focal_length = frame_width
    center = (frame_width / 2, frame_height / 2)
    camera_matrix = np.array([
        [focal_length, 0, center[0]],
        [0, focal_length, center[1]],
        [0, 0, 1]
    ], dtype="double")
    
    # Assuming no lens distortion
    dist_coeffs = np.zeros((4, 1))
    
    # SolvePnP to find rotation and translation vectors
    success, rotation_vector, translation_vector = cv2.solvePnP(
        model_points, points, camera_matrix, dist_coeffs)
    
    if not success:
        return None, None, None, None
    
    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    
    # Calculate Euler angles from rotation matrix
    sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x_angle = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y_angle = math.atan2(-rotation_matrix[2, 0], sy)
        z_angle = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x_angle = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y_angle = math.atan2(-rotation_matrix[2, 0], sy)
        z_angle = 0
    
    return translation_vector, x_angle, y_angle, z_angle

def track_qr_with_precision(vehicle, initial_qr_data, cap, target_altitude=1.5):
    """
    More sensitive QR tracking with continuous camera operation
    Returns: (success, new_lat, new_lon) tuple
    """
    if not cap.isOpened():
        return False, None, None
    
    last_qr_time = time.time()
    start_altitude = vehicle.location.global_relative_frame.alt
    
    # Tuned control parameters
    VERTICAL_SPEED = 0.2  # Slower descent
    POSITION_SENSITIVITY = 3.0  # Increased position sensitivity
    ROTATION_SENSITIVITY = 0.2  # Increased rotation sensitivity
    POSITION_THRESHOLD = 0.05  # Tighter position tolerance
    
    try:
        ret, frame = cap.read()
        if not ret:
            return False, None, None
            
        decoded_objects = decode(frame)
        qr_data = None
        if decoded_objects:
            qr_data = decoded_objects[0].data.decode('utf-8')
        
        if qr_data == initial_qr_data:
            last_qr_time = time.time()
            rel_x, rel_y, qr_angle = get_qr_position_and_orientation(frame, decoded_objects)
            translation_vector, x_angle, y_angle, z_angle = get_qr_3d_orientation(frame, decoded_objects)
            
            if rel_x is not None:
                # More sensitive movement calculations
                forward_speed = -rel_y * POSITION_SENSITIVITY
                right_speed = -rel_x * POSITION_SENSITIVITY
                yaw_rate = qr_angle * ROTATION_SENSITIVITY
                
                current_alt = vehicle.location.global_relative_frame.alt
                vert_speed = VERTICAL_SPEED if current_alt > target_altitude else 0
                
                # Send movement command with increased precision
                msg = vehicle.message_factory.set_position_target_local_ned_encode(
                    0, 0, 0,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,
                    0, 0, 0,
                    forward_speed, right_speed, vert_speed,
                    0, 0, 0,
                    0, yaw_rate)
                
                vehicle.send_mavlink(msg)
                vehicle.flush()
                
                # Return current position if well-aligned
                if (abs(rel_x) < POSITION_THRESHOLD and 
                    abs(rel_y) < POSITION_THRESHOLD and 
                    current_alt <= target_altitude):
                    current_pos = vehicle.location.global_relative_frame
                    return True, current_pos.lat, current_pos.lon
                    
                return False, None, None
    
    except Exception as e:
        print(f"Tracking error: {e}")
        return False, None, None
    
    return False, None, None

def track_and_descend_continuously(vehicle, initial_qr_data, target_altitude=1.5):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return False
    
    last_qr_time = time.time()
    start_altitude = vehicle.location.global_relative_frame.alt
    descent_rate = 0.3  # meters per second
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            
            decoded_objects = decode(frame)
            qr_data = None
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
            
            if qr_data == initial_qr_data:
                last_qr_time = time.time()
                rel_x, rel_y, qr_angle = get_qr_position_and_orientation(frame, decoded_objects)
                
                if rel_x is not None:
                    # Calculate movement speeds
                    forward_speed = rel_y * 2
                    right_speed = rel_x * 2
                    
                    # Calculate required yaw adjustment
                    yaw_rate = qr_angle * 0.1  # Scale factor for rotation
                    
                    current_alt = vehicle.location.global_relative_frame.alt
                    if current_alt > target_altitude:
                        vert_speed = descent_rate
                    else:
                        vert_speed = 0
                        print("Reached target altitude, maintaining position")
                        if abs(rel_x) < 0.1 and abs(rel_y) < 0.1:  # If centered
                            cap.release()
                            cv2.destroyAllWindows()
                            return True
                    
                    # Send movement command
                    msg = vehicle.message_factory.set_position_target_local_ned_encode(
                        0, 0, 0,
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        0b0000111111000111,
                        0, 0, 0,
                        forward_speed, right_speed, vert_speed,
                        0, 0, 0,
                        0, yaw_rate)
                    
                    vehicle.send_mavlink(msg)
                    vehicle.flush()
                    
                    print(f"Tracking QR - Alt: {current_alt:.1f}m, X-off: {rel_x:.2f}, Y-off: {rel_y:.2f}, Angle: {qr_angle:.1f}")
            
            else:
                if time.time() - last_qr_time > 3:
                    print("Lost QR code - hovering")
                    # Send hover command
                    msg = vehicle.message_factory.set_position_target_local_ned_encode(
                        0, 0, 0,
                        mavutil.mavlink.MAV_FRAME_BODY_NED,
                        0b0000111111000111,
                        0, 0, 0,
                        0, 0, 0,
                        0, 0)
                    vehicle.send_mavlink(msg)
                    vehicle.flush()
            
            cv2.imshow("QR Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            time.sleep(0.1)  # Control rate
            
    except Exception as e:
        print(f"Error during tracking: {e}")
        return False
        
    finally:
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

# QR code ko continuously scan karna
def continuous_qr_scan(cap, initial_qr_data):
    """Continuously scan for QR codes and return new coordinates if matching QR found"""
    ret, frame = cap.read()
    if not ret:
        return None, None
        
    decoded_objects = decode(frame)
    for obj in decoded_objects:
        qr_data = obj.data.decode('utf-8')
        if qr_data == initial_qr_data:
            # Extract coordinates from QR data or use current drone position
            # This is a placeholder - modify based on your QR data format
            current_pos = vehicle.location.global_relative_frame
            return current_pos.lat, current_pos.lon
            
    cv2.imshow("QR Scanner", frame)
    cv2.waitKey(1)
    return None, None

def precision_qr_landing(vehicle, initial_qr_data, cap, start_altitude):
    """
    Precisely track and land on QR code while matching its orientation
    """
    if not cap.isOpened():
        return False
        
    # Control parameters
    DESCENT_RATE = 0.3  # m/s
    POSITION_SENSITIVITY = 4.0
    ROTATION_SENSITIVITY = 0.3
    ALIGNMENT_THRESHOLD = 0.03  # Stricter position tolerance
    ANGLE_THRESHOLD = 2.0  # degrees
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
                
            decoded_objects = decode(frame)
            if not decoded_objects:
                print("QR not visible - hovering")
                # Send hover command
                msg = vehicle.message_factory.set_position_target_local_ned_encode(
                    0, 0, 0,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                vehicle.send_mavlink(msg)
                continue
                
            qr_data = decoded_objects[0].data.decode('utf-8')
            if qr_data != initial_qr_data:
                continue
                
            # Get position and 3D orientation
            rel_x, rel_y, qr_angle = get_qr_position_and_orientation(frame, decoded_objects)
            trans_vec, roll, pitch, yaw = get_qr_3d_orientation(frame, decoded_objects)
            
            if rel_x is None or trans_vec is None:
                continue
                
            # Calculate control inputs
            forward_speed = -rel_y * POSITION_SENSITIVITY
            right_speed = -rel_x * POSITION_SENSITIVITY
            
            # Match QR orientation
            roll_rate = -roll * ROTATION_SENSITIVITY
            pitch_rate = -pitch * ROTATION_SENSITIVITY
            yaw_rate = -yaw * ROTATION_SENSITIVITY
            
            current_alt = vehicle.location.global_relative_frame.alt
            
            # Always descend while maintaining position
            vertical_speed = DESCENT_RATE
            
            # Send movement command with attitude control
            msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0, 0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000101111000111,  # Enable attitude control
                0, 0, 0,
                forward_speed, right_speed, vertical_speed,
                0, 0, 0,
                roll_rate, pitch_rate, yaw_rate)
            
            vehicle.send_mavlink(msg)
            vehicle.flush()
            
            # Debug info
            print(f"Alt: {current_alt:.1f}m, Offsets: X={rel_x:.3f}, Y={rel_y:.3f}")
            print(f"Attitude: Roll={math.degrees(roll):.1f}°, Pitch={math.degrees(pitch):.1f}°, Yaw={math.degrees(yaw):.1f}°")
            
            # Check if we've reached target height and are aligned
            if current_alt <= 2.0:
                if (abs(rel_x) < ALIGNMENT_THRESHOLD and 
                    abs(rel_y) < ALIGNMENT_THRESHOLD and
                    abs(math.degrees(roll)) < ANGLE_THRESHOLD and
                    abs(math.degrees(pitch)) < ANGLE_THRESHOLD):
                    print("Aligned with QR code at target height")
                    return True
                    
            cv2.imshow("Precision Landing", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            time.sleep(0.05)  # 20Hz update rate
            
    except Exception as e:
        print(f"Error during precision landing: {e}")
        return False

def go_to_location_with_qr_scan(vehicle, latitude, longitude, altitude, initial_qr_data):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return False
    
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)
    
    try:
        while True:
            current_location = vehicle.location.global_relative_frame
            distance = haversine_distance(
                current_location.lat, 
                current_location.lon, 
                latitude, 
                longitude
            )
            
            # Start looking for QR as soon as possible
            decoded_objects = decode(cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2GRAY))
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                if qr_data == initial_qr_data:
                    print("QR code detected - starting precision landing")
                    if precision_qr_landing(vehicle, initial_qr_data, cap, current_location.alt):
                        return True
            
            if distance < 1:
                print("Reached target but couldn't find QR")
                break
                
            time.sleep(0.1)
            
    except Exception as e:
        print(f"Error during mission: {e}")
        return False
    finally:
        cap.release()
        cv2.destroyAllWindows()
    
    return False

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