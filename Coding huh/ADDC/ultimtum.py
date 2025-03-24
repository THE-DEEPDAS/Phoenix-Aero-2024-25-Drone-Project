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
    Precision landing with QR tracking and fallback behavior
    """
    if not cap.isOpened():
        return False
        
    # Control parameters
    DESCENT_RATE = 0.2  # Slower descent for more stability
    POSITION_SENSITIVITY = 5.0  # Higher sensitivity for precise movements
    HOVER_TIME = 1.5  # seconds to wait before dropping payload if QR lost
    POSITION_THRESHOLD = 0.02  # Tighter position control
    
    last_qr_time = time.time()
    start_hover_time = None
    best_position = None
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
                
            # Always show camera feed with overlay
            display_frame = frame.copy()
            cv2.putText(display_frame, f"Alt: {vehicle.location.global_relative_frame.alt:.1f}m", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Precision Landing", display_frame)
            cv2.waitKey(1)
                
            decoded_objects = decode(frame)
            current_alt = vehicle.location.global_relative_frame.alt
            
            if not decoded_objects:
                time_since_last_qr = time.time() - last_qr_time
                print(f"QR not visible for {time_since_last_qr:.1f}s")
                
                if time_since_last_qr > HOVER_TIME:
                    print("QR lost - executing payload drop at current position")
                    return True  # Return to trigger payload drop
                    
                # Hover in place
                msg = vehicle.message_factory.set_position_target_local_ned_encode(
                    0, 0, 0,    # time_boot_ms, target system, target component
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,  # type_mask
                    0, 0, 0,    # x, y, z positions
                    0, 0, 0,    # x, y, z velocity
                    0, 0, 0,    # x, y, z acceleration
                    0, 0)       # yaw, yaw_rate
                vehicle.send_mavlink(msg)
                continue
                
            last_qr_time = time.time()
            qr_data = decoded_objects[0].data.decode('utf-8')
            if qr_data != initial_qr_data:
                continue
                
            # Get position and orientation
            rel_x, rel_y, qr_angle = get_qr_position_and_orientation(frame, decoded_objects)
            
            if rel_x is None:
                continue
                
            # Fix inverted directions
            forward_speed = rel_y * POSITION_SENSITIVITY  # Positive is forward
            right_speed = rel_x * POSITION_SENSITIVITY   # Positive is right
            
            # Calculate descent rate based on alignment
            alignment_error = math.sqrt(rel_x**2 + rel_y**2)
            if alignment_error < POSITION_THRESHOLD:
                vert_speed = DESCENT_RATE
                if best_position is None:
                    best_position = vehicle.location.global_relative_frame
            else:
                vert_speed = 0  # Hold altitude until aligned
            
            # Send movement command
            msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0, 0, 0,    # time_boot_ms, target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,  # type_mask
                0, 0, 0,    # x, y, z positions
                forward_speed, right_speed, vert_speed,  # x, y, z velocity
                0, 0, 0,    # x, y, z acceleration
                0, qr_angle * 0.3)  # yaw, yaw_rate
            
            vehicle.send_mavlink(msg)
            
            # Debug info
            print(f"Alt: {current_alt:.1f}m, Offset: X={rel_x:.3f}, Y={rel_y:.3f}, Error={alignment_error:.3f}")
            
            # Check for landing condition
            if current_alt <= 2.0 and alignment_error < POSITION_THRESHOLD:
                if start_hover_time is None:
                    start_hover_time = time.time()
                elif time.time() - start_hover_time >= 1.0:  # Stable for 1 second
                    print("Achieved precision position for payload drop")
                    return True
            else:
                start_hover_time = None
            
            time.sleep(0.05)  # 20Hz update rate
            
    except Exception as e:
        print(f"Error during precision landing: {e}")
        if best_position:
            print("Using best achieved position for payload drop")
            return True
        return False

def get_qr_centerline_orientation(frame, decoded_objects):
    """Calculate orientation based on QR code centerline"""
    if not decoded_objects:
        return None, None, None
    
    obj = decoded_objects[0]
    points = np.array(obj.polygon)
    
    # Get top and bottom center points
    top_points = points[points[:, 1].argsort()][:2]  # Two points with lowest y
    bottom_points = points[points[:, 1].argsort()][2:]  # Two points with highest y
    
    top_center = np.mean(top_points, axis=0)
    bottom_center = np.mean(bottom_points, axis=0)
    
    # Calculate centerline vector
    centerline = bottom_center - top_center
    centerline_angle = math.degrees(math.atan2(centerline[1], centerline[0]))
    
    # Calculate relative position from image center
    frame_height, frame_width = frame.shape[:2]
    center_point = (top_center + bottom_center) / 2
    relative_x = (center_point[0] - frame_width/2) / (frame_width/2)
    relative_y = (center_point[1] - frame_height/2) / (frame_height/2)
    
    # Draw debug visualization
    debug_frame = frame.copy()
    cv2.line(debug_frame, 
             tuple(map(int, top_center)), 
             tuple(map(int, bottom_center)), 
             (0, 255, 0), 2)
    cv2.circle(debug_frame, 
              tuple(map(int, center_point)), 
              5, (0, 0, 255), -1)
    
    # Calculate roll and pitch based on centerline
    centerline_length = np.linalg.norm(centerline)
    roll_angle = math.degrees(math.asin(centerline[0] / centerline_length))
    pitch_angle = math.degrees(math.asin(centerline[1] / centerline_length))
    
    cv2.putText(debug_frame, 
                f"Roll: {roll_angle:.1f} Pitch: {pitch_angle:.1f} Yaw: {centerline_angle:.1f}", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    cv2.imshow("QR Orientation", debug_frame)
    
    return relative_x, relative_y, roll_angle, pitch_angle, centerline_angle

def follow_qr_orientation(vehicle, initial_qr_data, cap):
    """Follow QR using centerline orientation"""
    if not cap.isOpened():
        return False
        
    # Control parameters
    DESCENT_RATE = 0.2
    POSITION_SENSITIVITY = 0.3
    ANGLE_SENSITIVITY = 0.2
    MAX_SPEED = 0.3
    QR_LOST_TIMEOUT = 1.5
    
    last_qr_time = time.time()
    stable_start_time = None
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        decoded_objects = decode(frame)
        current_alt = vehicle.location.global_relative_frame.alt
        
        if not decoded_objects:
            time_since_qr = time.time() - last_qr_time
            print(f"QR lost for {time_since_qr:.1f}s")
            if time_since_qr > QR_LOST_TIMEOUT and current_alt <= 2.0:
                return True
            
            # Hover in place
            vehicle.send_mavlink(
                vehicle.message_factory.set_attitude_target_encode(
                    0, 0, 0, 0b00000000,
                    [0, 0, 0, 0.5],  # Level attitude
                    0, 0, 0, 0.5))
            continue
        
        last_qr_time = time.time()
        qr_data = decoded_objects[0].data.decode('utf-8')
        if qr_data != initial_qr_data:
            continue
        
        # Get centerline-based orientation
        rel_x, rel_y, roll, pitch, yaw = get_qr_centerline_orientation(frame, decoded_objects)
        if rel_x is None:
            continue
        
        # Calculate control inputs
        roll_correction = np.clip(roll * ANGLE_SENSITIVITY, -MAX_SPEED, MAX_SPEED)
        pitch_correction = np.clip(-pitch * ANGLE_SENSITIVITY, -MAX_SPEED, MAX_SPEED)
        yaw_correction = np.clip(yaw * ANGLE_SENSITIVITY, -MAX_SPEED, MAX_SPEED)
        
        # Position corrections
        lateral_correction = np.clip(-rel_x * POSITION_SENSITIVITY, -MAX_SPEED, MAX_SPEED)
        forward_correction = np.clip(-rel_y * POSITION_SENSITIVITY, -MAX_SPEED, MAX_SPEED)
        
        # Combine corrections
        target_roll = roll_correction + lateral_correction
        target_pitch = pitch_correction + forward_correction
        target_yaw = yaw_correction
        
        # Calculate descent
        vertical_speed = DESCENT_RATE if current_alt > 2.0 else 0
        
        # Send attitude command
        msg = vehicle.message_factory.set_attitude_target_encode(
            0, 0, 0,
            0b00000000,
            [target_roll, target_pitch, target_yaw, 0.5],
            0, 0, vertical_speed,
            0.5)
        vehicle.send_mavlink(msg)
        
        # Debug info
        print(f"Alt: {current_alt:.1f}m | Corrections - R:{roll_correction:.2f} P:{pitch_correction:.2f} Y:{yaw_correction:.2f}")
        
        # Check if stable at target height
        alignment_error = math.sqrt(rel_x**2 + rel_y**2)
        if current_alt <= 2.0 and alignment_error < 0.05:
            if stable_start_time is None:
                stable_start_time = time.time()
            elif time.time() - stable_start_time > 1.0:
                print("Stable at drop position")
                return True
        else:
            stable_start_time = None
        
        time.sleep(0.05)

def get_qr_centerline(frame, decoded_objects):
    """Get QR code centerline and calculate orientation"""
    if not decoded_objects:
        return None, None, None, None
    
    obj = decoded_objects[0]
    points = np.array([[p.x, p.y] for p in obj.polygon])
    
    # Sort points by y-coordinate
    top_points = points[points[:, 1].argsort()][:2]
    bottom_points = points[points[:, 1].argsort()][2:]
    
    # Get center points
    top_center = np.mean(top_points, axis=0)
    bottom_center = np.mean(bottom_points, axis=0)
    qr_center = (top_center + bottom_center) / 2
    
    # Calculate centerline vector
    centerline = bottom_center - top_center
    
    # Calculate angles
    frame_height, frame_width = frame.shape[:2]
    
    # Draw visualization
    debug_frame = frame.copy()
    # Draw centerline
    cv2.line(debug_frame, 
             tuple(map(int, top_center)), 
             tuple(map(int, bottom_center)), 
             (0, 255, 0), 2)
    # Draw center point
    cv2.circle(debug_frame, 
               tuple(map(int, qr_center)), 
               5, (0, 0, 255), -1)
    
    # Calculate control angles
    dx = centerline[0]
    dy = centerline[1]
    length = np.linalg.norm(centerline)
    
    # Roll: side tilt (x deviation)
    roll_angle = math.degrees(math.atan2(dx, length))
    
    # Pitch: forward tilt (y deviation)
    pitch_angle = math.degrees(math.atan2(dy, length))
    
    # Yaw: rotation around vertical axis
    yaw_angle = math.degrees(math.atan2(dx, dy))
    
    # Position relative to frame center
    rel_x = (qr_center[0] - frame_width/2) / (frame_width/2)
    rel_y = (qr_center[1] - frame_height/2) / (frame_height/2)
    
    # Draw telemetry
    cv2.putText(debug_frame, 
                f"Roll: {roll_angle:.1f}° Pitch: {pitch_angle:.1f}° Yaw: {yaw_angle:.1f}°",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(debug_frame,
                f"X: {rel_x:.2f} Y: {rel_y:.2f}", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    cv2.imshow("QR Tracking", debug_frame)
    cv2.waitKey(1)
    
    return rel_x, rel_y, (roll_angle, pitch_angle, yaw_angle)

def follow_centerline(vehicle, initial_qr_data, cap):
    """Simple centerline-based QR following"""
    DESCENT_RATE = 0.2
    MAX_SPEED = 0.3
    LOST_TIMEOUT = 1.5
    
    last_qr_time = time.time()
    start_stable_time = None
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        decoded_objects = decode(frame)
        current_alt = vehicle.location.global_relative_frame.alt
        
        if not decoded_objects:
            time_since_qr = time.time() - last_qr_time
            print(f"QR lost for {time_since_qr:.1f}s")
            
            if time_since_qr > LOST_TIMEOUT and current_alt <= 2.0:
                print("QR lost - dropping payload")
                return True
                
            # Hover in place
            msg = vehicle.message_factory.set_attitude_target_encode(
                0, 0, 0,
                0b00000000,
                [0, 0, 0, 0.5],  # Level attitude
                0, 0, 0,
                0.5)
            vehicle.send_mavlink(msg)
            continue
        
        last_qr_time = time.time()
        qr_data = decoded_objects[0].data.decode('utf-8')
        if qr_data != initial_qr_data:
            continue
        
        rel_x, rel_y, angles = get_qr_centerline(frame, decoded_objects)
        if rel_x is None:
            continue
            
        roll_angle, pitch_angle, yaw_angle = angles
        
        # Convert angles to normalized control inputs (-1 to 1)
        roll_correction = np.clip(roll_angle / 45.0, -MAX_SPEED, MAX_SPEED)
        pitch_correction = np.clip(-pitch_angle / 45.0, -MAX_SPEED, MAX_SPEED)
        yaw_correction = np.clip(yaw_angle / 90.0, -MAX_SPEED, MAX_SPEED)
        
        # Descend if above 2m
        vertical_speed = DESCENT_RATE if current_alt > 2.0 else 0
        
        # Send attitude command
        msg = vehicle.message_factory.set_attitude_target_encode(
            0, 0, 0,
            0b00000000,
            [roll_correction, pitch_correction, yaw_correction, 0.5],
            0, 0, vertical_speed,
            0.5)
        vehicle.send_mavlink(msg)
        
        # Check if stable at target height
        alignment_error = math.sqrt(rel_x**2 + rel_y**2)
        if current_alt <= 2.0 and alignment_error < 0.1:
            if start_stable_time is None:
                start_stable_time = time.time()
            elif time.time() - start_stable_time > 1.0:
                print("Stable at drop position")
                return True
        else:
            start_stable_time = None
        
        time.sleep(0.05)

class MissionControl:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")

    def follow_qr_orientation(self, vehicle, initial_qr_data):
        """Track QR and match drone orientation"""
        DESCENT_RATE = 0.2
        ORIENTATION_SENSITIVITY = 0.5
        POSITION_THRESHOLD = 0.03
        QR_LOST_TIMEOUT = 1.5
        MAX_SPEED = 0.5  # m/s maximum movement speed
        
        last_qr_time = time.time()
        stable_start_time = None
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            # Always show camera feed
            current_alt = vehicle.location.global_relative_frame.alt
            cv2.putText(frame, f"ALT: {current_alt:.1f}m", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("QR Tracking", frame)
            cv2.waitKey(1)
            
            decoded_objects = decode(frame)
            
            if not decoded_objects:
                time_since_qr = time.time() - last_qr_time
                print(f"No QR visible for {time_since_qr:.1f}s")
                
                if time_since_qr > QR_LOST_TIMEOUT:
                    print("QR lost - dropping payload")
                    return True
                
                # Hold position
                msg = vehicle.message_factory.set_attitude_target_encode(
                    0, 0, 0,
                    0b00000000,
                    [0, 0, 0, 0.5],  # Hold level
                    0, 0, 0,
                    0.5)
                vehicle.send_mavlink(msg)
                continue
            
            last_qr_time = time.time()
            qr_data = decoded_objects[0].data.decode('utf-8')
            if qr_data != initial_qr_data:
                continue
            
            rel_x, rel_y, qr_angle = get_qr_position_and_orientation(frame, decoded_objects)
            trans_vec, roll, pitch, yaw = get_qr_3d_orientation(frame, decoded_objects)
            
            if rel_x is None or trans_vec is None:
                continue
            
            # Limit movement speeds
            target_roll = np.clip(rel_x * ORIENTATION_SENSITIVITY, -MAX_SPEED, MAX_SPEED)
            target_pitch = np.clip(-rel_y * ORIENTATION_SENSITIVITY, -MAX_SPEED, MAX_SPEED)
            target_yaw = np.clip(math.radians(qr_angle) * ORIENTATION_SENSITIVITY, -MAX_SPEED, MAX_SPEED)
            
            alignment_error = math.sqrt(rel_x**2 + rel_y**2)
            
            # Only descend if well aligned
            if alignment_error < POSITION_THRESHOLD and current_alt > 2.0:
                vertical_velocity = DESCENT_RATE
            else:
                vertical_velocity = 0
            
            # Send attitude command
            msg = vehicle.message_factory.set_attitude_target_encode(
                0, 0, 0,
                0b00000000,
                [target_roll, target_pitch, target_yaw, 0.5],
                0, 0, vertical_velocity,
                0.5)
            
            vehicle.send_mavlink(msg)
            
            print(f"Alt: {current_alt:.1f}m | Error: {alignment_error:.3f}")
            print(f"Roll: {math.degrees(target_roll):.1f}° Pitch: {math.degrees(target_pitch):.1f}° Yaw: {math.degrees(target_yaw):.1f}°")
            
            if current_alt <= 2.0 and alignment_error < POSITION_THRESHOLD:
                if stable_start_time is None:
                    stable_start_time = time.time()
                elif time.time() - stable_start_time > 1.0:
                    print("Stable at target height")
                    return True
            else:
                stable_start_time = None
            
            time.sleep(0.05)

    def go_to_location_with_qr_scan(self, vehicle, latitude, longitude, altitude, initial_qr_data):
        target_location = LocationGlobalRelative(latitude, longitude, altitude)
        vehicle.simple_goto(target_location)
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            cv2.imshow("QR Search", frame)
            cv2.waitKey(1)
            
            distance = haversine_distance(
                vehicle.location.global_relative_frame.lat,
                vehicle.location.global_relative_frame.lon,
                latitude, longitude
            )
            
            decoded_objects = decode(frame)
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                if qr_data == initial_qr_data:
                    print("QR detected - following orientation")
                    return self.follow_qr_orientation(vehicle, initial_qr_data)
            
            if distance < 1:
                print("Searching for QR")
                # Execute search pattern
                for angle in [0, 90, 180, 270]:
                    vehicle.simple_goto(target_location)
                    msg = vehicle.message_factory.command_long_encode(
                        0, 0,
                        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                        0, angle, 45, 1, 0, 0, 0, 0)
                    vehicle.send_mavlink(msg)
                    time.sleep(2)
            
            time.sleep(0.1)
        
        return False

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    try:
        mission = MissionControl()
        arm_and_takeoff(vehicle, 10)
        
        initial_qr_data = scan_qr_from_image_file("/home/bodhini/Documents/QrCode/QR.png")
        if not initial_qr_data:
            print("Launch site QR not found. Aborting.")
            return 

        target_lat = -35.36255470  
        target_lon = 149.16386005 
        
        qr_verified = mission.go_to_location_with_qr_scan(
            vehicle, target_lat, target_lon, 10, initial_qr_data)

        if qr_verified:
            drop_payload()
        else:
            print("QR verification failed")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        mission.cleanup()
        vehicle.mode = VehicleMode("RTL")
        vehicle.close()

if __name__ == "__main__":
    main()