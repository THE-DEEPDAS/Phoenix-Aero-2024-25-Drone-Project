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
        
   
def detect_qr_orientation(frame):
    """
    Enhanced QR code orientation detection using the three finder patterns
    Returns: angle, patterns, and identified corners (top-left, top-right, bottom-left)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find nested squares (finder patterns)
    finder_patterns = []
    
    if hierarchy is None:
        return None, None, None
        
    for i, contour in enumerate(contours):
        # Check for nested contours
        if hierarchy[0][i][2] != -1:  # Has child
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            
            if len(approx) == 4:  # Is square
                (x, y, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)
                
                if 0.8 <= ar <= 1.2 and w >= 20:  # Square with minimum size
                    # Verify inner square
                    child_idx = hierarchy[0][i][2]
                    child_contour = contours[child_idx]
                    child_peri = cv2.arcLength(child_contour, True)
                    child_approx = cv2.approxPolyDP(child_contour, 0.04 * child_peri, True)
                    
                    if len(child_approx) == 4:  # Inner square verified
                        center = (x + w//2, y + h//2)
                        finder_patterns.append((center, w))  # Store center and width
    
    if len(finder_patterns) >= 3:
        # Sort by width to get the three main finder patterns
        finder_patterns = sorted(finder_patterns, key=lambda x: x[1], reverse=True)[:3]
        centers = [p[0] for p in finder_patterns]
        
        # Sort by Y coordinate
        sorted_by_y = sorted(centers, key=lambda x: x[1])
        
        # Get top two and bottom patterns
        top_two = sorted(sorted_by_y[:2], key=lambda x: x[0])
        bottom = sorted_by_y[2]
        
        top_left, top_right = top_two
        
        # Calculate orientation angle
        dx = top_right[0] - top_left[0]
        dy = top_right[1] - top_left[1]
        angle = math.degrees(math.atan2(dy, dx))
        
        return angle, centers, (top_left, top_right, bottom)
    
    return None, None, None

def scan_qr_with_orientation():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Webcam nahi khul rahi")
        return None

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        angle, patterns, corners = detect_qr_orientation(frame)
        
        if angle is not None and patterns:
            # Draw detected patterns
            for (x, y) in patterns:
                cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
            
            # Draw connections between patterns
            if corners:
                top_left, top_right, bottom = corners
                cv2.line(frame, 
                        (int(top_left[0]), int(top_left[1])),
                        (int(top_right[0]), int(top_right[1])),
                        (0, 255, 0), 2)
                cv2.line(frame, 
                        (int(top_left[0]), int(top_left[1])),
                        (int(bottom[0]), int(bottom[1])),
                        (0, 255, 0), 2)
            
            # Align drone
            set_drone_attitude(vehicle, angle)
            print(f"Aligning drone to QR orientation: {angle:.2f} degrees")

        cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

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
            #cap.release()
            #cv2.destroyAllWindows()
            return str(qr_data)

        cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

def set_drone_attitude(vehicle, desired_angle):
    """
    Sets the drone's yaw to match the QR code orientation
    """
    yaw = desired_angle % 360
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        yaw,
        25,  # yaw speed deg/s
        1,   # direction (-1: ccw, 1: cw)
        1,   # relative offset
        0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    # Wait for the yaw to complete
    time.sleep(math.fabs(yaw) / 25 + 1)

def precise_landing_with_qr(vehicle, initial_qr_data):
    """
    Executes precise landing using QR code orientation
    """
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        return False
        
    print("Starting precise landing sequence...")
    alignment_achieved = False
    descent_started = False
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        # Get QR orientation
        angle, patterns, corners = detect_qr_orientation(frame)
        decoded_objects = decode(frame)
        
        if decoded_objects and corners:
            qr_data = decoded_objects[0].data.decode('utf-8')
            if qr_data == initial_qr_data:
                top_left, top_right, bottom = corners
                
                # Draw visualization
                if patterns:
                    for center in patterns:
                        cv2.circle(frame, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)
                    cv2.line(frame, 
                            (int(top_left[0]), int(top_left[1])),
                            (int(top_right[0]), int(top_right[1])),
                            (0, 0, 255), 2)  # Front edge in red
                
                if not alignment_achieved:
                    # Align drone with QR orientation
                    print(f"Aligning to angle: {angle:.2f}")
                    set_drone_attitude(vehicle, angle)
                    alignment_achieved = True
                    time.sleep(2)  # Wait for alignment
                
                if alignment_achieved and not descent_started:
                    print("Starting controlled descent...")
                    target_location = LocationGlobalRelative(
                        vehicle.location.global_relative_frame.lat,
                        vehicle.location.global_relative_frame.lon,
                        1.0
                    )
                    vehicle.simple_goto(target_location)
                    descent_started = True
                
                if descent_started:
                    current_alt = vehicle.location.global_relative_frame.alt
                    print(f"Current altitude: {current_alt:.1f}m")
                    if current_alt <= 1.2:
                        print("Reached payload drop height")
                        drop_payload()
                        cap.release()
                        cv2.destroyAllWindows()
                        return True
            
        cv2.imshow("Precise Landing", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return False

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
        target_qr_data = scan_qr_code()
        if target_qr_data == initial_qr_data:
            print(f"QR Code verified ho gaya target location par: {target_qr_data}")
            qr_verified = True
            scan_qr_with_orientation()
            return qr_verified
        if distance < 1:
            print("Target location reached.")
            break

        time.sleep(20)

    return qr_verified

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

def set_attitude(vehicle, roll_angle, pitch_angle, yaw_angle):
    """
    Set complete attitude of drone using roll, pitch and yaw
    All angles in degrees
    """
    # Convert to radians
    roll = math.radians(roll_angle)
    pitch = math.radians(pitch_angle)
    yaw = math.radians(yaw_angle)
    
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # target system
        1,  # target component
        0b00000111,  # type_mask (ignore rates)
        [roll, pitch, yaw, 0],  # quaternion
        0, 0, 0,  # roll, pitch, yaw rates
        0.5  # thrust (0.5 = maintain altitude)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    time.sleep(0.1)  # Small delay to allow attitude change

def calculate_3d_orientation(corners):
    """
    Calculate roll, pitch, yaw from QR finder patterns
    """
    if len(corners) != 3:
        return None, None, None
        
    top_left, top_right, bottom = corners
    
    # Calculate yaw from top edge
    dx = top_right[0] - top_left[0]
    dy = top_right[1] - top_left[1]
    yaw = math.degrees(math.atan2(dy, dx))
    
    # Calculate roll and pitch from relative positions
    # of bottom corner to top edge
    edge_center = ((top_left[0] + top_right[0])/2, (top_left[1] + top_right[1])/2)
    dx_bottom = bottom[0] - edge_center[0]
    dy_bottom = bottom[1] - edge_center[1]
    
    # Estimate roll and pitch based on perspective distortion
    roll = math.degrees(math.atan2(dx_bottom, dy_bottom))
    pitch = -math.degrees(math.atan2(dy_bottom, dx_bottom))
    
    return roll, pitch, yaw

class QRTrackingMission:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.cap = None  # Will open after takeoff
        self.initial_qr = None
        self.descent_started = False
        self.qr_tracking_active = False

    def start_camera(self):
        """Open camera after takeoff"""
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Camera initialization failed")
        print("Camera started successfully")

    def track_qr_and_adjust(self, frame):
        """Track QR and adjust drone attitude"""
        decoded = decode(frame)
        if not decoded:
            return False

        qr_data = decoded[0].data.decode('utf-8')
        if qr_data != self.initial_qr:
            return False

        angle, patterns, corners = detect_qr_orientation(frame)
        if angle is not None and patterns and corners:
            # Calculate 3D orientation from corners
            roll, pitch, yaw = calculate_3d_orientation(corners)
            if all(x is not None for x in [roll, pitch, yaw]):
                # Adjust drone attitude to maintain parallel orientation with QR
                set_attitude(self.vehicle, roll, pitch, yaw)
                
                # Visualize tracking
                self.visualize_tracking(frame, patterns, corners)
                return True
        return False

    def visualize_tracking(self, frame, patterns, corners):
        """Draw QR tracking visualization"""
        for center in patterns:
            cv2.circle(frame, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)
        top_left, top_right, bottom = corners
        cv2.line(frame, 
                (int(top_left[0]), int(top_left[1])),
                (int(top_right[0]), int(top_right[1])),
                (0, 0, 255), 2)  # Red line shows front edge
        cv2.putText(frame, "QR Tracked", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    def execute_mission(self, target_lat, target_lon):
        try:
            # Initial QR scan
            print("Starting mission...")
            self.initial_qr = scan_qr_from_image_file("/path/to/initial/qr.png")
            if not self.initial_qr:
                print("Failed to get initial QR")
                return False

            # Takeoff
            arm_and_takeoff(self.vehicle, 10)
            
            # Start camera after takeoff
            self.start_camera()
            print("Moving to target location...")
            
            # Start moving to target
            target = LocationGlobalRelative(target_lat, target_lon, 10)
            self.vehicle.simple_goto(target)

            while True:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                cv2.imshow("Mission Camera", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Check for QR anywhere during flight
                if self.track_qr_and_adjust(frame):
                    if not self.descent_started:
                        print("QR detected - Starting descent at current location")
                        self.descent_started = True
                        current = self.vehicle.location.global_relative_frame
                        
                        # Descend while maintaining position
                        target = LocationGlobalRelative(
                            current.lat, current.lon, 1.0
                        )
                        self.vehicle.simple_goto(target)
                        self.qr_tracking_active = True

                    # Continue tracking during descent
                    if self.qr_tracking_active:
                        current_alt = self.vehicle.location.global_relative_frame.alt
                        print(f"Descending - Altitude: {current_alt:.1f}m")
                        
                        if current_alt <= 1.2:
                            print("At delivery height - dropping payload")
                            drop_payload()
                            print("Starting RTL...")
                            self.vehicle.mode = VehicleMode("RTL")
                            break
                
                # If QR lost during descent
                elif self.descent_started:
                    current_alt = self.vehicle.location.global_relative_frame.alt
                    if current_alt <= 1.2:
                        print("QR lost but at delivery height - dropping payload")
                        drop_payload()
                        self.vehicle.mode = VehicleMode("RTL")
                        break

                time.sleep(0.1)

            # Keep camera feed during RTL
            while self.vehicle.mode.name == "RTL":
                ret, frame = self.cap.read()
                if ret:
                    cv2.imshow("Return to Launch", frame)
                    cv2.waitKey(1)
                time.sleep(0.1)

        except Exception as e:
            print(f"Mission Error: {e}")
            return False
        finally:
            cv2.destroyAllWindows()
            if self.cap is not None:
                self.cap.release()
        return True

def main_mission(vehicle, target_lat, target_lon):
    # Step 1: Load initial QR from image
    initial_qr_data = scan_qr_from_image_file("/path/to/initial_qr.png")
    if not initial_qr_data:
        print("Could not read initial QR from image, aborting.")
        return

    # Step 2: Arm and take off
    arm_and_takeoff(vehicle, 10)
    print("Takeoff successful, heading to target...")

    # Step 3: Travel to target location
    target_location = LocationGlobalRelative(target_lat, target_lon, 10)
    vehicle.simple_goto(target_location)

    qr_found = False
    while True:
        # Continuously check camera feed
        ret, frame = cap.read()
        if not ret:
            continue
        if detect_and_track_qr(vehicle, frame, initial_qr_data):
            # Stop, precisely align, descend, drop payload
            qr_found = True
            break

        current_location = vehicle.location.global_relative_frame
        current_lat = current_location.lat
        current_lon = current_location.lon

        distance = haversine_distance(current_lat, current_lon, target_lat, target_lon)
        print(f"Target se distance: {distance:.2f} meters")

        # If close enough to target (within 10m), look around a bit
        if distance < 10:
            # Attempt further scanning in a small search pattern
            if detect_and_track_qr(vehicle, frame, initial_qr_data):
                qr_found = True
            break

        time.sleep(0.1)

    if qr_found:
        # Step 4: Precisely align, descend, drop payload
        print("Payload dropped. Returning to launch.")
    else:
        print("QR not found. Returning to launch.")

    vehicle.mode = VehicleMode("RTL")

# Helper function to detect & track the desired QR in live feed
def detect_and_track_qr(vehicle, frame, initial_qr_data):
    decoded_objects = decode(frame)
    if decoded_objects:
        # If QR matches initial QR data
        if decoded_objects[0].data.decode('utf-8') == initial_qr_data:
            # Gradually align and descend
            angle, patterns, corners = detect_qr_orientation(frame)
            if angle is not None and patterns and corners:
                roll, pitch, yaw = calculate_3d_orientation(corners)
                if all(x is not None for x in [roll, pitch, yaw]):
                    set_attitude(vehicle, roll, pitch, yaw)
                    drop_payload()
                    return True
    return False

def main():
    try:
        mission = QRTrackingMission(vehicle)
        target_lat = -35.36255470
        target_lon = 149.16386005
        
        if mission.execute_mission(target_lat, target_lon):
            print("Mission completed successfully")
        else:
            print("Mission failed")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        vehicle.close()

if __name__ == "__main__":
    main()
