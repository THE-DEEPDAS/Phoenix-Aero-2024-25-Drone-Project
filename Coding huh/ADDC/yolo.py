import cv2
import numpy as np
from pyzbar.pyzbar import decode
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import argparse
import math
import os
import torch

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
        target_qr_data = scan_qr_code()
        if target_qr_data == initial_qr_data:
            print(f"QR Code verified ho gaya target location par: {target_qr_data}")
            qr_verified = True
            current = vehicle.location.global_relative_frame
            target=LocationGlobalRelative(current.lat, current.lon, 1)
            vehicle.simple_goto(target)
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

class QRAttitudeLanding:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.cap = cv2.VideoCapture(0)
        # Load YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='qr_corners.pt')
        self.descent_rate = 0.5  # m/s
        self.last_altitude = None
        self.start_height = 10.0
        
    def detect_qr_corners(self, frame):
        """Use YOLO to detect QR code corners"""
        results = self.model(frame)
        corners = []
        
        if len(results.xyxy[0]) >= 3:  # We need at least 3 corners
            boxes = results.xyxy[0].cpu().numpy()
            # Sort corners by confidence and get top 3
            boxes = sorted(boxes, key=lambda x: x[4], reverse=True)[:3]
            corners = [(int((box[0] + box[2])/2), int((box[1] + box[3])/2)) for box in boxes]
            return corners
        return None

    def calculate_attitude_correction(self, corners):
        """Calculate required roll, pitch, yaw from corner positions"""
        if len(corners) != 3:
            return None, None, None

        # Sort corners by Y coordinate to find top two
        sorted_corners = sorted(corners, key=lambda x: x[1])
        top_two = sorted_corners[:2]
        bottom = sorted_corners[2]

        # Sort top two by X coordinate
        top_left, top_right = sorted(top_two, key=lambda x: x[0])

        # Calculate frame center
        frame_height, frame_width = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT), self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        center_x, center_y = frame_width/2, frame_height/2

        # Calculate yaw from top edge
        dx = top_right[0] - top_left[0]
        dy = top_right[1] - top_left[1]
        yaw = math.degrees(math.atan2(dy, dx))

        # Calculate roll and pitch from QR perspective
        qr_center_x = (top_left[0] + top_right[0] + bottom[0]) / 3
        qr_center_y = (top_left[1] + top_right[1] + bottom[1]) / 3
        
        # Calculate roll and pitch based on QR center offset from frame center
        roll = -math.degrees(math.atan2(qr_center_x - center_x, frame_width/2))
        pitch = math.degrees(math.atan2(qr_center_y - center_y, frame_height/2))

        return roll, pitch, yaw

    def adjust_attitude(self, roll, pitch, yaw):
        """Send attitude adjustment commands to vehicle"""
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,
            1, 1,
            0b00000111,  # Ignore rates
            [math.radians(roll), math.radians(pitch), math.radians(yaw), 0.5],
            0, 0, 0,  # Roll, pitch, yaw rates
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def controlled_descent(self):
        """Execute descent while maintaining attitude alignment"""
        print("Starting precision landing with attitude control...")
        current_alt = self.vehicle.location.global_relative_frame.alt
        
        while current_alt > 0.5:  # Stop at 0.5m
            ret, frame = self.cap.read()
            if not ret:
                continue

            # Detect QR corners
            corners = self.detect_qr_corners(frame)
            
            if corners:
                # Calculate required attitude corrections
                roll, pitch, yaw = self.calculate_attitude_correction(corners)
                
                if all(x is not None for x in [roll, pitch, yaw]):
                    # Apply attitude correction
                    self.adjust_attitude(roll, pitch, yaw)
                    
                    # Draw visualization
                    self.draw_detection(frame, corners, roll, pitch, yaw)
                    
                    # Continue descent
                    new_alt = current_alt - self.descent_rate * 0.1  # 0.1s interval
                    target = LocationGlobalRelative(
                        self.vehicle.location.global_relative_frame.lat,
                        self.vehicle.location.global_relative_frame.lon,
                        new_alt
                    )
                    self.vehicle.simple_goto(target)
            
            cv2.imshow("Precision Landing", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            current_alt = self.vehicle.location.global_relative_frame.alt
            time.sleep(0.1)

        print("Reached landing height")
        return True

    def draw_detection(self, frame, corners, roll, pitch, yaw):
        """Visualize detection and attitude information"""
        for i, corner in enumerate(corners):
            cv2.circle(frame, corner, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"C{i}", (corner[0]+10, corner[1]+10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw attitude info
        cv2.putText(frame, f"Roll: {roll:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Pitch: {pitch:.1f}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Yaw: {yaw:.1f}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    def execute_landing(self):
        """Main landing sequence"""
        try:
            if self.controlled_descent():
                drop_payload()
                print("Starting RTL...")
                self.vehicle.mode = VehicleMode("RTL")
                return True
            return False
        finally:
            self.cap.release()
            cv2.destroyAllWindows()

# Main mission ko run karna
def main():
    try:
        arm_and_takeoff(vehicle, 10)
        initial_qr_data = scan_qr_from_image_file("/home/bodhini/Documents/QrCode/QR.png")
        if not initial_qr_data:
            return

        target_lat = -35.36255470
        target_lon = 149.16386005
        
        # Go to target location
        target = LocationGlobalRelative(target_lat, target_lon, 10)
        vehicle.simple_goto(target)
        
        # Wait to reach target area
        while haversine_distance(
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon,
            target_lat, target_lon) > 2.0:
            time.sleep(1)
            
        # Start precision landing
        landing_controller = QRAttitudeLanding(vehicle)
        if landing_controller.execute_landing():
            print("Mission successful!")
        else:
            print("Landing sequence failed!")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if vehicle.mode.name != "RTL":
            vehicle.mode = VehicleMode("RTL")
        vehicle.close()

if __name__== "__main__":
    main()