from dronekit import VehicleMode, LocationGlobalRelative
from pyzbar.pyzbar import decode
from picamera2 import Picamera2
import time
import cv2
import signal
import sys
import numpy as np
import threading
import math

# Flag to control camera operations
running = True

# Signal handler for graceful shutdown
def signal_handler(sig, frame):
    global running
    print("Caught signal, shutting down...")
    running = False

# Register signal handlers
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class FakeDrone:
    def __init__(self, initial_lat=21.1600980, initial_lon=72.7867741, initial_alt=0.0):
        # Basic drone state
        self.is_armable = True
        self.armed = False
        self._mode = "GUIDED"
        
        # Drone location and movement
        self.location = type('Location', (), {})()
        self.location.global_relative_frame = type('GlobalRelativeFrame', (), {
            'lat': initial_lat,
            'lon': initial_lon,
            'alt': initial_alt
        })()
        
        # Drone parameters
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self._target_location = None
        self._movement_thread = None
        self._movement_active = False
        
        # Create message factory for servo control
        self.message_factory = type('MessageFactory', (), {})()
        
        print(f"Fake drone initialized at: Lat={initial_lat}, Lon={initial_lon}, Alt={initial_alt}")
    
    @property
    def mode(self):
        return self._mode
        
    @mode.setter
    def mode(self, new_mode):
        if isinstance(new_mode, str):
            self._mode = new_mode
        else:
            self._mode = new_mode.name
        print(f"Drone mode changed to: {self._mode}")
        
        if self._mode == "RTL":
            # Handle Return to Launch
            self._target_location = LocationGlobalRelative(21.1600980, 72.7867741, 10)
            self._start_movement_thread()
            time.sleep(2)  # Simulate travel time
            # Then descend to ground
            self._target_location = LocationGlobalRelative(21.1600980, 72.7867741, 0)
            time.sleep(3)  # Simulate landing time
            self.location.global_relative_frame.alt = 0
            self.armed = False
            print("Simulated RTL completed - drone landed")
    
    def simple_takeoff(self, target_altitude):
        """Simulate a drone takeoff to the specified altitude."""
        if not self.armed:
            print("Cannot take off - drone is not armed")
            return
            
        print(f"Taking off to {target_altitude} meters...")
        
        # Simulate gradual climb to target altitude
        original_alt = self.location.global_relative_frame.alt
        climb_rate = 1.0  # 1 meter per second
        
        for alt in range(int(original_alt), int(target_altitude) + 1):
            self.location.global_relative_frame.alt = alt
            print(f"Altitude: {alt} meters")
            time.sleep(climb_rate)
            
        self.location.global_relative_frame.alt = target_altitude
        print(f"Reached target altitude of {target_altitude} meters")
    
    def simple_goto(self, target_location, groundspeed=None):
        """Simulate moving the drone to a target location."""
        if groundspeed:
            self.groundspeed = groundspeed
            
        self._target_location = target_location
        print(f"Moving to: Lat={target_location.lat}, Lon={target_location.lon}, Alt={target_location.alt}")
        
        # Start a thread to simulate the movement
        self._start_movement_thread()
    
    def _start_movement_thread(self):
        """Start a thread to handle simulated movement toward the target location."""
        if self._movement_thread and self._movement_active:
            self._movement_active = False
            self._movement_thread.join()
            
        self._movement_active = True
        self._movement_thread = threading.Thread(target=self._simulate_movement)
        self._movement_thread.daemon = True
        self._movement_thread.start()
    
    def _simulate_movement(self):
        """Simulate the drone moving toward the target location."""
        if not self._target_location:
            return
            
        move_rate = 0.5  # Update position 2 times per second
        speed = self.groundspeed if self.groundspeed > 0 else 5.0  # meters per second
        
        while self._movement_active and self._target_location:
            # Get current position
            current_lat = self.location.global_relative_frame.lat
            current_lon = self.location.global_relative_frame.lon
            current_alt = self.location.global_relative_frame.alt
            
            # Get target position
            target_lat = self._target_location.lat
            target_lon = self._target_location.lon
            target_alt = self._target_location.alt
            
            # Calculate distance (simplified)
            # Note: This is a very simplified distance calculation and not accurate for real GPS coords
            lat_diff = target_lat - current_lat
            lon_diff = target_lon - current_lon
            alt_diff = target_alt - current_alt
            
            # Simplified distance calculation (not geographically accurate but sufficient for simulation)
            distance = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Rough conversion to meters
            altitude_distance = abs(alt_diff)
            
            # If we're close enough to the target, end movement
            if distance < 1.0 and altitude_distance < 0.1:
                self.location.global_relative_frame.lat = target_lat
                self.location.global_relative_frame.lon = target_lon
                self.location.global_relative_frame.alt = target_alt
                print(f"Reached target location: Lat={target_lat}, Lon={target_lon}, Alt={target_alt}")
                break
                
            # Calculate movement in this time step
            move_distance = speed * move_rate  # meters to move in this step
            
            # Calculate the proportion of the total distance to move
            if distance > 0:
                proportion = min(move_distance / distance, 1.0)
                # Update position
                self.location.global_relative_frame.lat = current_lat + (lat_diff * proportion)
                self.location.global_relative_frame.lon = current_lon + (lon_diff * proportion)
            
            # Handle altitude separately
            if abs(alt_diff) > 0:
                alt_sign = 1 if alt_diff > 0 else -1
                alt_change = min(move_distance, abs(alt_diff))
                self.location.global_relative_frame.alt = current_alt + (alt_sign * alt_change)
            
            print(f"Current position: Lat={self.location.global_relative_frame.lat:.7f}, "
                  f"Lon={self.location.global_relative_frame.lon:.7f}, "
                  f"Alt={self.location.global_relative_frame.alt:.2f}")
                  
            time.sleep(move_rate)
    
    def command_long_encode(self, target_system, target_component, command, confirmation,
                           param1, param2, param3, param4, param5, param6, param7):
        """Simulate the message_factory.command_long_encode function."""
        print(f"Command sent: {command}, Params: {param1}, {param2}")
        return f"Command {command} with params {param1}, {param2}"
    
    def send_mavlink(self, msg):
        """Simulate sending a mavlink message."""
        print(f"Mavlink message sent: {msg}")
        
        # Handle specific commands
        if "183" in str(msg):  # DO_SET_SERVO command
            print("Servo actuation simulated")
    
    def flush(self):
        """Simulate the flush method."""
        pass
    
    def close(self):
        """Close the connection and stop any active threads."""
        if self._movement_active:
            self._movement_active = False
            if self._movement_thread:
                self._movement_thread.join()
        print("Fake drone connection closed")

def connect(connection_string=None, wait_ready=False):
    """
    Fake implementation of dronekit's connect function.
    Returns a FakeDrone instance instead of an actual Vehicle.
    """
    print(f"Connecting to fake drone (simulating: {connection_string})...")
    time.sleep(1)  # Simulate connection delay
    print("Fake drone connected!")
    return FakeDrone()

def setup_camera():
    """Initialize the Picamera2 camera"""
    try:
        # Initialize the camera
        picam2 = Picamera2()
        
        # Configure the camera
        config = picam2.create_preview_configuration(main={"size": (640, 480)})
        picam2.configure(config)
        
        # Start the camera
        picam2.start()
        time.sleep(2)  # Allow time for camera to start
        
        print("Camera initialized successfully")
        return picam2
    except Exception as e:
        print(f"Camera initialization error: {e}")
        return None

# Extract expected QR code data from image
qr_image_path = "qr.jpeg"

def extract_qr_code_data(image_path):
    """
    Extracts QR code data from the given image.
    """
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found: {image_path}")
    decoded_objects = decode(image)
    if not decoded_objects:
        raise ValueError("No QR code found in the provided image.")
    for obj in decoded_objects:
        qr_data = obj.data.decode('utf-8')
        print(f"QR Code Content: {qr_data}")
    return decoded_objects[0].data.decode('utf-8')

try:
    expected_qr_code_data = extract_qr_code_data(qr_image_path)
    print(f"Expected QR Code Data: {expected_qr_code_data}")
except Exception as e:
    print(f"Error: {e}")
    exit(1)

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
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude:.2f}")
        if current_altitude >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def actuate_servo(vehicle, channel, pwm_value):
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

def detect_qr_and_descend(vehicle, target_altitude=2.0):
    """
    Simple QR detection and descent function without complex adjustments.
    Returns True if QR was detected, False otherwise.
    """
    camera = setup_camera()
    if camera is None:
        print("Failed to initialize camera. Cannot proceed with QR detection.")
        return False
    
    print("Starting QR code detection...")
    
    global running
    running = True
    qr_detected = False
    timeout = 30  # 30 seconds timeout
    start_time = time.time()
    
    # Create window for display
    cv2.namedWindow("QR Detection", cv2.WINDOW_NORMAL)
    
    while running and time.time() - start_time < timeout:
        try:
            # Capture frame
            frame = camera.capture_array()
            
            # Convert to grayscale for better QR detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect QR codes in the frame
            qr_codes = decode(gray)
            
            # Display status on frame
            status_text = "Scanning for QR codes..."
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Process detected QR codes
            for qr in qr_codes:
                # Decode the QR data
                qr_data = qr.data.decode('utf-8')
                
                # Compare with reference data
                if qr_data == expected_qr_code_data:
                    match_text = f"Match found! Data: {qr_data}"
                    cv2.putText(frame, match_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    print(f"Match found! QR code data: {qr_data}")
                    qr_detected = True
                    
                    # Draw rectangle around the QR code with green color (match)
                    points = qr.polygon
                    if len(points) == 4:
                        hull = np.array([points], dtype=np.int32)
                        cv2.polylines(frame, hull, True, (0, 255, 0), 3)
                    
                    # Begin descent to target altitude
                    print("QR code detected. Beginning descent...")
                    current_altitude = vehicle.location.global_relative_frame.alt
                    target_location = LocationGlobalRelative(
                        vehicle.location.global_relative_frame.lat,
                        vehicle.location.global_relative_frame.lon,
                        target_altitude
                    )
                    vehicle.simple_goto(target_location)
                    
                else:
                    # Draw rectangle around non-matching QR code with blue color
                    points = qr.polygon
                    if len(points) == 4:
                        hull = np.array([points], dtype=np.int32)
                        cv2.polylines(frame, hull, True, (255, 0, 0), 3)
                    
                    # Display the QR data on frame
                    data_text = f"Data: {qr_data} (not a match)"
                    cv2.putText(frame, data_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # Display the frame
            cv2.imshow("QR Detection", frame)
            
            # Check for quit command
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            # Check if we've reached target altitude after QR detection
            if qr_detected and abs(vehicle.location.global_relative_frame.alt - target_altitude) < 0.1:
                print(f"Reached target altitude of {target_altitude}m after QR detection")
                break
                
            # Small delay to reduce CPU usage
            time.sleep(0.05)
                
        except Exception as e:
            print(f"Error processing frame: {e}")
            time.sleep(0.5)
    
    # Clean up
    camera.close()
    cv2.destroyAllWindows()
    
    if not qr_detected and time.time() - start_time >= timeout:
        print("Timeout: No QR code detected within the allocated time.")
    
    return qr_detected

# Main script
if __name__ == "__main__":
    # Connect to the fake drone
    vehicle = connect()
    
    # Take off to initial altitude
    arm_and_takeoff(3)
    print("Take off complete")
    vehicle.airspeed = 7

    # Move towards the target location
    target_location = LocationGlobalRelative(21.1600980, 72.7867741, 3)
    vehicle.simple_goto(target_location, groundspeed=7)
    print("Moving towards target location...")
    
    # Wait to reach approximately above target location
    time.sleep(5)
    
    # Try to detect QR code and descend
    qr_detected = detect_qr_and_descend(vehicle)
    
    if not qr_detected:
        print("QR code not detected. Descending to fallback altitude anyway.")
        # Descend to target altitude even if QR not detected
        target_location = LocationGlobalRelative(
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon, 
            2.0
        )
        vehicle.simple_goto(target_location)
        
        # Wait for descent
        while vehicle.location.global_relative_frame.alt > 2.1:
            print(f"Descending: Current Altitude = {vehicle.location.global_relative_frame.alt:.2f}")
            time.sleep(1)
    
    # Wait at target altitude
    print("Hovering at target altitude...")
    time.sleep(5)
    
    # Actuate servo regardless of QR detection
    print("Actuating servo...")
    actuate_servo(vehicle, 6, 1100)
    time.sleep(3)

    # Return to Launch
    print("Returning to Launch (RTL)...")
    vehicle.mode = VehicleMode("RTL")

    # Monitor altitude during RTL
    while vehicle.location.global_relative_frame.alt > 0.1:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        time.sleep(0.5)

    print("Landed. Disarming...")
    vehicle.armed = False
    vehicle.close()