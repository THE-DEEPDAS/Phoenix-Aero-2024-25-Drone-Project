from dronekit import VehicleMode, LocationGlobalRelative
from pyzbar.pyzbar import decode
import time
import cv2
import signal
import sys
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
            lat_diff = target_lat - current_lat
            lon_diff = target_lon - current_lon
            alt_diff = target_alt - current_alt
            
            # Simplified distance calculation
            distance = math.sqrt(lat_diff*2 + lon_diff*2) * 111000  # Rough conversion to meters
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

# Extract expected QR code data from image
qr_image_path = "qr.jpeg"

def extract_qr_code_data(image_path):
    """
    Extracts QR code data from the given image.
    """
    image = cv2.imread(qr_image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found: {qr_image_path}")
    decoded_objects = decode(image)
    if not decoded_objects:
        raise ValueError("No QR code found in the provided image.")
    for obj in decoded_objects:
        qr_data = obj.data.decode('utf-8')
        print(f"QR Code Content: {qr_data}")
    return decoded_objects[0].data.decode('utf-8')

def initialize_camera():
    """Initialize and return camera capture object with error handling."""
    max_attempts = 3
    for attempt in range(max_attempts):
        try:
            print(f"Attempt {attempt+1}/{max_attempts} to initialize camera...")
            gst_pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=BGR ! videoconvert ! appsink"
            cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            
            # Verify camera is working by reading a test frame
            ret, test_frame = cap.read()
            if not ret or test_frame is None:
                print("Camera opened but failed to read frame. Retrying...")
                cap.release()
                time.sleep(2)  # Wait before retry
                continue
                
            print("Camera initialized successfully")
            return cap
        except Exception as e:
            print(f"Camera initialization error: {e}")
            time.sleep(2)  # Wait before retry
            
    print("Failed to initialize camera after multiple attempts")
    return None

try:
    expected_qr_code_data = extract_qr_code_data(qr_image_path)
    print(f"Expected QR Code Data: {expected_qr_code_data}")
except Exception as e:
    print(f"Error: {e}")
    # For testing, provide a default value if the QR image is not found
    expected_qr_code_data = "test_qr_code"
    print(f"Using default QR code data for testing: {expected_qr_code_data}")

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

def detect_qr_code(timeout=30):
    """
    Scans for QR code using the camera.
    Returns True if the expected QR code is detected, False otherwise.
    """
    max_retries = 3  # Maximum retries for reopening the camera
    retry_delay = 2  # Delay in seconds between retries

    for attempt in range(max_retries):
        cap = initialize_camera()
        if cap is None:
            print(f"Camera initialization failed (attempt {attempt + 1}/{max_retries}). Retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)
            continue

        print("Opening camera for QR code detection...")
        start_time = time.time()
        global running
        running = True

        try:
            while running:
                # Check for timeout
                if time.time() - start_time > timeout:
                    print("Timeout: No QR code detected after 30 seconds.")
                    break

                ret, frame = cap.read()
                if not ret or frame is None:
                    print("Failed to capture frame from camera.")
                    time.sleep(1)
                    continue

                # Decode QR codes in the frame
                detected_objects = decode(frame)
                for obj in detected_objects:
                    qr_data = obj.data.decode('utf-8')
                    print(f"Detected QR Code: {qr_data}")
                    if qr_data == expected_qr_code_data:
                        print("Expected QR code detected!")
                        cap.release()
                        cv2.destroyAllWindows()
                        return True

                # Optional: Display the camera feed for debugging
                cv2.imshow('Camera', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                time.sleep(0.1)  # Small delay to prevent high CPU usage
        except Exception as e:
            print(f"Error in QR detection: {e}")
        finally:
            print("Attempting to close the camera...")
            cap.release()
            cv2.destroyAllWindows()
            print("Camera closed successfully.")

        print(f"Retrying QR code detection (attempt {attempt + 1}/{max_retries})...")
        time.sleep(retry_delay)

    print("Failed to detect QR code after multiple attempts.")
    return False  # No expected QR code detected

def descend_to_altitude(target_altitude):
    """
    Descend the drone to the specified target altitude.
    """
    print(f"Descending to {target_altitude} meters...")
    
    # Create a target location at the current lat/lon but at the new altitude
    target_location = LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        target_altitude
    )
    vehicle.simple_goto(target_location)
    
    # Wait for the drone to reach the target altitude
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude:.2f}")
        if current_altitude <= target_altitude + 0.1:
            print(f"Reached target altitude of {target_altitude} meters")
            break
        time.sleep(1)
    
    return True

# Main script
if __name__ == "__main__":
    # Connect to the fake drone
    vehicle = connect()
    
    # Take off to 3 meters
    arm_and_takeoff(3)
    print("Take off complete")
    
    # Wait for the drone to stabilize
    time.sleep(2)
    
    # Detect QR code
    print("Starting QR code detection...")
    qr_detected = detect_qr_code(timeout=30)
    print("hello")
    
    if qr_detected:
        print("QR code detected! Descending to 1 meter...")
        descend_to_altitude(1)
        
        # Hold position briefly
        time.sleep(3)
        
        # Land
        print("Landing...")
        vehicle.mode = VehicleMode("LAND")
        
        # Wait for landing to complete
        while vehicle.location.global_relative_frame.alt > 0.1:
            print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
            time.sleep(1)
        
        print("Landed. Disarming...")
        vehicle.armed = False
    else:
        print("QR code not detected. Returning to launch...")
        vehicle.mode = VehicleMode("RTL")
        
        # Monitor altitude during RTL
        while vehicle.location.global_relative_frame.alt > 0.1:
            print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
            time.sleep(1)
        
        print("Landed. Disarming...")
        vehicle.armed = False
    
    # Close the vehicle
    vehicle.close()