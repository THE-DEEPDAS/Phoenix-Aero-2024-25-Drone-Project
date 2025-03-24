from dronekit import connect, VehicleMode, LocationGlobalRelative
from pyzbar.pyzbar import decode
import time
import cv2
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk
import threading
import queue
from datetime import datetime

# Extract expected QR code data from image
qr_image_path = "WhatsApp Image 2025-01-22 at 7.02.01 PM.jpeg"
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

def verify_qr_code_data(qr_data):
    """
    Verifies if the detected QR code data matches the expected QR code data.
    """
    return qr_data == expected_qr_code_data

def adjust_and_descend_to_qr(vehicle, target_altitude=2.0):
    """
    Adjusts the drone's position while descending to hover over the QR code.
    """
    cap = cv2.VideoCapture(0)
    print("Opening camera for QR code detection and position adjustment...")
    
    timeout = 60  # Wait for 60 seconds to detect a QR code
    start_time = time.time()

    qr_detected_once = False  # Track if QR code was detected at least once

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame. Exiting adjustment loop.")
            break

        # Decode QR codes in the frame
        detected_objects = decode(frame)
        qr_found = False
        for obj in detected_objects:
            qr_data = obj.data.decode('utf-8')
            print(f"Detected QR Code Data: {qr_data}")  # Display information inside QR code
            if verify_qr_code_data(qr_data):
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
                    adjustment_factor = 0.000001  # Tune this factor based on your setup
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
                        cap.release()
                        cv2.destroyAllWindows()
                        return True

        if not qr_found and qr_detected_once:
            # If QR code was detected once but now it's lost
            print("QR code disappeared. Continuing descent to fallback altitude...")
            current_altitude = vehicle.location.global_relative_frame.alt
            descent_step = 0.3  # 30 cm per second
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
                cap.release()
                cv2.destroyAllWindows()
                return True

        # Show camera feed for debugging (optional)
        cv2.imshow('Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        # Check if timeout has been reached
        if time.time() - start_time > timeout:
            print("Timeout: No QR code detected after 60 seconds.")
            break
            
    cap.release()
    cv2.destroyAllWindows()
    print("QR code adjustment failed or not detected.")
    return False

class ModernButton(tk.Button):
    def __init__(self, master=None, **kwargs):
        super().__init__(master, **kwargs)
        self.configure(
            relief="flat",
            borderwidth=0,
            padx=15,
            pady=8,
            font=("Helvetica", 11, "bold"),
            cursor="hand2"
        )
        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)

    def on_enter(self, e):
        if self["state"] != "disabled":
            self.configure(bg=self.darker(self["bg"], 0.1))

    def on_leave(self, e):
        if self["state"] != "disabled":
            self.configure(bg=self.original_color)

    def configure(self, **kwargs):
        if "bg" in kwargs:
            self.original_color = kwargs["bg"]
        super().configure(**kwargs)

    @staticmethod
    def darker(color, factor):
        r, g, b = [int(color[i:i+2], 16) for i in (1, 3, 5)]
        return f"#{int(r * (1-factor)):02x}{int(g * (1-factor)):02x}{int(b * (1-factor)):02x}"

class ModernEntry(tk.Entry):
    def __init__(self, master=None, **kwargs):
        super().__init__(master, **kwargs)
        self.configure(
            relief="flat",
            borderwidth=0,
            highlightthickness=1,
            highlightbackground="#404040",
            highlightcolor="#00a8e8",
            insertwidth=1
        )

class DroneControlGUI:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.root = tk.Tk()
        self.root.title("Drone Control System")
        self.root.geometry("1280x960")
        self.root.configure(bg="#1a1a1a")
        
        # Colors
        self.colors = {
            "bg_dark": "#1a1a1a",
            "bg_medium": "#2c2c2c",
            "bg_light": "#404040",
            "text": "#ffffff",
            "accent": "#00a8e8",
            "success": "#00c853",
            "warning": "#ffd700",
            "error": "#ff4444",
            "highlight": "#00a8e8"
        }
        
        # Create message queue for thread-safe GUI updates
        self.msg_queue = queue.Queue()
        
        # Drone state variables
        self.qr_image_path = None
        self.expected_qr_code_data = None
        
        self.create_main_panel()
        self.process_message_queue()
        
    def create_main_panel(self):
        # Main container with padding
        self.main_frame = tk.Frame(self.root, bg=self.colors["bg_dark"])
        self.main_frame.pack(expand=True, fill="both", padx=30, pady=30)
        
        # Header
        header_frame = tk.Frame(self.main_frame, bg=self.colors["bg_dark"])
        header_frame.pack(fill="x", pady=(0, 20))
        
        title = tk.Label(
            header_frame,
            text="Drone Control System",
            font=("Helvetica", 28, "bold"),
            fg=self.colors["text"],
            bg=self.colors["bg_dark"]
        )
        title.pack(side="left")
        
        # Status indicator
        self.connection_status = tk.Label(
            header_frame,
            text="● Connected",
            fg=self.colors["success"],
            bg=self.colors["bg_dark"],
            font=("Helvetica", 12)
        )
        self.connection_status.pack(side="right", padx=10)
        
        # Create two-column layout
        content_frame = tk.Frame(self.main_frame, bg=self.colors["bg_dark"])
        content_frame.pack(fill="both", expand=True)
        
        # Left column (Controls)
        left_column = tk.Frame(content_frame, bg=self.colors["bg_dark"])
        left_column.pack(side="left", fill="both", expand=True, padx=(0, 15))
        
        # Controls panel
        controls_panel = tk.LabelFrame(
            left_column,
            text="Mission Controls",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Helvetica", 12, "bold"),
            padx=15,
            pady=15
        )
        controls_panel.pack(fill="x", pady=(0, 15))
        
        # QR Code selection
        qr_frame = tk.Frame(controls_panel, bg=self.colors["bg_medium"])
        qr_frame.pack(fill="x", pady=10)
        
        self.qr_code_btn = ModernButton(
            qr_frame,
            text="Select QR Code Image",
            command=self.select_qr_image,
            bg=self.colors["accent"],
            fg=self.colors["text"]
        )
        self.qr_code_btn.pack(fill="x")
        
        # Action buttons
        button_frame = tk.Frame(controls_panel, bg=self.colors["bg_medium"])
        button_frame.pack(fill="x", pady=(10, 0))
        
        self.start_mission_btn = ModernButton(
            button_frame,
            text="Start Mission",
            command=self.start_mission,
            bg=self.colors["success"],
            fg=self.colors["text"]
        )
        self.start_mission_btn.pack(side="left", expand=True, padx=(5, 0))
        
        # Telemetry panel
        telemetry_panel = tk.LabelFrame(
            left_column,
            text="Telemetry",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Helvetica", 12, "bold")
        )
        telemetry_panel.pack(fill="x")
        
        # Telemetry grid
        telemetry_grid = tk.Frame(telemetry_panel, bg=self.colors["bg_medium"], padx=15, pady=15)
        telemetry_grid.pack(fill="x")
        
        self.altitude_label = tk.Label(
            telemetry_grid,
            text="Altitude: 0.0 m",
            fg=self.colors["accent"],
            bg=self.colors["bg_medium"],
            font=("Helvetica", 12)
        )
        self.altitude_label.pack(side="left", expand=True)
        
        self.speed_label = tk.Label(
            telemetry_grid,
            text="Speed: 0.0 m/s",
            fg=self.colors["accent"],
            bg=self.colors["bg_medium"],
            font=("Helvetica", 12)
        )
        self.speed_label.pack(side="left", expand=True)
        
        # Right column (Camera and Log)
        right_column = tk.Frame(content_frame, bg=self.colors["bg_dark"])
        right_column.pack(side="left", fill="both", expand=True)
        
        # Camera feed
        camera_panel = tk.LabelFrame(
            right_column,
            text="Camera Feed",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Helvetica", 12, "bold")
        )
        camera_panel.pack(fill="both", expand=True, pady=(0, 15))
        
        self.camera_label = tk.Label(camera_panel, bg="black")
        self.camera_label.pack(fill="both", expand=True, padx=15, pady=15)
        
        # Log panel
        log_panel = tk.LabelFrame(
            right_column,
            text="Mission Log",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Helvetica", 12, "bold")
        )
        log_panel.pack(fill="both", expand=True)
        
        self.log_text = tk.Text(
            log_panel,
            height=10,
            bg=self.colors["bg_light"],
            fg=self.colors["text"],
            font=("Consolas", 10),
            padx=10,
            pady=10
        )
        self.log_text.pack(fill="both", expand=True, padx=15, pady=15)

    def select_qr_image(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.gif *.bmp")]
        )
        if file_path:
            self.qr_image_path = file_path
            self.qr_code_btn.config(text=file_path.split("/")[-1])
            self.load_qr_code()
    
    def load_qr_code(self):
        if not self.qr_image_path:
            self.log_message("No QR code image selected!")
            return
            
        try:
            qr_img = cv2.imread(self.qr_image_path)
            decoded_objects = decode(qr_img)
            
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8')
                self.expected_qr_code_data = qr_data
                self.log_message(f"QR Code loaded: {qr_data}")
            else:
                self.log_message("No QR code detected in the image!")
        except Exception as e:
            self.log_message(f"Error loading QR code: {str(e)}")
    
    def log_message(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.msg_queue.put(("log", f"[{timestamp}] {message}\n"))
    
    def process_message_queue(self):
        try:
            while True:
                msg_type, msg_data = self.msg_queue.get_nowait()
                if msg_type == "log":
                    self.log_text.insert(tk.END, msg_data)
                    self.log_text.see(tk.END)
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.process_message_queue)
    
    def start_mission(self):
        def mission_thread():
            try:
                self.log_message("Starting mission...")
                arm_and_takeoff(8)
                self.log_message("Take off complete")
                vehicle.airspeed = 3
                
                # Move towards the target location
                target_location = LocationGlobalRelative(-35.36326170, 149.16462709, 10)
                vehicle.simple_goto(target_location, groundspeed=50)
                self.log_message("Moving towards target location...")
                
                if adjust_and_descend_to_qr(vehicle):
                    self.log_message("Position adjusted and descended over QR code.")
                else:
                    self.log_message("Failed to adjust position or detect QR code.")
                
                # Actuate servo
                self.log_message("Actuating servo...")
                actuate_servo(vehicle, 6, 1100)
                time.sleep(20)
                
                # Return to Launch
                self.log_message("Returning to Launch (RTL)...")
                vehicle.mode = VehicleMode("RTL")
                
                while vehicle.location.global_relative_frame.alt > 0.1:
                    time.sleep(0.5)
                
                self.log_message("Mission completed")
            except Exception as e:
                self.log_message(f"Mission failed: {str(e)}")
        
        threading.Thread(target=mission_thread, daemon=True).start()
    
    def run(self):
        self.root.mainloop()

# Main script
if __name__ == "__main__":
    vehicle = connect('127.0.0.1:14550', wait_ready=True)
    gui = DroneControlGUI(vehicle)
    gui.run()

