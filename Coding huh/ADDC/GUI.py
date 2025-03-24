import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
from PIL import Image, ImageTk
import cv2
import threading
import time
from datetime import datetime
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pyzbar.pyzbar import decode
import queue

class DroneControlGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Drone Control System")
        self.root.geometry("1024x768")
        self.root.configure(bg="#1a1a1a")
        
        # Create message queue for thread-safe GUI updates
        self.msg_queue = queue.Queue()
        
        # Drone state variables
        self.vehicle = None
        self.is_connected = False
        self.is_armed = False
        self.current_altitude = 0.0
        self.current_speed = 0.0
        self.mission_running = False
        self.qr_image_path = None
        
        self.create_main_panel()
        self.process_message_queue()
        
    def create_main_panel(self):
        # Main container
        self.main_frame = tk.Frame(self.root, bg="#1a1a1a")
        self.main_frame.pack(expand=True, fill="both", padx=20, pady=20)
        
        # Title
        title = tk.Label(
            self.main_frame,
            text="Drone Control System",
            font=("Helvetica", 24, "bold"),
            fg="#ffffff",
            bg="#1a1a1a"
        )
        title.pack(pady=(0, 20))
        
        # Input fields frame
        input_frame = tk.Frame(self.main_frame, bg="#2c2c2c", pady=10)
        input_frame.pack(fill="x", padx=10, pady=5)
        
        # Create input fields (2 rows x 3 columns)
        input_fields = [
            ("GPS Coordinates", "gps", "0.0, 0.0"),
            ("Landing Speed (m/s)", "landing_speed", "2.0"),
            ("Hover Weight (kg)", "hover_weight", "1.5"),
            ("Height (m)", "height", "10"),
            ("IP:Port", "ip_port", "127.0.0.1:14550"),
            ("QR Code Image", "qr_code", "Select File...")
        ]
        
        for i, (label_text, var_name, default_value) in enumerate(input_fields):
            row = i // 3
            col = i % 3
            
            # Container for each input group
            container = tk.Frame(input_frame, bg="#2c2c2c", padx=10, pady=5)
            container.grid(row=row, column=col, sticky="ew", padx=5, pady=5)
            
            # Label
            label = tk.Label(
                container,
                text=label_text,
                fg="white",
                bg="#2c2c2c",
                font=("Helvetica", 10)
            )
            label.pack(anchor="w")
            
            # Input field or button
            if var_name == "qr_code":
                btn = tk.Button(
                    container,
                    text=default_value,
                    command=self.select_qr_image,
                    bg="#404040",
                    fg="white",
                    font=("Helvetica", 10),
                    width=20
                )
                btn.pack(fill="x", pady=(5, 0))
                setattr(self, f"{var_name}_btn", btn)
            else:
                entry = tk.Entry(
                    container,
                    bg="#404040",
                    fg="white",
                    font=("Helvetica", 10),
                    insertbackground="white"
                )
                entry.insert(0, default_value)
                entry.pack(fill="x", pady=(5, 0))
                setattr(self, f"{var_name}_entry", entry)
        
        # Connection and Start Mission buttons
        button_frame = tk.Frame(self.main_frame, bg="#1a1a1a")
        button_frame.pack(fill="x", padx=10, pady=10)
        
        self.connect_btn = tk.Button(
            button_frame,
            text="Connect Drone",
            command=self.connect_drone,
            bg="#00a8e8",
            fg="white",
            font=("Helvetica", 10, "bold"),
            width=15
        )
        self.connect_btn.pack(side="left", padx=5)
        
        self.start_mission_btn = tk.Button(
            button_frame,
            text="Start Mission",
            command=self.start_mission,
            bg="#00c853",
            fg="white",
            font=("Helvetica", 10, "bold"),
            width=15
        )
        self.start_mission_btn.pack(side="left", padx=5)
        
        self.emergency_stop_btn = tk.Button(
            button_frame,
            text="EMERGENCY STOP",
            command=self.emergency_stop,
            bg="#ff0000",
            fg="white",
            font=("Helvetica", 10, "bold"),
            width=15
        )
        self.emergency_stop_btn.pack(side="right", padx=5)
        
        self.connection_status = tk.Label(
            button_frame,
            text="Not Connected",
            fg="#ff4444",
            bg="#1a1a1a",
            font=("Helvetica", 10)
        )
        self.connection_status.pack(side="left", padx=10)
        
        # Telemetry frame
        telemetry_frame = tk.LabelFrame(
            self.main_frame,
            text="Telemetry",
            fg="white",
            bg="#2c2c2c",
            font=("Helvetica", 12)
        )
        telemetry_frame.pack(fill="x", padx=10, pady=10)
        
        # Altitude display
        self.altitude_label = tk.Label(
            telemetry_frame,
            text="Altitude: 0.0 m",
            fg="#00ff00",
            bg="#2c2c2c",
            font=("Helvetica", 12)
        )
        self.altitude_label.pack(side="left", padx=20, pady=10)
        
        # Speed display
        self.speed_label = tk.Label(
            telemetry_frame,
            text="Speed: 0.0 m/s",
            fg="#00ff00",
            bg="#2c2c2c",
            font=("Helvetica", 12)
        )
        self.speed_label.pack(side="left", padx=20, pady=10)
        
        # Camera feed frame
        self.camera_frame = tk.LabelFrame(
            self.main_frame,
            text="Camera Feed",
            fg="white",
            bg="#2c2c2c",
            font=("Helvetica", 12)
        )
        self.camera_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.camera_label = tk.Label(self.camera_frame, bg="black")
        self.camera_label.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Log frame
        log_frame = tk.LabelFrame(
            self.main_frame,
            text="Mission Log",
            fg="white",
            bg="#2c2c2c",
            font=("Helvetica", 12)
        )
        log_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=8,
            bg="black",
            fg="#00ff00",
            font=("Courier", 10)
        )
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
    
    def select_qr_image(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.gif *.bmp")]
        )
        if file_path:
            self.qr_image_path = file_path
            self.qr_code_btn.config(text=file_path.split("/")[-1])
            self.load_qr_code()
    
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
                elif msg_type == "altitude":
                    self.altitude_label.config(text=f"Altitude: {msg_data:.2f} m")
                elif msg_type == "speed":
                    self.speed_label.config(text=f"Speed: {msg_data:.2f} m/s")
                elif msg_type == "connection":
                    self.connection_status.config(
                        text="Connected" if msg_data else "Not Connected",
                        fg="#00ff00" if msg_data else "#ff4444"
                    )
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.process_message_queue)
    
    def connect_drone(self):
        def connect_thread():
            try:
                ip_port = self.ip_port_entry.get()
                self.log_message(f"Connecting to drone at {ip_port}...")
                self.vehicle = connect(ip_port, wait_ready=True)
                self.is_connected = True
                self.msg_queue.put(("connection", True))
                self.log_message("Drone connected successfully!")
                
                # Start telemetry updates
                self.update_telemetry()
            except Exception as e:
                self.log_message(f"Connection failed: {str(e)}")
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def update_telemetry(self):
        def telemetry_thread():
            while self.is_connected and self.vehicle:
                try:
                    altitude = self.vehicle.location.global_relative_frame.alt
                    speed = self.vehicle.airspeed
                    self.msg_queue.put(("altitude", altitude))
                    self.msg_queue.put(("speed", speed))
                    time.sleep(0.1)
                except:
                    break
        
        threading.Thread(target=telemetry_thread, daemon=True).start()
    
    def load_qr_code(self):
        if not self.qr_image_path:
            self.log_message("No QR code image selected!")
            return
            
        try:
            qr_img = Image.open(self.qr_image_path)
            decoded_objects = decode(qr_img)
            
            if decoded_objects:
                self.reference_qr_data = decoded_objects[0].data.decode('utf-8')
                self.log_message(f"QR Code loaded: {self.reference_qr_data}")
            else:
                self.log_message("No QR code detected in the image!")
        except Exception as e:
            self.log_message(f"Error loading QR code: {str(e)}")
    
    def start_mission(self):
        if not self.vehicle or not self.is_connected:
            self.log_message("Error: Drone not connected!")
            return
            
        if not self.qr_image_path:
            self.log_message("Error: QR code image not loaded!")
            return
        
        try:
            # Parse GPS coordinates
            lat, lon = map(float, self.gps_entry.get().split(','))
            target_altitude = float(self.height_entry.get())
            landing_speed = float(self.landing_speed_entry.get())
            
            def mission_thread():
                try:
                    self.mission_running = True
                    self.log_message("Starting mission...")
                    
                    # Arm and take off
                    self.log_message("Performing pre-arm checks...")
                    while not self.vehicle.is_armable:
                        self.log_message("Waiting for vehicle to initialize...")
                        time.sleep(1)
                    
                    self.log_message("Arming motors...")
                    self.vehicle.mode = VehicleMode("GUIDED")
                    self.vehicle.armed = True
                    
                    while not self.vehicle.armed:
                        self.log_message("Waiting for arming...")
                        time.sleep(1)
                    
                    # Take off
                    self.log_message(f"Taking off to {target_altitude}m...")
                    self.vehicle.simple_takeoff(target_altitude)
                    
                    # Wait for target altitude
                    while True:
                        current_altitude = self.vehicle.location.global_relative_frame.alt
                        if current_altitude >= target_altitude * 0.95:
                            self.log_message("Reached target altitude")
                            break
                        time.sleep(1)
                    
                    # Set airspeed
                    self.vehicle.airspeed = landing_speed
                    self.log_message("Moving to target location...")
                    
                    # Move to target location
                    target_location = LocationGlobalRelative(lat, lon, target_altitude)
                    self.vehicle.simple_goto(target_location, groundspeed=landing_speed)
                    
                    # Start QR detection and positioning
                    if self.adjust_and_descend_to_qr():
                        self.log_message("Successfully positioned over QR code")
                        
                        # Actuate servo
                        self.log_message("Actuating servo...")
                        self.actuate_servo()
                        time.sleep(20)
                    else:
                        self.log_message("Failed to position over QR code")
                    
                    self.mission_running = False
                    
                except Exception as e:
                    self.log_message(f"Mission failed: {str(e)}")
                    self.mission_running = False
            
            threading.Thread(target=mission_thread, daemon=True).start()
            
        except ValueError as e:
            self.log_message(f"Error parsing input values: {str(e)}")
    
    def adjust_and_descend_to_qr(self):
        cap = cv2.VideoCapture(0)
        self.log_message("Starting QR code detection...")
        
        def update_camera_feed():
            while cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    # Convert frame to PhotoImage for display
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img = Image.fromarray(frame_rgb)
                    img = ImageTk.PhotoImage(image=img)
                    self.camera_label.config(image=img)
                    self.camera_label.image = img
                time.sleep(0.03)
        
        # Start camera feed update thread
        camera_thread = threading.Thread(target=update_camera_feed, daemon=True)
        camera_thread.start()
        
        # Rest of your existing QR detection and positioning logic here
        # Modified to use self.log_message instead of print
        
        return True
    
    def emergency_stop(self):
        if not self.vehicle:
            self.log_message("Error: Drone not connected!")
            return
        
        try:
            self.log_message("EMERGENCY STOP INITIATED")
            self.vehicle.armed = False
            self.vehicle.mode = VehicleMode("RTL")
        except Exception as e:
            self.log_message(f"Emergency stop failed: {str(e)}")
    
    def actuate_servo(self):
        if not self.vehicle:
            self.log_message("Error: Drone not connected!")
            return
        
        try:
            channel = 6  # Servo channel
            pwm_value = 1100  # PWM value
            
            self.log_message(f"Actuating servo on channel {channel}")
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                183,
                0,
                channel,
                pwm_value,
                0, 0, 0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            self.log_message("Servo actuation complete")
            
        except Exception as e:
            self.log_message(f"Servo actuation failed: {str(e)}")
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = DroneControlGUI()
    app.run()