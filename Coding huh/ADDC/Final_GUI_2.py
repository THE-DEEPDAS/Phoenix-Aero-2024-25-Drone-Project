import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
from PIL import Image, ImageTk
import cv2
import threading
import time
import math
from datetime import datetime
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pyzbar.pyzbar import decode
import queue

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
    def __init__(self):
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
        self.vehicle = None
        self.is_connected = False
        self.is_armed = False
        self.current_altitude = 0.0
        self.current_speed = 0.0
        self.mission_running = False
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
            text="● Not Connected",
            fg=self.colors["error"],
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
        
        # Input fields in a grid
        input_fields = [
            ("GPS Coordinates", "gps", "-35.36326170, 149.16462709"),
            ("Landing Speed (m/s)", "landing_speed", "3.0"),
            ("Hover Height (m)", "hover_height", "1.0"),
            ("Initial Height (m)", "height", "8.0"),
            ("IP:Port", "ip_port", "127.0.0.1:14550")
        ]
        
        for i, (label_text, var_name, default_value) in enumerate(input_fields):
            label = tk.Label(
                controls_panel,
                text=label_text,
                fg=self.colors["text"],
                bg=self.colors["bg_medium"],
                font=("Helvetica", 10)
            )
            label.grid(row=i, column=0, sticky="w", pady=5)
            
            entry = ModernEntry(
                controls_panel,
                bg=self.colors["bg_light"],
                fg=self.colors["text"],
                font=("Helvetica", 10),
                insertbackground=self.colors["text"]
            )
            entry.insert(0, default_value)
            entry.grid(row=i, column=1, sticky="ew", padx=(10, 0), pady=5)
            setattr(self, f"{var_name}_entry", entry)
        
        # QR Code selection
        qr_frame = tk.Frame(controls_panel, bg=self.colors["bg_medium"])
        qr_frame.grid(row=len(input_fields), column=0, columnspan=2, sticky="ew", pady=10)
        
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
        button_frame.grid(row=len(input_fields)+1, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        
        self.connect_btn = ModernButton(
            button_frame,
            text="Connect Drone",
            command=self.connect_drone,
            bg=self.colors["accent"],
            fg=self.colors["text"]
        )
        self.connect_btn.pack(side="left", expand=True, padx=(0, 5))
        
        self.start_mission_btn = ModernButton(
            button_frame,
            text="Start Mission",
            command=self.start_mission,
            bg=self.colors["success"],
            fg=self.colors["text"],
            state="disabled"
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
        
        self.log_text = scrolledtext.ScrolledText(
            log_panel,
            height=10,
            bg=self.colors["bg_light"],
            fg=self.colors["text"],
            font=("Consolas", 10),
            padx=10,
            pady=10
        )
        self.log_text.pack(fill="both", expand=True, padx=15, pady=15)

    def show_qr_detected_popup(self, qr_data):
        popup = tk.Toplevel(self.root)
        popup.title("QR Code Detected")
        popup.geometry("400x200")
        popup.configure(bg=self.colors["bg_medium"])
        
        # Make the popup modal
        popup.transient(self.root)
        popup.grab_set()
        
        # Center the popup on screen
        popup.geometry("+%d+%d" % (
            self.root.winfo_x() + (self.root.winfo_width() - 400) // 2,
            self.root.winfo_y() + (self.root.winfo_height() - 200) // 2
        ))
        
        # Close button in top-right corner
        close_btn = tk.Button(
            popup,
            text="✕",
            command=popup.destroy,
            bg=self.colors["bg_medium"],
            fg=self.colors["text"],
            font=("Helvetica", 12),
            relief="flat",
            borderwidth=0
        )
        close_btn.pack(anchor="ne", padx=10, pady=5)
        
        # QR Data
        tk.Label(
            popup,
            text="QR Code Data:",
            fg=self.colors["text"],
            bg=self.colors["bg_medium"],
            font=("Helvetica", 12, "bold"),
            pady=10
        ).pack()
        
        tk.Label(
            popup,
            text=qr_data,
            fg=self.colors["accent"],
            bg=self.colors["bg_medium"],
            font=("Helvetica", 12),
            wraplength=350
        ).pack(expand=True)
        
        # Wait for popup to be closed before continuing
        self.root.wait_window(popup)

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
                        text="● Connected" if msg_data else "● Not Connected",
                        fg=self.colors["success"] if msg_data else self.colors["error"]
                    )
                    self.start_mission_btn.config(state="normal" if msg_data else "disabled")
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
                qr_data = decoded_objects[0].data.decode('utf-8')
                self.expected_qr_code_data = qr_data
                self.log_message(f"QR Code loaded: {qr_data}")
            else:
                self.log_message("No QR code detected in the image!")
        except Exception as e:
            self.log_message(f"Error loading QR code: {str(e)}")
    
    def adjust_and_descend_to_qr(self, target_altitude=1.0):
        cap = cv2.VideoCapture(0)
        self.log_message("Starting QR code detection...")
        
        # Change timeout to 20 seconds
        timeout = 20  # Changed from 60 to 20 seconds
        start_time = time.time()
        qr_detected_once = False

        while True:
            ret, frame = cap.read()
            if not ret:
                self.log_message("Failed to capture frame")
                break

            # Update camera feed
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            img = ImageTk.PhotoImage(image=img)
            self.camera_label.config(image=img)
            self.camera_label.image = img

            detected_objects = decode(frame)
            qr_found = False
            for obj in detected_objects:
                qr_data = obj.data.decode('utf-8')
                if qr_data == self.expected_qr_code_data:
                    if not qr_detected_once:
                        # Show popup with actual QR data
                        self.root.after(0, lambda: self.show_qr_detected_popup(qr_data))
                    qr_found = True
                    qr_detected_once = True
                    points = obj.polygon
                    if len(points) > 0:
                        cx = int((points[0].x + points[2].x) / 2)
                        cy = int((points[0].y + points[2].y) / 2)
                        frame_center_x = frame.shape[1] // 2
                        frame_center_y = frame.shape[0] // 2
                        
                        offset_x = cx - frame_center_x
                        offset_y = cy - frame_center_y

                        movement_lat = self.vehicle.location.global_relative_frame.lat
                        movement_lon = self.vehicle.location.global_relative_frame.lon
                        
                        adjustment_factor = 0.000001
                        movement_lat -= offset_y * adjustment_factor
                        movement_lon += offset_x * adjustment_factor

                        current_altitude = self.vehicle.location.global_relative_frame.alt
                        descent_step = 0.3
                        target_location = LocationGlobalRelative(
                            movement_lat, movement_lon, max(current_altitude - descent_step, target_altitude)
                        )
                        self.vehicle.simple_goto(target_location)
                        self.log_message(f"Adjusting position: X={offset_x}, Y={offset_y}, Alt={current_altitude:.2f}m")

                        if abs(offset_x) < 10 and abs(offset_y) < 10 and current_altitude <= target_altitude + 0.1:
                            self.log_message("QR code centered and target altitude reached")
                            cap.release()
                            return True

            if not qr_found and qr_detected_once:
                current_altitude = self.vehicle.location.global_relative_frame.alt
                descent_step = 0.3
                if current_altitude > target_altitude:
                    target_location = LocationGlobalRelative(
                        self.vehicle.location.global_relative_frame.lat,
                        self.vehicle.location.global_relative_frame.lon,
                        max(current_altitude - descent_step, target_altitude)
                    )
                    self.vehicle.simple_goto(target_location)
                    self.log_message(f"Descending: Current Altitude = {current_altitude:.2f}m")
                    time.sleep(1)
                elif current_altitude <= target_altitude:
                    self.log_message("Reached target altitude")
                    cap.release()
                    return True

            if time.time() - start_time > timeout:
                self.log_message("QR code detection timeout (20 seconds)")
                cap.release()
                return False
                
            time.sleep(0.03)
            
        cap.release()
        return False

    def search_in_radius(self, center_lat, center_lon, radius, target_altitude):
        """
        Search for QR code in a circular pattern within given radius
        """
        self.log_message(f"Searching for QR code in {radius}m radius...")
        
        # Define search points in a spiral pattern
        points = []
        steps = 8  # Number of points in the search pattern
        for i in range(steps):
            angle = (i * 2 * 3.14159) / steps
            # Calculate offset from center
            lat_offset = radius * math.cos(angle) * 0.0000089
            lon_offset = radius * math.sin(angle) * 0.0000089 / math.cos(center_lat * 0.01745)
            points.append((center_lat + lat_offset, center_lon + lon_offset))
        
        # Visit each point and look for QR code
        for lat, lon in points:
            self.log_message(f"Moving to search position: {lat:.6f}, {lon:.6f}")
            target_location = LocationGlobalRelative(lat, lon, target_altitude)
            self.vehicle.simple_goto(target_location)
            
            # Wait for drone to reach the position
            time.sleep(5)
            
            # Check for QR code at this position
            if self.adjust_and_descend_to_qr(target_altitude):
                return True
        
        return False

    def actuate_servo(self, channel, pwm_value):
        if not self.vehicle:
            self.log_message("Error: Drone not connected!")
            return
        
        try:
            self.log_message(f"Setting servo on channel {channel}")
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
    
    def start_mission(self):
        if not self.vehicle or not self.is_connected:
            self.log_message("Error: Drone not connected!")
            return
            
        try:
            # Parse input values
            lat, lon = map(float, self.gps_entry.get().split(','))
            target_altitude = float(self.height_entry.get())
            landing_speed = float(self.landing_speed_entry.get())
            hover_height = float(self.hover_height_entry.get())
            
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
                    
                    # Wait to reach target location
                    time.sleep(10)
                    
                    # Try to find QR code at target location
                    qr_found = False
                    if self.adjust_and_descend_to_qr(hover_height):
                        qr_found = True
                    else:
                        self.log_message("QR code not found at target location. Starting radius search...")
                        # Search in 2m radius
                        qr_found = self.search_in_radius(lat, lon, 2, target_altitude)
                    
                    if qr_found:
                        self.log_message("Successfully positioned over QR code")
                    else:
                        self.log_message("QR code not found. Proceeding with payload drop at specified height...")
                        # Descend to hover height
                        descent_location = LocationGlobalRelative(lat, lon, hover_height)
                        self.vehicle.simple_goto(descent_location)
                        time.sleep(5)
                    
                    # Actuate servo
                    self.log_message("Actuating servo...")
                    self.actuate_servo(6, 1100)
                    time.sleep(20)
                    
                    # Return to launch
                    self.log_message("Returning to launch...")
                    self.vehicle.mode = VehicleMode("RTL")
                    
                    while self.vehicle.location.global_relative_frame.alt > 0.1:
                        time.sleep(0.5)
                    
                    self.log_message("Mission completed")
                    self.mission_running = False
                    
                except Exception as e:
                    self.log_message(f"Mission failed: {str(e)}")
                    self.mission_running = False
            
            threading.Thread(target=mission_thread, daemon=True).start()
            
        except ValueError as e:
            self.log_message(f"Error parsing input values: {str(e)}")
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = DroneControlGUI()
    app.run()