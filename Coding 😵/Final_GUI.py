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

class DroneControlGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Drone Control System")
        self.root.geometry("1200x900")
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
        self.expected_qr_code_data = "https://images.app.goo.gl/ajz5fubB7GLMiS5Y7"
        
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
            ("Hover Height (m)", "hover_height", "1.0"),
            ("Initial Height (m)", "height", "10"),
            ("IP:Port", "ip_port", "127.0.0.1:14550"),
            ("QR Code Image", "qr_code", "Select File...")
        ]
        
        for i, (label_text, var_name, default_value) in enumerate(input_fields):
            row = i // 3
            col = i % 3
            
            container = tk.Frame(input_frame, bg="#2c2c2c", padx=10, pady=5)
            container.grid(row=row, column=col, sticky="ew", padx=5, pady=5)
            
            label = tk.Label(
                container,
                text=label_text,
                fg="white",
                bg="#2c2c2c",
                font=("Helvetica", 10)
            )
            label.pack(anchor="w")
            
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
        
        self.connection_status = tk.Label(
            button_frame,
            text="Not Connected",
            fg="#ff4444",
            bg="#1a1a1a",
            font=("Helvetica", 10)
        )
        self.connection_status.pack(side="left", padx=10)
        
        # Camera container with message display
        camera_container = tk.Frame(self.main_frame, bg="#2c2c2c")
        camera_container.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Camera feed
        self.camera_frame = tk.LabelFrame(
            camera_container,
            text="Camera Feed",
            fg="white",
            bg="#2c2c2c",
            font=("Helvetica", 12)
        )
        self.camera_frame.pack(fill="both", expand=True, padx=0, pady=(0, 5))
        
        self.camera_label = tk.Label(self.camera_frame, bg="black", width=800, height=400)
        self.camera_label.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Message display below camera
        self.message_display = tk.Label(
            camera_container,
            text="Waiting for mission to start...",
            fg="#00ff00",
            bg="#1a1a1a",
            font=("Courier", 12),
            wraplength=780,
            justify="left",
            padx=10,
            pady=5
        )
        self.message_display.pack(fill="x", padx=5, pady=5)
        
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
    
    def show_qr_detected_popup(self):
        popup = tk.Toplevel(self.root)
        popup.title("QR Code Detected")
        popup.geometry("300x150")
        popup.configure(bg="#2c2c2c")
        
        # Center the popup on screen
        popup.geometry("+%d+%d" % (
            self.root.winfo_x() + (self.root.winfo_width() - 300) // 2,
            self.root.winfo_y() + (self.root.winfo_height() - 150) // 2
        ))
        
        # Message
        message = tk.Label(
            popup,
            text="QR Code Detected!\nProceeding with mission...",
            fg="white",
            bg="#2c2c2c",
            font=("Helvetica", 12),
            pady=20
        )
        message.pack(expand=True)
        
        # OK button
        ok_btn = tk.Button(
            popup,
            text="OK",
            command=popup.destroy,
            bg="#00a8e8",
            fg="white",
            font=("Helvetica", 10, "bold"),
            width=10
        )
        ok_btn.pack(pady=10)
    
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
        # Update message display
        self.msg_queue.put(("message", f"[{timestamp}] {message}"))
    
    def process_message_queue(self):
        try:
            while True:
                msg_type, msg_data = self.msg_queue.get_nowait()
                if msg_type == "log":
                    self.log_text.insert(tk.END, msg_data)
                    self.log_text.see(tk.END)
                elif msg_type == "message":
                    self.message_display.config(text=msg_data)
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
                qr_data = decoded_objects[0].data.decode('utf-8')
                if qr_data == self.expected_qr_code_data:
                    self.log_message(f"Valid QR Code loaded: {qr_data}")
                else:
                    self.log_message("Warning: QR code data doesn't match expected value")
            else:
                self.log_message("No QR code detected in the image!")
        except Exception as e:
            self.log_message(f"Error loading QR code: {str(e)}")
    
    def adjust_and_descend_to_qr(self, target_altitude=1.0):
        cap = cv2.VideoCapture(0)
        self.log_message("Starting QR code detection...")
        
        timeout = 60
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
                        # Show popup when QR code is first detected
                        self.root.after(0, self.show_qr_detected_popup)
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
                self.log_message("QR code detection timeout")
                break
                
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