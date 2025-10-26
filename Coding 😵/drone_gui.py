import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import cv2
from PIL import Image, ImageTk
from pyzbar.pyzbar import decode
from QR_GUI_ADDC_2025 import *
import threading

class DroneGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Control Interface")
        self.vehicle = None
        self.camera_active = False
        self.mission_running = False

        # Create main frames
        self.setup_frame = ttk.LabelFrame(root, text="Setup", padding=10)
        self.setup_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        self.camera_frame = ttk.LabelFrame(root, text="Camera Feed", padding=10)
        self.camera_frame.grid(row=0, column=1, rowspan=2, padx=5, pady=5, sticky="nsew")

        self.control_frame = ttk.LabelFrame(root, text="Controls", padding=10)
        self.control_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        # Setup inputs
        ttk.Label(self.setup_frame, text="Connection:").grid(row=0, column=0)
        self.connection = ttk.Entry(self.setup_frame)
        self.connection.insert(0, '/dev/ttyACM0')
        self.connection.grid(row=0, column=1)

        ttk.Label(self.setup_frame, text="Target Latitude:").grid(row=1, column=0)
        self.target_lat = ttk.Entry(self.setup_frame)
        self.target_lat.insert(0, '21.1598244')
        self.target_lat.grid(row=1, column=1)

        ttk.Label(self.setup_frame, text="Target Longitude:").grid(row=2, column=0)
        self.target_lon = ttk.Entry(self.setup_frame)
        self.target_lon.insert(0, '72.7863403')
        self.target_lon.grid(row=2, column=1)

        ttk.Label(self.setup_frame, text="Target Altitude:").grid(row=3, column=0)
        self.target_alt = ttk.Entry(self.setup_frame)
        self.target_alt.insert(0, '10')
        self.target_alt.grid(row=3, column=1)

        # QR Code section
        ttk.Button(self.setup_frame, text="Load QR Code", command=self.load_qr_code).grid(row=4, column=0, columnspan=2)
        self.qr_data = tk.StringVar()
        self.qr_data.set("No QR code loaded")
        ttk.Label(self.setup_frame, textvariable=self.qr_data).grid(row=5, column=0, columnspan=2)

        # Camera display
        self.camera_label = ttk.Label(self.camera_frame)
        self.camera_label.pack()

        # Control buttons
        ttk.Button(self.control_frame, text="Connect Drone", command=self.connect_drone).grid(row=0, column=0, padx=5)
        ttk.Button(self.control_frame, text="Start Mission", command=self.start_mission).grid(row=0, column=1, padx=5)
        ttk.Button(self.control_frame, text="RTL", command=self.return_to_launch).grid(row=0, column=2, padx=5)

        # Start camera
        self.cap = cv2.VideoCapture(0)
        self.update_camera()

    def load_qr_code(self):
        file_path = filedialog.askopenfilename(filetypes=[("Image files", "*.png *.jpg *.jpeg")])
        if file_path:
            image = cv2.imread(file_path)
            detected = decode(image)
            if detected:
                global expected_qr_code_data
                expected_qr_code_data = detected[0].data.decode('utf-8')
                self.qr_data.set(f"Loaded QR code: {expected_qr_code_data}")
            else:
                self.qr_data.set("No QR code found in image")

    def update_camera(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (640, 480))
            photo = ImageTk.PhotoImage(image=Image.fromarray(frame))
            self.camera_label.configure(image=photo)
            self.camera_label.image = photo
        self.root.after(10, self.update_camera)

    def connect_drone(self):
        try:
            global vehicle
            vehicle = connect(self.connection.get(), wait_ready=True)
            tk.messagebox.showinfo("Success", "Drone connected successfully!")
        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to connect: {str(e)}")

    def start_mission(self):
        if not self.mission_running and vehicle:
            self.mission_running = True
            mission_thread = threading.Thread(target=self.run_mission)
            mission_thread.start()

    def run_mission(self):
        try:
            # Execute mission with GUI parameters
            arm_and_takeoff(float(self.target_alt.get()))
            vehicle.airspeed = 7 #last time 7 hati etle
            
            # Go to target location
            target_location = LocationGlobalRelative(
                float(self.target_lat.get()),
                float(self.target_lon.get()),
                float(self.target_alt.get())
            )
            vehicle.simple_goto(target_location, groundspeed=2)
            wait_for_arrival(target_location)
            
            # Descend to QR scanning height
            descend_and_hover(3.0)
            
            # Start precision landing
            if precision_landing(vehicle, self.cap):
                messagebox.showinfo("Success", "Landed successfully on QR code")
            else:
                response = messagebox.showwarning("Warning", 
                    "No QR code detected after 30 seconds.\nDropping payload at current altitude.")
                # Hover and drop payload
                descend_and_hover(3.0)  # Maintain safe altitude
            
            # Release payload
            actuate_servo(vehicle, 6, 1100)
            time.sleep(5)
            self.return_to_launch()
                
        except Exception as e:
            tk.messagebox.showerror("Error", f"Mission failed: {str(e)}")
        finally:
            self.mission_running = False

    def return_to_launch(self):
        if vehicle:
            vehicle.mode = VehicleMode("RTL")
            while vehicle.location.global_relative_frame.alt > 0.1:
                print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
                time.sleep(1)
            vehicle.armed = False

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneGUI(root)
    root.mainloop()
