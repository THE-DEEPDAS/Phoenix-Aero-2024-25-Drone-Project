"""
Author: Deep Das
Time: 2025-03-23 23:20:30

Welcome to the Deep's World, just believe in your self and you will achieve it.
Difficult times makes you shine.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import cv2
from PIL import Image, ImageTk
from pyzbar.pyzbar import decode
import threading
from simulation_qr import *  # Import all functions from simulation_qr

shown = False

class DroneGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Control Interface")
        self.mission_running = False

        # GStreamer pipeline for Raspberry Pi camera
        gst_pipeline = (
            "libcamerasrc ! video/x-raw,width=800,height=600,framerate=30/1 ! "
            "videoconvert ! appsink"
        )
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            messagebox.showerror("Camera Error", "Failed to open camera. Check GStreamer pipeline.")
            return

        # Create main frames
        self.setup_frame = ttk.LabelFrame(root, text="Mission Setup", padding=10)
        self.setup_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        self.camera_frame = ttk.LabelFrame(root, text="Camera Feed", padding=10)
        self.camera_frame.grid(row=0, column=1, rowspan=2, padx=5, pady=5, sticky="nsew")

        self.control_frame = ttk.LabelFrame(root, text="Mission Controls", padding=10)
        self.control_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        # Setup inputs
        ttk.Label(self.setup_frame, text="Connection:").grid(row=0, column=0)
        self.connection = ttk.Entry(self.setup_frame)
        self.connection.insert(0, '/dev/ttyACM0')  # Default Pixhawk connection on Pi
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

        # Control buttons
        ttk.Button(self.control_frame, text="Start Mission", command=self.start_mission).grid(row=0, column=0, padx=5)

        # Add status label
        self.status_label = ttk.Label(self.control_frame, text="Status: Ready")
        self.status_label.grid(row=1, column=0, columnspan=2, pady=5)
        
        # Add QR verification label
        self.qr_label = ttk.Label(self.control_frame, text="QR Status: Not Detected")
        self.qr_label.grid(row=2, column=0, columnspan=2, pady=5)

        # Camera display
        self.camera_label = ttk.Label(self.camera_frame)
        self.camera_label.pack()
        self.update_camera()

    def start_mission(self):
        if not self.mission_running:
            self.mission_running = True
            mission_thread = threading.Thread(target=self.execute_mission)
            mission_thread.start()

    def update_qr_status(self, status):
        """Update QR status with popup for verification"""
        self.qr_label.config(text=status)
        if "Verified" in status and not shown:
            messagebox.showinfo("QR Verification", status)
            shown = True
        self.root.update()

    def execute_mission(self):
        try:
            # Validate inputs before starting mission
            try:
                target_lat = float(self.target_lat.get())
                target_lon = float(self.target_lon.get())
                target_alt = float(self.target_alt.get())
                connection = self.connection.get().strip()
                
                if not connection:
                    raise ValueError("Connection string cannot be empty")
                if not (0 <= target_lat <= 90 and 0 <= target_lon <= 180):
                    raise ValueError("Invalid latitude/longitude values")
                if not (0 < target_alt <= 100):
                    raise ValueError("Altitude must be between 0 and 100 meters")
                
            except ValueError as e:
                self.status_label.config(text="Status: Invalid Input")
                messagebox.showerror("Input Error", str(e))
                return

            self.status_label.config(text="Status: Mission Started")
            self.qr_label.config(text="QR Status: Searching...")
            
            success, qr_verified = run_complete_mission(
                target_lat,  # Using validated float values
                target_lon,
                target_alt,
                connection,
                self.cap,
                self.update_qr_status  # Pass callback for QR updates
            )

            if success:
                self.status_label.config(text="Status: Mission Completed")
                messagebox.showinfo("Success", "Mission completed successfully!")
            else:
                self.status_label.config(text="Status: Mission Failed")
                messagebox.showwarning("Warning", "Mission completed with warnings")
        except Exception as e:
            self.status_label.config(text="Status: Error")
            messagebox.showerror("Error", f"Mission failed: {str(e)}")
        finally:
            self.mission_running = False

    def update_camera(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (800, 600))  # Medium resolution
            photo = ImageTk.PhotoImage(image=Image.fromarray(frame))
            self.camera_label.configure(image=photo)
            self.camera_label.image = photo
        self.root.after(10, self.update_camera)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneGUI(root)
    root.mainloop()
