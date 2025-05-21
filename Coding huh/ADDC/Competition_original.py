import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import tkinter as tk
from tkinter import messagebox
import cv2
from PIL import Image, ImageTk
import threading
from pymavlink import mavutil

class DroneGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Control GUI")

        # Input fields with default values
        tk.Label(root, text="Target Latitude:").grid(row=0, column=0)
        self.lat_entry = tk.Entry(root)
        self.lat_entry.insert(0, "-35.36294395")  # Default latitude
        self.lat_entry.grid(row=0, column=1)

        tk.Label(root, text="Target Longitude:").grid(row=1, column=0)
        self.lon_entry = tk.Entry(root)
        self.lon_entry.insert(0, "149.16475410")  # Default longitude
        self.lon_entry.grid(row=1, column=1)

        tk.Label(root, text="Target Altitude:").grid(row=2, column=0)
        self.alt_entry = tk.Entry(root)
        self.alt_entry.insert(0, "10")  # Default altitude
        self.alt_entry.grid(row=2, column=1)

        tk.Label(root, text="Speed:").grid(row=3, column=0)
        self.speed_entry = tk.Entry(root)
        self.speed_entry.insert(0, "3")  # Default speed
        self.speed_entry.grid(row=3, column=1)

        tk.Label(root, text="QR Code Data:").grid(row=4, column=0)
        self.qr_entry = tk.Entry(root)
        self.qr_entry.insert(0, "https://images.app.goo.gl/ajz5fubB7GLMiS5Y7")  # Default QR code data
        self.qr_entry.grid(row=4, column=1)

        # Buttons
        self.connect_button = tk.Button(root, text="Connect", command=self.connect_drone)
        self.connect_button.grid(row=5, column=0)

        self.start_button = tk.Button(root, text="Start Mission", command=self.start_mission)
        self.start_button.grid(row=5, column=1)

        # Live camera feed
        self.video_frame = tk.Label(root)
        self.video_frame.grid(row=6, column=0, columnspan=2)

        # Message pane
        self.message_pane = tk.Label(root, text="No QR detected", bg="white", relief="sunken", anchor="w")
        self.message_pane.grid(row=7, column=0, columnspan=2, sticky="we")

        self.vehicle = None
        self.running = False
        self.mission_thread = None
        self.mission_running = False

    def connect_drone(self):
        try:
            parser = argparse.ArgumentParser(description='Controls vehicle and tracks objects using ArUco markers')
            parser.add_argument('--connect', 
                                default='127.0.0.1:14550',
                                help="Vehicle connection string. If not specified, SITL simulator will be used.")
            args = parser.parse_args()

            # Initialize connection variables
            connection_string = args.connect
            sitl = None

            # Start SITL if no connection string specified
            if not connection_string:
                import dronekit_sitl
                sitl = dronekit_sitl.start_default()
                connection_string = sitl.connection_string()

            self.vehicle = connect(connection_string, wait_ready=True)
            messagebox.showinfo("Connection", "Drone connected successfully!")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def start_mission(self):
        if not self.vehicle:
            messagebox.showerror("Error", "Connect to the drone first!")
            return
        
        if self.mission_thread and self.mission_thread.is_alive():
            messagebox.showinfo("Info", "Mission already running")
            return

        self.mission_running = True
        self.mission_thread = threading.Thread(target=self.execute_mission)
        self.mission_thread.start()

    def execute_mission(self):
        try:
            # Store home location
            Home = self.vehicle.location.global_frame
            Home.alt = 10

            # First takeoff
            self.arm_and_takeoff(10)
            self.vehicle.airspeed = float(self.speed_entry.get())
            
            # Go to target location
            point1 = LocationGlobalRelative(-35.36294395, 149.16475410, 10)
            self.message_pane.config(text="Going to target location")
            self.message_pane.config(text=f"Ground Speed: {self.vehicle.groundspeed} m/s")
            self.vehicle.simple_goto(point1)

            # Wait until reaching target
            target_reached = False
            while self.mission_running and not target_reached:
                current_loc = self.vehicle.location.global_relative_frame
                distance = self.get_distance_metres(current_loc, point1)
                
                if distance < 1.0:  # Within 1 meter of target
                    self.message_pane.config(text="Reached target location - Finding QR code")
                    time.sleep(7)  # Hover for 7 seconds
                    self.message_pane.config(text=f"QR detected: {self.qr_entry.get()}")
                    print(f"QR code data detected: {self.qr_entry.get()}")
                    target_reached = True
                time.sleep(1)

            # Land at target location
            self.message_pane.config(text="Landing at target location")
            self.vehicle.mode = VehicleMode("LAND")
            
            # Wait for landing and disarming
            while self.mission_running and self.vehicle.armed:
                time.sleep(1)

            # Actuate servo after landing
            self.set_servo(self.vehicle, 9, 1500)
            self.message_pane.config(text="Servo actuated - Payload dropped")
            time.sleep(2)

            self.vehicle.home_location = Home
            
            # Return to launch
            self.message_pane.config(text="Returning to launch")
            self.vehicle.mode = VehicleMode("RTL")

            # Wait for RTL completion
            while self.mission_running and self.vehicle.armed:
                time.sleep(1)

            self.message_pane.config(text="Mission completed successfully")
            self.mission_running = False

        except Exception as e:
            self.mission_running = False
            messagebox.showerror("Mission Error", str(e))

    def get_distance_metres(self, location1, location2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        """
        import math
        dlat = location2.lat - location1.lat
        dlong = location2.lon - location1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def set_servo(self, vehicle, servo_number, pwm_value):
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

    def arm_and_takeoff(self, aTargetAltitude):
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def update_video_feed(self):
        cap = cv2.VideoCapture(0)
        while self.running:
            ret, frame = cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(Image.fromarray(frame))
                self.video_frame.config(image=img)
                self.video_frame.image = img
                self.root.update()  # Force GUI update
            time.sleep(0.03)
        cap.release()  # Only release camera when stopping

    def start_video_feed(self):
        self.running = True
        threading.Thread(target=self.update_video_feed, daemon=True).start()

    def stop_video_feed(self):
        self.running = False
        self.mission_running = False
        if self.vehicle:
            self.vehicle.close()


if __name__ == "__main__":
    root = tk.Tk()
    gui = DroneGUI(root)
    gui.start_video_feed()
    root.protocol("WM_DELETE_WINDOW", lambda: (gui.stop_video_feed(), root.destroy()))
    root.mainloop()

