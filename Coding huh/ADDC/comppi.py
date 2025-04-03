import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import tkinter as tk
from tkinter import messagebox, ttk
import threading
from pymavlink import mavutil
from picamera import PiCamera
from picamera.array import PiRGBArray
from PIL import Image, ImageTk
import io
import random
import math

class DroneGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Phoenix Aero Controller")
        
        # Set window size and make it non-resizable
        self.root.geometry("800x900")
        self.root.resizable(False, False)
        
        # Load and set background
        bg_image = Image.open("bg.jpg") 
        self.bg_photo = ImageTk.PhotoImage(bg_image)
        self.bg_label = tk.Label(root, image=self.bg_photo)
        self.bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        
        # Create main frame with transparent dark background
        self.main_frame = tk.Frame(root, bg='#1a1a1a', bd=2)
        self.main_frame.place(relx=0.1, rely=0.1, relwidth=0.8, relheight=0.8)
        
        # Style configuration
        self.style = ttk.Style()
        self.style.configure('Dark.TLabel', 
                           background='#1a1a1a',
                           foreground='#00ff00',
                           font=('Orbitron', 10))
        self.style.configure('Dark.TEntry',
                           fieldbackground='#2d2d2d',
                           foreground='#00ff00')
        
        # Input fields with space theme
        self.create_input_fields()
        
        # Create animated buttons
        self.create_buttons()
        
        # Video frame with space-themed border
        self.video_frame = tk.Label(self.main_frame, 
                                  bg='#000000', 
                                  relief='ridge', 
                                  bd=3)
        self.video_frame.grid(row=6, column=0, columnspan=2, pady=10)
        
        # Message panes with animation capability
        self.create_message_panes()
        
        # Initialize fire particles
        self.particles = []
        self.animate_fire()

        self.vehicle = None
        self.running = False
        self.mission_thread = None
        self.mission_running = False
        self.camera = None

    def create_input_fields(self):
        labels = ["Target Latitude:", "Target Longitude:", "Target Altitude:", 
                 "Speed:", "QR Code Data:"]
        defaults = ["-35.36294395", "149.16475410", "10", "3", 
                   "https://images.app.goo.gl/ajz5fubB7GLMiS5Y7"]
        
        # Create dictionary to store entries
        self.entries = {}
        
        for i, (label, default) in enumerate(zip(labels, defaults)):
            tk.Label(self.main_frame, 
                    text=label, 
                    bg='#1a1a1a', 
                    fg='#00ff00',
                    font=('Orbitron', 10)).grid(row=i, column=0, pady=5)
            entry = tk.Entry(self.main_frame, 
                           bg='#2d2d2d', 
                           fg='#00ff00',
                           insertbackground='#00ff00',
                           font=('Orbitron', 10),
                           relief='sunken')
            entry.insert(0, default)
            entry.grid(row=i, column=1, pady=5)
            # Store entry widget with a clean key name
            key = label.lower().replace(' ','_').replace(':','')
            self.entries[key] = entry

    def create_buttons(self):
        button_frame = tk.Frame(self.main_frame, bg='#1a1a1a')
        button_frame.grid(row=5, column=0, columnspan=2, pady=10)
        
        for text, command in [("Connect", self.connect_drone), 
                            ("Start Mission", self.start_mission)]:
            btn = tk.Button(button_frame, 
                          text=text,
                          command=command,
                          bg='#2d2d2d',
                          fg='#00ff00',
                          font=('Orbitron', 10),
                          relief='raised',
                          activebackground='#404040')
            btn.pack(side=tk.LEFT, padx=10)
            
    def create_message_panes(self):
        self.qr_message_pane = tk.Label(self.main_frame, 
                                      text="No QR detected",
                                      bg='#000000',
                                      fg='#00ff00',
                                      font=('Orbitron', 10),
                                      relief='sunken',
                                      anchor="w")
        self.qr_message_pane.grid(row=7, column=0, columnspan=2, sticky="we", pady=5)
        
        self.message_pane = tk.Label(self.main_frame,
                                   text="Ready",
                                   bg='#000000',
                                   fg='#00ff00',
                                   font=('Orbitron', 10),
                                   relief='sunken',
                                   anchor="w")
        self.message_pane.grid(row=8, column=0, columnspan=2, sticky="we", pady=5)

    def animate_fire(self):
        # Create new particles
        if len(self.particles) < 50:
            x = random.randint(0, self.root.winfo_width())
            self.particles.append({
                'x': x,
                'y': self.root.winfo_height(),
                'vx': random.uniform(-2, 2),
                'vy': random.uniform(-8, -4),
                'life': 1.0
            })
        
        # Update and draw particles
        canvas = tk.Canvas(self.root, 
                         width=self.root.winfo_width(),
                         height=self.root.winfo_height(),
                         highlightthickness=0)
        canvas.place(x=0, y=0)
        
        for p in self.particles[:]:
            p['x'] += p['vx']
            p['y'] += p['vy']
            p['life'] -= 0.02
            
            if p['life'] <= 0:
                self.particles.remove(p)
            else:
                color = '#{:02x}00{:02x}'.format(
                    int(255 * p['life']),
                    int(255 * (1 - p['life']))
                )
                canvas.create_oval(
                    p['x'], p['y'],
                    p['x'] + 3, p['y'] + 3,
                    fill=color,
                    outline=''
                )
        
        self.root.after(50, self.animate_fire)

    def update_message(self, pane, text):
        # Animate message update
        current_text = pane.cget("text")
        if current_text != text:
            pane.configure(fg='#ffffff')  # Flash white
            pane.after(100, lambda: pane.configure(text=text, fg='#00ff00'))

    def connect_drone(self):
        try:
            parser = argparse.ArgumentParser()
            parser.add_argument('--connect', default='/dev/ttyACM0')
            args = parser.parse_args()

            # Connect to the actual drone
            self.vehicle = connect(args.connect, wait_ready=True)
            messagebox.showinfo("Connection", "Drone connected successfully!")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def get_distance_metres(self, location1, location2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        """
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
        # Initialize PiCamera
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 24
        
        # Allow camera to warm up
        time.sleep(2)
        
        # Create in-memory stream
        stream = io.BytesIO()
        
        try:
            while self.running:
                # Capture frame
                stream.seek(0)
                self.camera.capture(stream, format='jpeg', use_video_port=True)
                stream.seek(0)
                
                # Convert to PIL Image
                pil_image = Image.open(stream)
                
                # Convert to ImageTk format for display
                img = ImageTk.PhotoImage(pil_image)
                self.video_frame.config(image=img)
                self.video_frame.image = img
                self.root.update()  # Force GUI update
                
                # Small delay to maintain frame rate
                time.sleep(0.01)
        finally:
            if self.camera:
                self.camera.close()

    def start_video_feed(self):
        self.running = True
        threading.Thread(target=self.update_video_feed, daemon=True).start()

    def stop_video_feed(self):
        self.running = False
        self.mission_running = False
        if self.camera:
            self.camera.close()
        if self.vehicle:
            self.vehicle.close()

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
            target_alt = float(self.entries['target_altitude'].get())
            Home.alt = target_alt

            # First takeoff
            self.arm_and_takeoff(target_alt)
            self.vehicle.airspeed = float(self.entries['speed'].get())
            
            # Go to target location
            target_lat = float(self.entries['target_latitude'].get())
            target_lon = float(self.entries['target_longitude'].get())
            point1 = LocationGlobalRelative(target_lat, target_lon, target_alt)
            
            # Rest of execute_mission remains the same...
            self.update_message(self.message_pane, "Going to target location")
            self.update_message(self.message_pane, f"Ground Speed: {self.vehicle.groundspeed} m/s")
            self.vehicle.simple_goto(point1)

            # Wait until reaching target
            target_reached = False
            while self.mission_running and not target_reached:
                current_loc = self.vehicle.location.global_relative_frame
                distance = self.get_distance_metres(current_loc, point1)
                
                if distance < 1.0:  # Within 1 meter of target
                    self.update_message(self.qr_message_pane, f"QR detected: {self.entries['qr_code_data'].get()}")
                    self.update_message(self.message_pane, "Reached target location - Finding QR code")
                    time.sleep(7)  # Hover for 7 seconds
                    print(f"QR code data detected: {self.entries['qr_code_data'].get()}")
                    target_reached = True
                time.sleep(1)

            # Land at target location
            self.update_message(self.message_pane, "Landing at target location")
            self.vehicle.mode = VehicleMode("LAND")
            
            # Wait for landing and disarming
            while self.mission_running and self.vehicle.armed:
                time.sleep(1)

            # Actuate servo after landing
            self.set_servo(self.vehicle, 9, 1500)
            self.update_message(self.message_pane, "Servo actuated - Payload dropped")
            time.sleep(2)

            # Second takeoff for RTL
            self.arm_and_takeoff(target_alt)
            self.vehicle.home_location = Home
            
            # Return to launch
            self.update_message(self.message_pane, "Returning to launch")
            self.vehicle.mode = VehicleMode("RTL")

            # Wait for RTL completion
            while self.mission_running and self.vehicle.armed:
                time.sleep(1)

            self.update_message(self.message_pane, "Mission completed successfully")
            self.mission_running = False

        except Exception as e:
            self.mission_running = False
            messagebox.showerror("Mission Error", str(e))


if __name__ == "__main__":
    root = tk.Tk()
    gui = DroneGUI(root)
    gui.start_video_feed()
    root.protocol("WM_DELETE_WINDOW", lambda: (gui.stop_video_feed(), root.destroy()))
    root.mainloop()
