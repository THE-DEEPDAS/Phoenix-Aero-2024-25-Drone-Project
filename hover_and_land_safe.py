from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import threading

TARGET_ALTITUDE = 3
HOVER_DURATION = 10
CONNECTION_STRING = '/dev/ttyACM0'
MIN_BATTERY = 20
CRITICAL_BATTERY = 10
HEARTBEAT_TIMEOUT = 5


class DroneController:
    """
    Yeh class drone ko safely control karti hai.
    Isme teen major safety features hain:
    1. Connection loss detection — agar drone se baat band ho jaaye toh auto-land
    2. Low battery handling — battery kam hone pe warning aur auto-land
    3. Emergency motor cutoff — motors turant band karne ka option

    GPS ke bina kaam karta hai kyunki:
    - Altitude barometer se milti hai (air pressure measure karke)
    - Stability gyroscope + accelerometer se aati hai
    - GUIDED mode mein sirf altitude control ke liye GPS ki zaroorat nahi
    """

    def __init__(self):
        self.vehicle = None
        self.is_flying = False
        self.heartbeat_ok = True
        self.stop_monitor = False

    def connect_to_drone(self):
        """
        Drone se serial connection banata hai.
        heartbeat_timeout = HEARTBEAT_TIMEOUT seconds — agar itne time mein
        drone se koi signal nahi aaya toh connection lost maana jaayega.
        """
        print("Drone se connect ho rahe hain...")
        try:
            self.vehicle = connect(
                CONNECTION_STRING,
                wait_ready=True,
                heartbeat_timeout=HEARTBEAT_TIMEOUT
            )
            print("Drone se successfully connected!")
            self.print_status()
            return True
        except Exception as e:
            print(f"CONNECTION FAILED: {e}")
            return False

    def print_status(self):
        print(f"   Mode: {self.vehicle.mode.name}")
        print(f"   Armed: {self.vehicle.armed}")
        print(f"   Battery: {self.vehicle.battery.voltage}V, {self.vehicle.battery.level}%")
        print(f"   GPS: {self.vehicle.gps_0}")
        print(f"   EKF OK: {self.vehicle.ekf_ok}")

    # =====================================================
    # SAFETY FEATURE 1: CONNECTION LOSS DETECTION
    # =====================================================

    def start_connection_monitor(self):
        """
        Ek alag thread mein drone ka heartbeat monitor karta hai.
        Heartbeat kya hai — drone har second ek signal bhejta hai ("main zinda hoon").
        Agar HEARTBEAT_TIMEOUT seconds tak koi heartbeat nahi aaya,
        matlab connection toot gaya — toh auto-land trigger hoga.

        Threading isliye use karte hain kyunki main code apna kaam kare
        aur background mein heartbeat check hota rahe simultaneously.
        """
        self.stop_monitor = False
        monitor_thread = threading.Thread(target=self._monitor_heartbeat, daemon=True)
        monitor_thread.start()
        print("Connection monitor started (background thread)")

    def _monitor_heartbeat(self):
        """
        Background thread — har 1 second mein check karta hai ki
        drone se last heartbeat kab aaya tha.
        Agar zyada time ho jaaye toh connection lost maano.
        last_heartbeat ek timestamp hai — DroneKit isko auto-update karta hai
        jab bhi drone se heartbeat message aata hai.
        """
        while not self.stop_monitor:
            try:
                if self.vehicle and self.is_flying:
                    last_hb = self.vehicle.last_heartbeat
                    if last_hb > HEARTBEAT_TIMEOUT:
                        print(f"CONNECTION LOST! Last heartbeat: {last_hb:.1f}s ago")
                        self.heartbeat_ok = False
                        self._handle_connection_loss()
                        return
            except Exception:
                pass
            time.sleep(1)

    def _handle_connection_loss(self):
        """
        Connection tootne pe kya karna hai:
        Drone ko LAND mode mein daal do — yeh onboard flight controller pe chalta hai.
        Matlab agar connection toot bhi gaya, toh drone khud se land kar lega
        kyunki LAND mode flight controller ke andar run hota hai,
        usse ground station se connection ki zaroorat nahi hoti.
        """
        print("CONNECTION LOSS DETECTED!")
        print("Drone ko LAND mode mein daal rahe hain...")
        try:
            self.vehicle.mode = VehicleMode("LAND")
            print("LAND mode set kiya — drone apne aap land karega")
        except Exception:
            print("LAND mode set nahi ho paya — drone ka failsafe activate hoga")

    # =====================================================
    # SAFETY FEATURE 2: LOW BATTERY HANDLING
    # =====================================================

    def check_battery(self):
        """
        Battery level check karta hai flight se pehle aur flight ke dauran.
        Drone ki battery voltage aur percentage flight controller se aati hai.

        Teen levels hain:
        - Above MIN_BATTERY (20%): Sab theek, udho
        - Below MIN_BATTERY (20%): WARNING — jaldi land karo
        - Below CRITICAL_BATTERY (10%): DANGER — turant land karo, crash ho sakta hai

        Battery check mein GPS ki koi zaroorat nahi — yeh flight controller ka
        power module directly measure karta hai.
        """
        battery = self.vehicle.battery
        voltage = battery.voltage
        level = battery.level
        print(f"Battery: {voltage}V, {level}%")

        if level is None:
            print("Battery level read nahi ho raha — sensor check karo")
            return "unknown"

        if level < CRITICAL_BATTERY:
            print(f"CRITICAL BATTERY! {level}% — turant land karna padega!")
            return "critical"
        elif level < MIN_BATTERY:
            print(f"LOW BATTERY WARNING! {level}% — jaldi land karo")
            return "low"
        else:
            print(f"Battery OK: {level}%")
            return "ok"

    def start_battery_monitor(self):
        """
        Background thread mein battery continuously monitor karta hai.
        Har 5 seconds mein battery check hoti hai.
        Agar critical level pe pahunch jaaye toh auto-land trigger hota hai.
        """
        monitor_thread = threading.Thread(target=self._monitor_battery, daemon=True)
        monitor_thread.start()
        print("Battery monitor started (background thread)")

    def _monitor_battery(self):
        """
        Har 5 seconds mein battery level check karta hai.
        Critical pe immediate land, low pe warning deta hai.
        """
        while not self.stop_monitor:
            try:
                if self.vehicle and self.is_flying:
                    level = self.vehicle.battery.level
                    if level is not None:
                        if level < CRITICAL_BATTERY:
                            print(f"CRITICAL BATTERY ({level}%)! Auto-landing...")
                            self.force_land()
                            return
                        elif level < MIN_BATTERY:
                            print(f"LOW BATTERY WARNING: {level}%")
            except Exception:
                pass
            time.sleep(5)

    # =====================================================
    # SAFETY FEATURE 3: EMERGENCY MOTOR CUTOFF
    # =====================================================

    def emergency_motor_cutoff(self):
        """
        EMERGENCY STOP — saare motors turant band kardo.

        Yeh kab use karna hai:
        - Drone kisi cheez se takra raha hai aur land karne ka time nahi hai
        - Propeller mein kuch fass gaya hai
        - Drone control se bahar ja raha hai aur kisi ko lag sakta hai

        WARNING: Motors band hone pe drone seedha neeche girega!
        Yeh sirf LAST RESORT hai — pehle LAND mode try karo.

        Kaise kaam karta hai:
        - MAV_CMD_COMPONENT_ARM_DISARM command bhejta hai force flag ke saath
        - Force flag (21196) matlab "chahe kuch bhi ho, motors band karo"
        - Normally drone udte hue disarm nahi hota safety ke liye,
          but force flag se yeh override ho jaata hai
        """
        print("!!! EMERGENCY MOTOR CUTOFF !!!")
        print("Saare motors turant band ho rahe hain!")
        try:
            self.vehicle._master.mav.command_long_send(
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,    # confirmation number
                0,    # 0 = disarm
                21196,  # force disarm magic number — yeh override karta hai in-flight safety
                0, 0, 0, 0, 0
            )
            self.is_flying = False
            print("Motors KILLED! Drone ab freefall mein hai!")
        except Exception as e:
            print(f"Motor cutoff FAILED: {e}")

    # =====================================================
    # FLIGHT FUNCTIONS
    # =====================================================

    def arm_drone(self):
        """
        GUIDED mode mein GPS zaroori nahi — barometer aur IMU se kaam chalta hai.
        Arming se motors spin hone lagte hain.
        """
        print("Drone ko GUIDED mode mein set kar rahe hain...")
        self.vehicle.mode = VehicleMode("GUIDED")

        while not self.vehicle.mode.name == 'GUIDED':
            print("   GUIDED mode ka wait kar rahe hain...")
            time.sleep(1)
        print("GUIDED mode set ho gaya!")

        print("Drone arm ho raha hai...")
        while not self.vehicle.is_armable:
            print("   Drone armable hone ka wait kar rahe hain...")
            print(f"   GPS: {self.vehicle.gps_0}, EKF OK: {self.vehicle.ekf_ok}")
            time.sleep(1)

        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("   Arming ka wait kar rahe hain...")
            time.sleep(1)
        print("Drone ARMED hai!")

    def takeoff(self, target_altitude):
        """
        Barometer se altitude track hoti hai — GPS ki zaroorat nahi.
        simple_takeoff mein drone vertically upar jaata hai aur
        barometer air pressure change se height calculate karta hai.
        """
        print(f"TAKEOFF! Target: {target_altitude}m")
        self.vehicle.simple_takeoff(target_altitude)
        self.is_flying = True

        while True:
            if not self.heartbeat_ok:
                print("Takeoff abort — connection lost!")
                return False

            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(f"   Altitude: {current_altitude:.2f}m / {target_altitude}m")

            if current_altitude >= target_altitude * 0.95:
                print(f"Target altitude {target_altitude}m reached!")
                return True
            time.sleep(1)

    def hover(self, duration):
        """
        GPS ke bina hover:
        - Barometer altitude hold karta hai
        - Gyroscope level rakhta hai
        - Accelerometer sudden movements compensate karta hai
        - Thoda horizontal drift ho sakta hai bina GPS ke
        """
        print(f"Hovering for {duration} seconds...")
        for i in range(duration, 0, -1):
            if not self.heartbeat_ok:
                print("Hover abort — connection lost!")
                return

            battery_status = self.check_battery()
            if battery_status == "critical":
                print("Hover abort — battery critical!")
                self.force_land()
                return

            current_alt = self.vehicle.location.global_relative_frame.alt
            print(f"   Hover: {i}s left | Alt: {current_alt:.2f}m | Batt: {self.vehicle.battery.level}%")
            time.sleep(1)
        print("Hover complete!")

    def force_land(self):
        """
        Safe landing — drone controlled speed se neeche aata hai.
        LAND mode barometer se altitude track karta hai — GPS nahi chahiye.
        Flight controller onboard LAND handle karta hai,
        toh agar connection bhi toot jaaye toh landing continue rahegi.
        """
        print("LANDING...")
        try:
            self.vehicle.mode = VehicleMode("LAND")
            while self.vehicle.armed:
                current_alt = self.vehicle.location.global_relative_frame.alt
                print(f"   Landing... Alt: {current_alt:.2f}m")
                time.sleep(1)
            self.is_flying = False
            print("Drone LANDED aur DISARMED!")
        except Exception as e:
            print(f"Normal landing FAILED: {e}")
            print("Emergency motor cutoff try kar rahe hain...")
            self.emergency_motor_cutoff()

    def close_connection(self):
        self.stop_monitor = True
        if self.vehicle:
            print("Connection close kar rahe hain...")
            self.vehicle.close()
            print("Connection closed!")


def main():
    print("YO deep code chalu....")

    drone = DroneController()

    try:
        if not drone.connect_to_drone():
            print("Connection fail — exit")
            return

        battery_status = drone.check_battery()
        if battery_status == "critical":
            print("Battery critical hai — fly nahi kar sakte!")
            return
        if battery_status == "low":
            print("Battery low hai — short flight only!")

        drone.start_connection_monitor()
        drone.start_battery_monitor()

        drone.arm_drone()

        if not drone.takeoff(TARGET_ALTITUDE):
            print("Takeoff fail hua!")
            drone.force_land()
            return

        drone.hover(HOVER_DURATION)

        drone.force_land()

    except KeyboardInterrupt:
        print("\nCtrl+C detected!")
        if drone.vehicle and drone.vehicle.armed:
            print("Emergency landing...")
            try:
                drone.force_land()
            except Exception:
                print("Landing fail — motor cutoff!")
                drone.emergency_motor_cutoff()

    except Exception as e:
        print(f"\nERROR: {e}")
        if drone.vehicle and drone.vehicle.armed:
            try:
                drone.force_land()
            except Exception:
                drone.emergency_motor_cutoff()

    finally:
        drone.close_connection()

    print("Saras")


if __name__ == "__main__":
    main()
