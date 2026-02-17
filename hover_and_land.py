from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

TARGET_ALTITUDE = 3
HOVER_DURATION = 10
CONNECTION_STRING = '/dev/ttyACM0'


def connect_to_drone():
    """
    GPS ke bina kaam karta hai kyunki yeh sirf serial communication hai.
    """
    print("Drone se connect ho rahe hain...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True)
    print("Drone se successfully connected!")
    return vehicle


def arm_drone(vehicle):
    """
    GUIDED mode mein GPS zaroori nahi — barometer aur IMU se kaam chalta hai.
    Arming se motors spin hone lagte hain.
    """
    print("Drone ko GUIDED mode mein set kar rahe hain...")

    vehicle.mode = VehicleMode("GUIDED")

    while not vehicle.mode.name == 'GUIDED':
        print("   GUIDED mode ka wait kar rahe hain...")
        time.sleep(1)
    print("GUIDED mode set ho gaya!")

    print("Drone arm ho raha hai (motors start ho rahe hain)...")

    while not vehicle.is_armable:
        print("   Drone armable hone ka wait kar rahe hain...")
        print(f"   GPS: {vehicle.gps_0}, EKF OK: {vehicle.ekf_ok}")
        time.sleep(1)

    vehicle.armed = True

    while not vehicle.armed:
        print("   Arming ka wait kar rahe hain...")
        time.sleep(1)

    print("Drone ARMED hai! Motors spin ho rahe hain!")


def takeoff(vehicle, target_altitude):
    print(f"TAKEOFF! Target altitude: {target_altitude}m")

    vehicle.simple_takeoff(target_altitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"   Current Altitude: {current_altitude:.2f}m / Target: {target_altitude}m")

        if current_altitude >= target_altitude * 0.95:
            print(f"Target altitude {target_altitude}m reached!")
            break

        time.sleep(1)


def hover(vehicle, duration):
    """
    GPS ke bina hover kaise hota hai:
    - Barometer continuously altitude monitor karta hai
    - Gyroscope drone ko level rakhta hai
    - Accelerometer sudden movements compensate karta hai
    - Horizontal drift thoda ho sakta hai bina GPS ke, but altitude accurate rahegi
    """
    print(f"Drone {duration} seconds ke liye hover kar raha hai...")

    for i in range(duration, 0, -1):
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"   Hover: {i}s remaining | Altitude: {current_alt:.2f}m")
        time.sleep(1)

    print("Hover complete!")


def land_drone(vehicle):
    print("LANDING mode activate kar rahe hain...")

    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"   Landing... Current Altitude: {current_alt:.2f}m")
        time.sleep(1)

    print("Drone successfully LAND ho gaya aur DISARMED hai!")


def close_connection(vehicle):
    print("Connection close kar rahe hain...")
    vehicle.close()
    print("Connection safely closed!")


def main():
    print("YO deep code chalu....")

    vehicle = None

    try:
        vehicle = connect_to_drone()

        print(f"   Mode: {vehicle.mode.name}")
        print(f"   Armed: {vehicle.armed}")
        print(f"   GPS: {vehicle.gps_0}")
        print(f"   EKF OK: {vehicle.ekf_ok}")

        arm_drone(vehicle)

        takeoff(vehicle, TARGET_ALTITUDE)

        hover(vehicle, HOVER_DURATION)

        land_drone(vehicle)

    except KeyboardInterrupt:
        print("\nUser ne flight cancel ki (Ctrl+C)!")
        if vehicle and vehicle.armed:
            print("Emergency landing kar rahe hain...")
            vehicle.mode = VehicleMode("LAND")
            while vehicle.armed:
                time.sleep(1)
            print("Emergency landing complete!")

    except Exception as e:
        print(f"\nERROR: {e}")
        if vehicle and vehicle.armed:
            print("Error ke baad emergency landing kar rahe hain...")
            vehicle.mode = VehicleMode("LAND")
            while vehicle.armed:
                time.sleep(1)
            print("Emergency landing complete!")

    finally:
        if vehicle:
            close_connection(vehicle)

    print("Saras")


if __name__ == "__main__":
    main()
