import copy
import math
import statistics
import sys
import time
from collections import namedtuple
from numbers import Real
from typing import List, Tuple, Iterable

from dronekit import VehicleMode
from dronekit import connect
from pymavlink import mavutil

from config import ARDUPILOT_CONNECTION
from controller import Controller
from log import Logger
from simple_guidance import SimplePosition
from target_handler import TargetHandler

Target = namedtuple('Target', ['x', 'y', 'z', 'layer', 'time'])


class PositionAggregator:
    """Receives position updates and produces
    estimates by averaging the time series of inputs.
    Also maintains target tracking status details, such as
    the amount of time the drone has been unsuccessfully seeking
    a target and the last known height above the target. Finally,
    tracks the finalApproach variable, which is true if the inner
    qr code has been visible in the frame for at least 3 consecutive
    frames."""

    def __init__(self, handler: TargetHandler):
        """:param handler: The source for landing position data."""
        self.targetHandler = handler
        self.targetZero: List[Target] = []
        self.targetOne: List[Target] = []
        self.targetTwo: List[Target] = []
        self.missCount = 0
        self.hitCount = 0
        self.lost = True
        self.targetLayer = 0
        self.logger = Logger("Position_Estimate_Averages.csv",
                             ["Level 0", "X", "Y", "Z", "Level 1", "X", "Y", "Z", "Level 2", "X", "Y", "Z"])
        self.finalApproach = False
        self.targetLayer = 0
        self.lastHeight: float = 10
        self.missTime: float = 0
        self._firstLost = 0
        self.lastSeen = None

    @staticmethod
    def estimate_layer_position(targets: Iterable[Target]):
        """Get a weighted average of previously seen positions.

        Newer positions are weighted more heavily.

        :param targets:
        :returns: The x, y, z weighted averages."""
        now = time.time()
        x_sum = 0
        y_sum = 0
        z_sum = 0
        weight_sum = 0
        for target in targets:
            current_weight = 1 / (1 + (now - target.time))
            x_sum += current_weight * target.x
            y_sum += current_weight * target.y
            z_sum += current_weight * target.z
            weight_sum += current_weight
        if weight_sum > 0:
            x_sum = x_sum / weight_sum
            y_sum = y_sum / weight_sum
            z_sum = z_sum / weight_sum
            return x_sum, y_sum, z_sum
        return None, None, None

    def estimate_position(self):
        """Get the mean of position estimates over levels."""
        zero_x, zero_y, zero_z = self.estimate_layer_position(self.targetZero)
        one_x, one_y, one_z = self.estimate_layer_position(self.targetOne)
        two_x, two_y, two_z = self.estimate_layer_position(self.targetTwo)
        self.logger.writeline(["0", zero_x, zero_y, zero_z, "1", one_x, one_y, one_z, "2", two_x, two_y, two_z])
        avg_x = self._generate_averages(zero_x, one_x, two_x)
        avg_y = self._generate_averages(zero_y, one_y, two_y)
        avg_z = self._generate_averages(zero_z, one_z, two_z)
        if avg_z is not None:
            self.lastHeight = abs(avg_z)
        return avg_x, avg_y, avg_z

    @staticmethod
    def _generate_averages(*args: Real):
        """Get the mean of arguments within certain limits.

        :param args: Numeric values to average
        :returns: The mean of the arguments. Arguments which are None or whose
            absolute value is greater than 15 are not counted."""
        outputs = [
            x
            for x in args
            if x is not None and abs(x) < 15
        ]
        if len(outputs) > 0:
            return statistics.mean(outputs)
        return None

    def update_target_data(self):
        """Receives input from the TargetHandler class, parses the input layer
        by layer, and adds the new data to the other data stored within."""
        for i in range(0, 3):
            lz = self.targetHandler.get_target(i)
            if lz is not None:
                if lz.getLayer() > self.targetLayer:
                    self.targetLayer = lz.getLayer()
                self.missCount = 0
                self.hitCount += 1
                self.lost = False
                target = Target(*lz.getPosition(), layer=lz.getLayer(), time=time.time())
                if target.layer == 0:
                    self.targetZero.append(target)
                elif target.layer == 1:
                    self.targetOne.append(target)
                elif target.layer == 2:
                    self.targetTwo.append(target)
                self.lastSeen = target.layer
            else:
                self.missCount += 1
                if len(self.targetTwo) == 0 and len(self.targetOne) == 0 and len(self.targetZero) == 0:
                    self.hitCount = 0
                    if not self.lost:
                        self.lost = True
                        self.missTime = time.time()
        if len(self.targetTwo) > 3:
            self.finalApproach = True
        else:
            self.finalApproach = False
        self._clear_position_data(self.targetZero)
        self._clear_position_data(self.targetOne)
        self._clear_position_data(self.targetTwo)

    @staticmethod
    def _clear_position_data(targets: List[Target]):
        """Removes old position data from the list."""
        now = time.time()
        for target in targets:
            if now - target.time > 3:
                targets.remove(target)

    def get_last_height(self):
        return self.lastHeight


class DroneControl:
    """The DroneControl class is where the magic happens, so to speak.
    This class is responsible for translating output from image processing
    to drone command updates. When it is initialized, a whole raft of
    internal variables are created. Many of these are copies of the drone
    internal state, and some are internal configuration variables or
    simply references to related classes. Note that a connection to
    the drone is made when this class is instantiated. The PID
    controllers and logging functionality are also initialized and
    configured here."""

    def __init__(self, handler: TargetHandler):
        """
        :param handler: An instance of the TargetHandler class, used to
            manage the position estimates.
        """
        self.north_start = None
        self.east_start = None
        self.down_start = None
        self.loc = None
        self.attitude = None
        self.poll_delay = 0
        self.currentTime = self.previousTime = self.start_time = 0
        self.positioning = PositionAggregator(handler)
        self.simplePosition = None
        self.missLimit = 1000
        # Connect to the Vehicle
        print(f'Connecting to vehicle on: {ARDUPILOT_CONNECTION}')
        self.vehicle = connect(ARDUPILOT_CONNECTION, wait_ready=True)
        while not self.vehicle.is_armable:
            print("Initializing ...")
            time.sleep(1)

        self.Z_control = Controller(
            kp=0.5,
            ki=0.2,
            kd=0.4,
            target=0,
            lower_limit=-.15,
            upper_limit=1,
            update_rate=0.01,
            scalar=0
        )
        self.X_control = Controller(
            kp=0.105,
            ki=0.005,
            kd=0.055,
            target=0.0,
            lower_limit=-1,
            upper_limit=1,
            update_rate=0.01,
            scalar=0
        )
        self.Y_control = Controller(
            kp=0.105,
            ki=0.005,
            kd=0.055,
            target=0.0,
            lower_limit=-1,
            upper_limit=1,
            update_rate=0.01,
            scalar=0
        )
        self.landed = False
        self.waiting = True

        # logging
        self.logging = Logger("Drone_Control_Log.csv", ["Time", "Mode", "Roll", "Pitch", "Z Relative distance",
                                                        "Y Relative Distance", "X Relative Distance",
                                                        "Z Velocity Input", "Y Velocity Input", "X Velocity Input",
                                                        "Z Absolute Variance", "Y Absolute Variance",
                                                        "X Absolute Variance"])

    def startup_simulation(
            self,
            starting_altitude: float,
            target_rate: float):
        """Initialize the drone in simulation.

        This mode is for use in the simulated environment. It initializes the automated takeoff procedure before
        triggering the landing procedure. This function is used to test landing accuracy by positioning the drone
        10 meters above the landing site before initiating landing procedure.

        :param starting_altitude: integer value for altitude achieved before landing sequence begins.
        :param target_rate: desired minimum polling frequency (in polls per second)"""
        self.poll_delay = 1 / target_rate
        self.attitude = self.vehicle.attitude
        self.loc = self.vehicle.location
        self.down_start = copy.deepcopy(self.loc.local_frame.down)
        self.north_start = copy.deepcopy(self.loc.local_frame.north)
        self.east_start = copy.deepcopy(self.loc.local_frame.east)
        self.vehicle.parameters['AHRS_GPS_USE'] = 0  # disable gps
        self.arm_and_takeoff(starting_altitude)
        self.Z_control.enable(0)
        self.Y_control.enable(0)
        self.X_control.enable(0)
        self.Z_control.set_target(0)
        self.Y_control.set_target(0)
        self.X_control.set_target(0)
        self.currentTime = self.previousTime = self.start_time = time.time()
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Arming")
            time.sleep(1)
        print(self.vehicle.mode)
        print(f'Vehicle.armed: {self.vehicle.armed}')
        # disable failsafes
        self.vehicle.parameters['FS_THR_ENABLE'] = 0
        # disable gps
        self.vehicle.parameters['AHRS_GPS_USE'] = 0
        # clear previous commands
        self.vehicle.commands.download()
        self.vehicle.commands.wait_ready()
        self.vehicle.commands.clear()

    def startup(self, target_rate: float):
        """Start the drone in real deployment.

        This mode is for use in a real drone. It does not connect to the drone, but it does
        handle most of the synchronization with the drone and sets up the drone so that it
        is ready to receive commands. Note that this function takes no responsibility for
        ensuring the drone is armed or in flight before starting. This should be achieved
        in other code.

        :param target_rate: desired minimum polling frequency (in polls per second)"""
        self.poll_delay = 1 / target_rate
        self.attitude = self.vehicle.attitude
        self.loc = self.vehicle.location
        self.down_start = copy.deepcopy(self.loc.local_frame.down)
        self.north_start = copy.deepcopy(self.loc.local_frame.north)
        self.east_start = copy.deepcopy(self.loc.local_frame.east)
        self.vehicle.parameters['AHRS_GPS_USE'] = 0  # disable gps
        self.Z_control.enable(0)
        self.Y_control.enable(0)
        self.X_control.enable(0)
        self.Z_control.set_target(0)
        self.Y_control.set_target(0)
        self.X_control.set_target(0)
        self.currentTime = self.previousTime = self.start_time = time.time()
        self.vehicle.armed = True
        print(self.vehicle.mode)
        # disable failsafes
        self.vehicle.parameters['FS_THR_ENABLE'] = 0
        # disable gps
        self.vehicle.parameters['AHRS_GPS_USE'] = 0
        # clear previous commands
        self.vehicle.commands.download()
        self.vehicle.commands.wait_ready()
        self.vehicle.commands.clear()

    def init_simple_position(self, agent: SimplePosition):
        """
        This function is used to pass in a simple_position
        object to this class. This class is used in final descent
        and to validate landing conditions.
        """
        self.simplePosition = agent

    def arm_and_takeoff(self, target_altitude: Real):
        """Arm the vehicle and fly to the target altitude.

        :param target_altitude: The target altitude in meters
        """

        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
        self.vehicle.mode = VehicleMode("RTL")

    def get_absolute_position(self) -> Tuple[float, float, float]:
        """Get the difference between the drone's internal reference position and it's initial position.

        This is used to estimate the accuracy of our distance estimation
        function in simulation. In production it has little purpose because
        the takeoff and landing positions may be different.
        """
        down = copy.deepcopy(self.loc.local_frame.down)
        north = copy.deepcopy(self.loc.local_frame.north)
        east = copy.deepcopy(self.loc.local_frame.east)
        down_difference = down - self.down_start
        north_difference = north - self.north_start
        east_difference = east - self.east_start
        return east_difference, north_difference, down_difference

    def circle(self, limit: Real):
        """
        This function instructs the drone to move in circles in order to re-establish visual
        contact with the landing pad. A better search pattern no doubt exists.
        """
        print("Mode: Circle")
        modifier = limit / (2 * math.pi)
        base = time.time() % limit
        component = base / modifier
        x = math.cos(component)
        y = math.sin(component)
        self.positioning.missCount += 1
        return x, y

    async def update_velocity(self):
        """
        This master function does the heavy lifting within the drone_control module.
        At a high level, the function begins by updating the internal state. This
        includes updating the time, getting the latest position estimate from the
        target handler and passing it into the position aggregator, which produces
        a time-weighted position estimate average.

        Then, the function checks the drone state. If it is RTL mode, the rest
        of the function activates. It immediately switches the drone to guided
        mode and instructs the drone to hold position while a three second timer
        counts off. This is designed to give the position aggregator time to
        receive enough position estimates to fill the buffer (which is also three
        seconds).

        When the three second timer completes, the function begins sending
        position commands to the drone using the average position estimate if it
        exists, or the drone will be commanded to circle to search for the landing
        zone.

        Note: the drone will cease circling and will instead switch to "Land" mode
        if no targets have been located within the specified number of frames. In
        production, you may want to switch this to "RTL" instead if a suitable landing
        zone cannot be located. Likewise, the mode listener for landing takeover could
        be switched to "alt-hold" or some other unused mode.

        As the drone nears the landing pad, it will check how centered it is and pause
        descent until the drone is more centered. If the landing pad fills enough of
        the frame and is close enough to being centered, the drone will land and disarm.
        """
        now = time.time()
        mode = "Tracking"
        # early return
        if self.landed:
            return
        attitude = self.vehicle.attitude
        self.currentTime = time.time()
        time_dif = self.currentTime - self.previousTime
        self.positioning.update_target_data()
        # early return
        if time_dif < self.poll_delay:
            return
        self.previousTime = self.currentTime

        x_vector, y_vector, z_vector = self.positioning.estimate_position()

        if self.vehicle.mode.name == "RTL":
            self.vehicle.mode = VehicleMode("GUIDED")
            self.send_ned_velocity(0, 0, 0)
            self.waiting = False
            self.start_time = time.time()

        if self.waiting or (now - self.start_time < 3):
            mode = "Wait"

        if (x_vector is None or y_vector is None) and mode != "Wait":
            x_vector, y_vector = self.circle(math.ceil(self.positioning.get_last_height()))
            mode = "Circle"

        if mode != "Wait":
            new_z = 0.15
            if z_vector is not None:
                new_z = z_vector * 0.1
            if z_vector is not None and z_vector < 1:
                new_z = 0
                x_angle = abs(self.calc_angle(x_vector, z_vector))
                y_angle = abs(self.calc_angle(y_vector, z_vector))
                if x_angle < .1 and y_angle < 0.1:
                    new_z = 0.1
                    if self.positioning.finalApproach:
                        new_z = 0.05
            elif mode == "Circle" or self.positioning.missCount > 10:
                z_vector = 10
                new_z = -0.15
                if self.positioning.lastSeen == 2:
                    new_z = -0.5

        if mode != "Wait":
            new_y = self.Y_control.update(3 * y_vector)
            new_x = self.X_control.update(3 * x_vector)
        else:
            new_y = 0
            new_x = 0
            new_z = 0
        absolute_x, absolute_y, absolute_z = self.get_absolute_position()

        self.logging.writeline([now, mode, attitude.roll, attitude.pitch, z_vector, y_vector, x_vector,
                                new_z, new_y, new_x, absolute_z, absolute_y, absolute_x])

        # land drone
        if self.should_land():
            mode = "Land"
            print("Landing!")
            self.vehicle.mode = VehicleMode("LAND")
            time.sleep(2)
            self.vehicle.disarm()
            self.landed = True
            absolute_x, absolute_y, absolute_z = self.get_absolute_position()
            self.logging.writeline([time.time(), mode, attitude.roll, attitude.pitch, "N/A", "N/A", "N/A",
                                    "N/A", "N/A", "N/A", absolute_z, absolute_y, absolute_x])
            if self.positioning.missCount <= self.missLimit:
                print("Successfully Landed!")
                sys.exit(0)
            else:
                print("Safety Abort Landed!")
                sys.exit(1)
        elif mode != "Wait":
            # fly drone
            self.send_ned_velocity(new_x, new_y, new_z)

    def send_ned_velocity(
            self,
            velocity_x: float,
            velocity_y: float,
            velocity_z: float):
        """Move vehicle in direction based on specified velocity vectors.

        :param velocity_x: The desired velocity in the x direction
        :param velocity_y: The desired velocity in the y direction
        :param velocity_z: The desired velocity in the z direction
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)

    def condition_yaw(
            self,
            heading: float,
            relative: bool = False):
        """Allows the user to set the direction that the drone is facing.

        Copied from https://dronekit-python.readthedocs.io/en/stable/guide/copter/guided_mode.html
        :param heading: The angle to turn. Specified in degrees.
        :param relative: Whether the drone should rotate with respect to itself or the environment.
            When true, a positive heading indicates clockwise. When false, north is at 0 degrees,
            east is at 90 degrees, south is at 180 degrees, and west is at 270 degrees."""
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    @staticmethod
    def calc_angle(a, b):
        """A function for computing an angle between vectors."""
        return math.atan(a/b)

    def should_land(self):
        """Determine if the drone should land now."""
        if self.positioning.missCount > 200:
            return True

        attitude = self.vehicle.attitude
        if abs(attitude.pitch) >= 0.2 or abs(attitude.roll) >= 0.2:  # Drone tilted too much
            return False

        x_offset, y_offset, scale = self.simplePosition.get_scale_and_offset(2)
        for var in [x_offset, y_offset, scale]:
            if var is None:
                return False
        if self.positioning.finalApproach:
            if scale > 0.20 and abs(x_offset) < 250 and abs(y_offset) < 250:
                return True
            if scale > 0.15 and abs(x_offset) < 200 and abs(y_offset) < 200:
                return True
            if scale > 0.10 and abs(x_offset) < 150 and abs(y_offset) < 150:
                return True

        return False