from socket_handler import checksum_fcn
from math import pi

MIN_ANGLES = [1028, 1784, 1028, 790, 1928]
MAX_ANGLES = [3073, 2570, 3073, 3328, 2800]
V_RANGE = [55, 45, 90, 67, 90]
LOAD_RANGE = [225, 300, 250, 300, 400]
MAX_TEMP = 80
BUFFER = 10
DYNA_MIN = 0
DYNA_MAX = 4095


def change_scale(old_min, old_max, new_min, new_max, old_value):
    """Converts value in [old_min, old_max] proportionally to [new_min, new_max]"""
    return (((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min


def clamp_range(min_val, max_val, value):
    """Clamps value within [min_val,max_val]"""
    return max(min_val, min(value, max_val))


class ServoInfo(object):
    # Abstract class to hold basic implementation for servo related information
    def __init__(self):
        self.id = None
        self.position = None  # The current position value of the servo
        self.velocity = None  # The current moving speed of the servo
        self.load = None  # The current load applied to the servo
        self.temp = None  # The internal temperature of the servo in deg


class MxSeries(ServoInfo):
    # Specific instance of all servos of the mx series type (i.e mx-28, mx-64, mx-106)
    # * Position: 0 - 4095 (0.088 degrees/value, The central neutral position for each servo is 2048)
    # * Velocity: 0 - 2047 (0.11rpm/value, If the value reads 1024 then it means that the  servo is stationary. A value below 1023 means the motor rotates in a CCW direction and a value above 1023 means the motor rotates in a CW direction.)
    # TODO ^^^ This does not appear to be true, got these descriptions for BrachIO, this value seems to be the current set velocity, could of been something I did wrong in BrachIOplexus
    # * Load: 0 - 2047 (0.1% load/value, If the value is 0 - 1023 it means the load works to the CCW direction and if the value is 1024-2047 it means the load works in the CW direction. A value of 1024 means that there is 0% load. )
    # * Temp: 0 - 255 (1 degC/value)
    # * State: 0 - 3 (see above for definitions of each state)
    def __init__(self, index):
        super().__init__()
        self.id = index + 1  # ID start at 1, index starts at 0
        self.pmin = MIN_ANGLES[index] - BUFFER  # Min Dyna position
        self.pmax = MAX_ANGLES[index] + BUFFER  # Max Dyna position
        # [-pi, pi] range required by ikpy
        self.radians_min = self.dyna_to_radians(self.pmin, zero_to_2pi=False)
        self.radians_max = self.dyna_to_radians(self.pmax, zero_to_2pi=False)
        self.vmin = 1024 - V_RANGE[index] - BUFFER
        self.vmax = 1024 + V_RANGE[index] + BUFFER
        self.loadmin = 1024 - LOAD_RANGE[index] - BUFFER
        self.loadmax = 1024 + LOAD_RANGE[index] + BUFFER
        self.maxtemp = MAX_TEMP
        self.state = None

    def normalized_to_dyna_pos_range(self, value):
        """Takes in a value in [0,1] and converts it to a dyna position value within that particular motors range"""
        return change_scale(old_min=0, old_max=1, new_min=self.pmin, new_max=self.pmax, old_value=value)

    def normalized_to_dyna_vel_range(self, value):
        """Takes in a value in [0,1] and converts it to a dyna velocity value within that particular motors range"""
        return int(change_scale(old_min=0, old_max=1, new_min=self.vmin, new_max=self.vmax, old_value=value))

    def get_normalized_joint_position(self):
        """Converts current dyna position values self.position to a value in [0,1]"""
        normalized_position = change_scale(old_min=self.pmin, old_max=self.pmax, new_min=0, new_max=1,
                                           old_value=self.position)
        return normalized_position

    def get_clamped_dyna_joint_position(self, position):
        """Clamps given value to within allowable range for the particular joint"""
        return clamp_range(min_val=self.pmin, max_val=self.pmax, value=position)

    # TODO MAKE RANGE VALID ( THOUGH IKPY SHOULD HANDLE THIS )
    def dyna_to_radians(self, value, zero_to_2pi=False):
        if zero_to_2pi:
            return change_scale(DYNA_MIN, DYNA_MAX, 0, 2 * pi, value)
        else:
            return change_scale(DYNA_MIN, DYNA_MAX, -pi, pi, value)

    def radians_to_dyna(self, value, normalized=True, zero_to_2pi=False):
        if zero_to_2pi:
            old_min = 0
            old_max = 2 * pi
        else:
            old_min = -pi
            old_max = pi
        if normalized:
            return change_scale(old_min, old_max, 0, 1, value)
        else:
            return change_scale(old_min, old_max, DYNA_MIN, DYNA_MAX, value)


class Robot:
    def __init__(self):
        self.joints = [MxSeries(index=i) for i in range(5)]
        # Hand states in -pi to pi range (mainly needed for IKPY to send end effector)
        self.hand_states = {"closed": -0.15, "mid": 0.24, "open": 1.1428}

    def update_joints_from_packet(self, packet):
        """
        Parses a packet and updates each joints status (Position, Velocity, Load, Temperature also updates Robots
        total velocity used for deciding if robot is moving or still

        :param packet: UDP packet received from brachIOplexus
        :return:
        """
        for i in range(3, packet[2], 9):
            idx = packet[i] - 1  # Packet has ID (which starts at 1) need index
            # Since you can read values outside allowable range when torque is not enabled, best always clamp the value
            self.joints[idx].position = self.joints[idx].get_clamped_dyna_joint_position(
                position=int.from_bytes(packet[i + 1:i + 3], byteorder='little'))
            self.joints[idx].velocity = int.from_bytes(packet[i + 3:i + 5], byteorder='little')
            self.joints[idx].load = int.from_bytes(packet[i + 5:i + 7], byteorder='little')
            self.joints[idx].temp = packet[i + 7]
            self.joints[idx].state = packet[i + 8]

    def build_joints_packet(self, joint_positions: list, velocities=(1024,) * 5, normalized=True):
        """
        Builds a bytearray packet of velocities and positions for each motor to be sent/parsed by the BracIOplexus
        software to create motor commands. This packet structure was developed by us, not brachIOplexus.  It is a simple
        bytearray container two 0xFF headers followed by two bytes for position and two bytes for velocity in a little
        endian format and ending with checksum.  See README.md for breakdown of packet structure.  Lots of safety checks
        are done to ensure the position and velocity commands are correct but will not fix them as this is out of scope.

        :param joint_positions: A collection of 5 values of either [0,4096] or [0,1] representing position of each motor
        :param velocities: Velocities, just use default of 1024 for now, TODO add velocity control
        :param normalized: If normalized joint_positions values should be [0,1] else [0,4096] (raw dynamixel values)
        :return: bytearray of packet to be sent to BrachIOplexus via udp
        """

        # A bunch of safety checks since sending raw values to bento arm
        assert (len(joint_positions) == len(self.joints)), "Invalid positions length, pass for all 5 joints"
        assert (len(velocities) == len(self.joints)), "Invalid velocities length, pass for all 5 joints"

        if normalized:  # [0,1]
            """If normalized, convert to individual motors dyna range"""
            joint_positions = [float(i) for i in joint_positions]
            for i in range(len(self.joints)):
                # Converting normalized to dyna range always ensures it's within allowable range
                # If there are errors in the normalized range, this cannot be fixed here.
                joint_positions[i] = self.joints[i].normalized_to_dyna_pos_range((joint_positions[i]))
            joint_positions = [int(i) for i in joint_positions]
        else:  # [0,4095]
            """If raw values, assert if in range"""
            joint_positions = [int(i) for i in joint_positions]
            for i in range(len(self.joints)):
                # Check positions
                assert (self.joints[i].pmin <= joint_positions[i] <= self.joints[
                    i].pmax), "Make sure servo positions are within valid range"

                # Check velocities
                assert (velocities[i] in range(self.joints[i].vmin, self.joints[
                    i].vmax)), "Make sure servo velocities are within valid range"

        packet = [0xFF, 0xFF, 4 * len(self.joints)]  # Length = 4 bytes (pos + vel) per joint
        for i in range(len(self.joints)):
            # Convert data into little endian bytes
            packet.append(joint_positions[i] & 0xFF)
            packet.append((joint_positions[i] >> 8) & 0xFF)
            packet.append(velocities[i] & 0xFF)
            packet.append((velocities[i] >> 8) & 0xFF)
        packet.append(checksum_fcn(packet[2:]))  # Append checksum function to end
        return bytearray(packet)

    def print_joints(self, normalized=False):
        """
        Prints the array of currently saved joint positions

        :param normalized: If true returns joint values within [0,1] relative to their range, else [pmin, pmax]
        :return:
        """
        for joint in self.joints:
            if normalized:
                print(f"ID {joint.id}: {joint.get_normalized_joint_position():.2f}  ", end="")
            else:
                print(f"ID {joint.id}: {joint.position}  ", end="")
        print("")

    def get_joints(self, normalized=False):
        """
        Returns the array of currently saved joint positions

        :param normalized: If true returns joint values within [0,1] relative to their range, else [pmin, pmax]
        :return:
        """
        if normalized:
            return [joint.get_normalized_joint_position() for joint in self.joints]
        else:
            return [joint.position for joint in self.joints]
