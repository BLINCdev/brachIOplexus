from helper_functions import clamp_range, change_scale, checksum_fcn
from math import pi

MIN_ANGLES = [1028, 1784, 1028, 790, 1928]
MAX_ANGLES = [3073, 2570, 3073, 3328, 2800]
V_RANGE = [55, 45, 90, 67, 90]
LOAD_RANGE = [225, 300, 250, 300, 400]
MAX_TEMP = 80
BUFFER = 10
DYNA_MIN = 0
DYNA_MAX = 4095


class Robot:
    def __init__(self, normalized=True):
        """
        Class for BentoArm which is just a collection of MxSeries joints.  There are two playback ways we can represent a
        joint state.

        - normalized state: position is in range [0,1], useful as it derives nicely for deep learning applications
        - dynamixel state: standard representation used by dynamixel [0,4095]

        Since there are many ways to represent states the code avoids doing as many state conversions as possible
        putting the onus on the user to change ranges as needed.  There are some functions that work in the range of
        [-π,π] as this is the range used by the inverse kinematics library IKPy.

        Args:
            normalized:  If true joint state is represented via a [0, 1] range else the dynamixel range [0,4095]

        Attributes:
            joints (list): A list of MxSeries joints
            hand_states (dict): Hand states in the range of [-π,π] mainly just used by IKPY

        Examples:
        ```python
        from robot import Robot
        from helper_functions import change_scale
        from math import radians, pi
        from socket_handler import SocketHandler


        sock = SocketHandler

        robot.update_joints_from_packet(sock.read_packet())  # Typically done in a thread

        # Get State, Build & Send Action (Rotate Shoulder by 45 Deg.) [NORMALIZED]
        robot = Robot(normalized=True)
        state = robot.get_joints(normalized = True)
        state[0] += change_scale(old_min = 0,
                                 old_max = 2*pi,
                                 new_min = 0,
                                 new_max = 1,
                                 value = radians(45)
        sock.send_packet(robot.build_joints_packet(joint_positions=state, normalized=True)

        # Get State, Build & Send Action (Rotate Shoulder by 45 Deg.) [DYNA]
        robot = Robot(normalized=False)
        state = robot.get_joints(normalized = FALSE)
        state[0] += change_scale(old_min = 0,
                                 old_max = 360,
                                 new_min = DYNA_MIN,
                                 new_max = DYNA_MAX,
                                 value = 45
        sock.send_packet(robot.build_joints_packet(joint_positions=state, normalized=False)
        ```
        """
        self.normalized = normalized
        self.joints = [MxSeries(index=i) for i in range(5)]
        self.hand_states = {"closed": -0.15, "mid": 0.24, "open": 1.1428}

    def update_joints_from_packet(self, packet):
        """
        Parses a packet and updates each joints status (Position, Velocity, Load, Temperature also updates Robots
        total velocity used for deciding if robot is moving or still

        Args:
            packet (bytearray): UDP packet received from brachIOplexus

        Returns:
            None
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

    def build_joints_packet(self, joint_positions, velocities=(1024,) * 5):
        """
        Builds a bytearray packet of velocities and positions for each motor to be sent/parsed by the BracIOplexus
        software to create motor commands. This packet structure was developed by us, not brachIOplexus.  It is a simple
        bytearray container two 0xFF headers followed by two bytes for position and two bytes for velocity in a little
        endian format and ending with checksum.  See README.md for breakdown of packet structure.  Lots of safety checks
        are done to ensure the position and velocity commands are correct but will not fix them as this is out of scope.

        Args:
            joint_positions (list): A collection of 5 values of either [0,4096] or [0,1] representing position of each motor
            velocities (tuple): Velocities, just use default of 1024 for now

        Returns:
            bytearray: packet to be sent to BrachIOplexus via udp

        Todo:
            This yet supports velocity control, just using default values for now
        """

        # Safety checks since sending raw values to bento arm can be dangerous
        assert (len(joint_positions) == len(self.joints)), "Invalid positions length, pass for all 5 joints"
        assert (len(velocities) == len(self.joints)), "Invalid velocities length, pass for all 5 joints"

        if self.normalized:  # [0,1]
            """If normalized, convert to individual motors dyna range"""
            joint_positions = [float(i) for i in joint_positions]
            for i in range(len(self.joints)):
                joint_positions[i] = self.joints[i].normalized_to_dyna_pos_range((joint_positions[
                    i]))  # Converting normalized to dyna range always ensures it's within allowable range
            joint_positions = [int(i) for i in joint_positions]
        else:  # [0,4095]
            """If raw values, assert if in range"""
            joint_positions = [int(i) for i in joint_positions]
            for i in range(len(self.joints)):
                # Check positions
                assert (self.joints[i].position_min <= joint_positions[i] <= self.joints[
                    i].postion_max), "Make sure servo positions are within valid range"

                # Check velocities
                assert (velocities[i] in range(self.joints[i].velocity_min, self.joints[
                    i].velocity_max)), "Make sure servo velocities are within valid range"

        packet = [0xFF, 0xFF, 4 * len(self.joints)]  # Length = 4 bytes (pos + vel) per joint
        for i in range(len(self.joints)):
            # Convert data into little endian bytes
            packet.append(joint_positions[i] & 0xFF)
            packet.append((joint_positions[i] >> 8) & 0xFF)
            packet.append(velocities[i] & 0xFF)
            packet.append((velocities[i] >> 8) & 0xFF)
        packet.append(checksum_fcn(packet[2:]))  # Append checksum function to end
        return bytearray(packet)

    def print_joints(self):
        """
        Prints the array of Robots current joint positions

        Returns:
            None

        """

        for joint in self.joints:
            if self.normalized:
                print(f"ID {joint.id}: {joint.get_normalized_joint_position():.2f}  ", end="")
            else:
                print(f"ID {joint.id}: {joint.position}  ", end="")
        print("")

    def get_joints(self):
        """
        Returns:
            list: current joint positions for Robot
        """

        if self.normalized:
            return [joint.get_normalized_joint_position() for joint in self.joints]
        else:
            return [joint.position for joint in self.joints]


class ServoInfo(object):
    """
    Abstract class to hold basic implementation for any servo related information

    Attributes:
        id (int): ID Number representing motor in bento arm, ID's are in range [1, # Of Joints]
        position (int): Position of joint in dynamixel range [0, 4095]
        velocity (int): Velocity for joint in dynamix range 1024 +- x
        load (int): Load / Torque of joint
        temp (int): Temperature of joint
    """

    def __init__(self):
        self.id = None
        self.position = None  # The current position value of the servo
        self.velocity = None  # The current moving speed of the servo
        self.load = None  # The current load applied to the servo
        self.temp = None  # The internal temperature of the servo in deg


class MxSeries(ServoInfo):
    """
    Specific instance of all servos of the mx series type (i.e mx-28, mx-64, mx-106)
    """

    def __init__(self, index):
        """
        Args:
            index: Index of motor in [0, # of Joints - 1]

        Attributes:
            position_min (int): Min position for joint in dynamixel range [0,4095]
            position_max (int): Max position for joint in dynamixel range [0,4095]
            radians_min (float): Min position for joint in radians by default in [-π, π]
            radians_max (float): Max position for joint in radians by default in [-π, π]
            velocity_min (int): Min velocity for joint in dynamix range 1024 - x
            velocity_max (int): Min velocity for joint in dynamix range 1024 + x
            load_min (int): Min effort / load / torque
            load_max (int): Max effort / load / torque
            max_temp (int): Default (80)

        """
        super().__init__()
        self.id = index + 1
        self.position_min = MIN_ANGLES[index] - BUFFER
        self.postion_max = MAX_ANGLES[index] + BUFFER
        self.radians_min = self.dyna_to_radians(self.position_min, zero_to_2pi=False)
        self.radians_max = self.dyna_to_radians(self.postion_max, zero_to_2pi=False)
        self.velocity_min = 1024 - V_RANGE[index] - BUFFER
        self.velocity_max = 1024 + V_RANGE[index] + BUFFER
        self.load_min = 1024 - LOAD_RANGE[index] - BUFFER
        self.load_max = 1024 + LOAD_RANGE[index] + BUFFER
        self.max_temp = MAX_TEMP
        self.state = None

    def normalized_to_dyna_pos_range(self, value):
        """Takes in a value in [0,1] and converts it to a dyna position value within that particular motors range"""
        return change_scale(old_min=0, old_max=1, new_min=self.position_min, new_max=self.postion_max, value=value)

    def normalized_to_dyna_vel_range(self, value):
        """Takes in a value in [0,1] and converts it to a dyna velocity value within that particular motors range"""
        return int(
            change_scale(old_min=0, old_max=1, new_min=self.velocity_min, new_max=self.velocity_max, value=value))

    def get_normalized_joint_position(self):
        """Converts current dyna position values self.position to a value in [0,1]"""
        normalized_position = change_scale(old_min=self.position_min, old_max=self.postion_max, new_min=0, new_max=1,
                                           value=self.position)
        return normalized_position

    def get_clamped_dyna_joint_position(self, position):
        """Clamps given value to within allowable range for the particular joint"""
        return clamp_range(min_val=self.position_min, max_val=self.postion_max, value=position)

    def dyna_to_radians(self, value, zero_to_2pi=False):
        """
        Converts a dyna value in range [0,4095] to radians range [-π,π] or [0,2π]
        Args:
            value (int): Dynamixel position value
            zero_to_2pi (bool): If true will convert to range [0,2π] else [-π,π]

        Returns:
            Dynamixel position in radians range of choice
        """
        if zero_to_2pi:
            return change_scale(DYNA_MIN, DYNA_MAX, 0, 2 * pi, value)
        else:
            return change_scale(DYNA_MIN, DYNA_MAX, -pi, pi, value)

    def radians_to_dyna(self, value, normalized=True, zero_to_2pi=False):
        """
        Converts a radians value in [-π,π] or [0,2π] to a dyna range in [0,4095]

        Args:
            value (float): radians value to be converted either in [0,2π] else [-π,π]
            normalized (bool): If true returns value in [0,1] else [0,4095]
            zero_to_2pi (bool): If true will convert from range [0,2π] else [-π,π]

        Returns:

        """
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
