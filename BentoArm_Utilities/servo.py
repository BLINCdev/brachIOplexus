from socket_handler import checksum_fcn
from math import pi

MIN_ANGLES = [1028, 1784, 1028, 790, 1928]
MAX_ANGLES = [3073, 2570, 3073, 3328, 2800]
V_RANGE = [55, 45, 90, 67, 90]
LOAD_RANGE = [225, 300, 250, 300, 400]
MAX_TEMP = 80
BUFFER = 10
DYNA_MIN = 0
DYNA_MAX = 4096

def change_scale(old_min, old_max, new_min, new_max, old_value):
    return (((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min
class ServoInfo(object):
    # Abstract class to hold basic implementation for servo related information
    def __init__(self):
        self.id = None
        self.position = 0  # The current position value of the servo
        self.velocity = 0  # The current moving speed of the servo
        self.load = 0  # The current load applied to the servo
        self.temp = 0  # The internal temperature of the servo in deg


class MxSeries(ServoInfo):
    # Specific instance of all servos of the mx series type (i.e mx-28, mx-64, mx-106)
    # * Position: 0 - 4095 (0.088 degrees/value, The central neutral position for each servo is 2048)
    # * Velocity: 0 - 2047 (0.11rpm/value, If the value reads 1024 then it means that the  servo is stationary. A value below 1023 means the motor rotates in a CCW direction and a value above 1023 means the motor rotates in a CW direction.)
    # * Load: 0 - 2047 (0.1% load/value, If the value is 0 - 1023 it means the load works to the CCW direction and if the value is 1024-2047 it means the load works in the CW direction. A value of 1024 means that there is 0% load. )
    # * Temp: 0 - 255 (1 degC/value)
    # * State: 0 - 3 (see above for definitions of each state)
    def __init__(self, index):
        super().__init__()
        self.id = index + 1  # ID start at 1, index starts at 0
        self.pmin = MIN_ANGLES[index] - BUFFER
        self.pmax = MAX_ANGLES[index] + BUFFER
        # [-pi, pi] range required by ikpy
        self.radians_min = self.dyna_to_radians(self.pmin, zero_to_2pi=False)
        self.radians_max = self.dyna_to_radians(self.pmax, zero_to_2pi=False)
        self.vmin = 1024 - V_RANGE[index] - BUFFER
        self.vmax = 1024 + V_RANGE[index] + BUFFER
        self.loadmin = 1024 - LOAD_RANGE[index] - BUFFER
        self.loadmax = 1024 + LOAD_RANGE[index] + BUFFER
        self.maxtemp = MAX_TEMP
        self.state = None


    def normalized_position(self):
        return max(0, min((self.position - self.pmin) / (self.pmax - self.pmin), 1))

    def unnormalized_position(self, value):
        return int(max(self.pmin, min(value * (self.pmax - self.pmin) + self.pmin, self.pmax)))



    # TODO MAKE RANGE VALID ( THOUGH IKPY SHOULD HANDLE THIS )
    def dyna_to_radians(self, value, zero_to_2pi=False):
        if zero_to_2pi:
            return change_scale(DYNA_MIN, DYNA_MAX,0, 2 * pi, value)
        else:
            return change_scale(DYNA_MIN, DYNA_MAX, -pi, pi, value)

    def radians_to_dyna(self, value, normalized=True, zero_to_2pi=False):
        if zero_to_2pi:
            old_min = 0
            old_max = 2*pi
        else:
            old_min = -pi
            old_max = pi
        if normalized:
            return change_scale(old_min, old_max, 0,1, value)
        else:
            return change_scale(old_min, old_max, DYNA_MIN, DYNA_MAX, value)


class Robot:
    def __init__(self):
        self.joints = [MxSeries(index=i) for i in range(5)]
        # Hand states in -pi to pi range
        self.hand_states = {"closed": -0.15, "mid": 0.24, "open": 1.1428}
        # TODO a nice feature would to get the actual current movement speed from Dynamixel so we can do actions until velocity is near zero
    def update_joints_from_packet(self, packet):
        self.total_velocity = 0
        for i in range(3, packet[2], 9):
            idx = packet[i] - 1  # Packet has ID (which starts at 1) need index
            self.joints[idx].position = int.from_bytes(packet[i + 1:i + 3], byteorder='little')
            self.joints[idx].velocity = int.from_bytes(packet[i + 3:i + 5], byteorder='little')
            self.joints[idx].load = int.from_bytes(packet[i + 5:i + 7], byteorder='little')
            self.joints[idx].temp = packet[i + 7]
            self.joints[idx].state = packet[i + 8]

    def build_joints_packet(self, positions: list, velocities=(1024,) * 5, normalized=True):

        # A bunch of safety checks since sending raw values to bento arm
        assert (len(positions) == len(self.joints)), "Invalid positions length, pass for all 5 joints"
        assert (len(velocities) == len(self.joints)), "Invalid velocities length, pass for all 5 joints"

        for i in range(len(self.joints)):
            # Assert the velocities are in allowable range, velocity of 0 = max velocity = bad
            assert (velocities[i] in range(1024 - V_RANGE[i] - BUFFER, 1024 + V_RANGE[
                i] + BUFFER)), "Make sure servo velocities are within valid range"

        if normalized:
            positions = [float(i) for i in positions]
            for i in range(len(self.joints)):
                positions[i] = self.joints[i].unnormalized_position((positions[i]))
        else:
            positions = [int(i) for i in positions]
            for i in range(len(self.joints)):
                if positions[i] not in range(MIN_ANGLES[i] - BUFFER, MAX_ANGLES[
                                                                         i] + BUFFER):
                    print("Warning position out of range, correcting")
                    positions[i] = max(self.joints[i].pmin, min(positions[i], self.joints[i].pmax))

        packet = [0xFF, 0xFF, 4 * len(self.joints)]  # Length = 4 bytes (pos + vel) per joint
        for i in range(len(self.joints)):
            # Convert data into little endian bytes
            packet.append(positions[i] & 0xFF)
            packet.append((positions[i] >> 8) & 0xFF)
            packet.append(velocities[i] & 0xFF)
            packet.append((velocities[i] >> 8) & 0xFF)
        packet.append(checksum_fcn(packet[2:]))  # Append checksum function to end
        return bytearray(packet)

    def print_joints(self, normalized=False):
        for joint in self.joints:
            if normalized:
                print(f"ID {joint.id}: {joint.normalized_position():.2f}  ", end="")
            else:
                print(f"ID {joint.id}: {joint.position}  ", end="")
        print("")

    def get_joints(self, normalized=False):
        if normalized:
            return [joint.normalized_position() for joint in self.joints]
        else:
            return [joint.position for joint in self.joints]
