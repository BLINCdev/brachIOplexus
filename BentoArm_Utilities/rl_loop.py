from socket_handler import SocketHandler
from servo import Robot, change_scale, DYNA_MIN, DYNA_MAX
from threading import Thread
from inverse_kinematics import InverseKinematics
import time
from math import pi
import matplotlib.pyplot as plt


class Rl:

    def __init__(self, normalized=True):
        self.socket_handler = SocketHandler()
        self.robot = Robot()
        self.ik = InverseKinematics()
        self.normalized = normalized
        self.state = None
        self.position_action = None
        self.raw_action = None
        self.thread = Thread(target=self.get_state)
        self.thread.start()

    def get_state(self):
        """Loop for constantly checking if there are packets to receive and updates state making this a thread ensures
        the state is always the state at the time of reading and allows time to pass inbetween actions without building
        up a queue of previous states"""
        while True:
            # Check if packet is available with current state
            packet = self.socket_handler.read_packet()

            if not packet:
                continue

            # Update known robot state using the packet
            self.robot.update_joints_from_packet(packet)  # Read current joint positions

            # Get the normalized/unnormalized robot state from known robot state
            self.state = self.robot.get_joints(normalized=self.normalized)  # Add positions to state

            time.sleep(0.001)

    def get_position_action(self, goal, hand_state="mid"):
        ik_radians = self.ik.inverse_kinematics(goal)[3:8]
        ik_radians[-1] = self.robot.hand_states[hand_state]
        # IKPY returns in [-pi, pi], need in [0,4096] or [0,1]
        if self.normalized:
            # TODO this is broken
            joints = [change_scale(old_min=-pi,
                                   old_max=pi,
                                   new_min=0,
                                   new_max=1,
                                   old_value=ik_radians[i]) for i in range(5)]
            return joints

        else:
            return [change_scale(-pi, pi, DYNA_MIN, DYNA_MAX, i) for i in ik_radians]

    def get_position_xyz(self):
        """I Think this might be broken"""
        if self.normalized:
            joints_to_ik = [change_scale(old_min=0,
                                         old_max=1,
                                         new_min=self.robot.joints[i].radians_min,
                                         new_max=self.robot.joints[i].radians_max,
                                         old_value=self.state[i]) for i in range(4)]
            return self.ik.forward_kinematics(joints_to_ik)
        else:
            joints_to_ik = [change_scale(old_min=DYNA_MIN,
                                         old_max=DYNA_MAX,
                                         new_min=-pi,
                                         new_max=pi,
                                         old_value=self.state[i]) for i in range(4)]
            return self.ik.forward_kinematics(joints_to_ik)

    def main(self):
        """Basic example that sets a series of goal states, and prints out the target vs. actual position currently does
        not work using normalized values"""

        while not self.state:  # Ensure we get a first state reading
            pass

        goal_states = ((50, 0, 15),
                       (29, 0, 45),
                       (31, -10, 40),
                       (31, -20, 30),
                       (31, -30, 20),
                       (31, -20, 30),
                       (31, -10, 40),
                       (29, 0, 45),
                       (31, 10, 40),
                       (31, 20, 30),
                       (40, 10, 25),
                       (45, 5, 20),
                       (50, 0, 15))

        for goal in goal_states:
            print(f"Previous State {self.get_position_xyz()}")
            print(f"Goal State: {goal}")
            action = self.get_position_action(goal)
            print(f"Action {action}")
            packet = self.robot.build_joints_packet(action, normalized=self.normalized)
            self.socket_handler.send_packet(packet)
            time.sleep(3)
            print(f"New State {self.get_position_xyz()}")
            print("----------------------")

        plt.show()


if __name__ == "__main__":
    rl = Rl(normalized=False)
    rl.main()
