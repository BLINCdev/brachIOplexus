from socket_handler import SocketHandler
from robot import Robot, DYNA_MIN, DYNA_MAX
from helper_functions import change_scale
from threading import Thread
from inverse_kinematics import InverseKinematics
import time
from math import pi
import matplotlib.pyplot as plt


class Rl:

    def __init__(self, normalized=True):
        self.socket_handler = SocketHandler()
        self.robot = Robot(normalized=normalized)
        self.robot.start_reading_thread(socket_handler=self.socket_handler)
        self.ik = InverseKinematics(robot_obj=self.robot)

    def get_full_state(self):
        """
        Returns:
            Full State is in the following list structure

            [
            [dyna_state] # 5 joint states in range [0,4095]
            [normalized_state] # 5 joint states in range [0,1]
            [end effector position]  # XYZ position in cm
            ]
        """

        dyna_state = self.robot.get_joint_positions(normalized=False)
        normalized_state = self.robot.get_joint_positions(normalized=True)
        end_effector_pos = self.ik.get_end_effector_position()

        return dyna_state, normalized_state, end_effector_pos

    def example(self):
        """
        Basic example of creating a basic action and sending it and also a RL loop of reading a state, getting a goal
        state and doing the action to get to that goal state.
        """

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

        # Manually change state
        if self.robot.normalized:
            # Important to remember that 0.5 doesn't mean centered, this is centered within it's possible range
            action = [0.5, 0.5, 0.5, 0.5, 0.5]
        else:
            # These are the raw centered values
            action = [2048, 2048, 2048, 2048, 2048]
        packet = self.robot.build_joints_packet(action)
        self.socket_handler.send_packet(packet)
        time.sleep(2)

        for goal in goal_states:
            dyna_state, normalized_state, end_effector_pos = self.get_full_state()

            # STATE
            print(f"Previous State {end_effector_pos}")
            print(f"Goal State: {goal}")

            # ACTION
            action = self.ik.get_joints_for_goal_xyz(goal)
            print(f"Action {action}")
            packet = self.robot.build_joints_packet(action)
            self.socket_handler.send_packet(packet)
            time.sleep(2)

            # NEW STATE
            print(f"New State {self.ik.get_end_effector_position()}")
            print("----------------------")

        self.robot.stop_reading_thread()
        plt.show()


if __name__ == "__main__":
    rl = Rl(normalized=True)
    rl.example()
