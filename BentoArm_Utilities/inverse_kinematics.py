from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from robot import Robot
from mpl_toolkits.mplot3d import Axes3D
from math import pi
import matplotlib.pyplot as plt


class InverseKinematics:

    def __init__(self, robot=None):
        """This is the chain/urdf of the arm used for forward and inverse kinematics with the final end effector
        being the center of the end of the fixed chopstick.  All measurements are in centimeters and have been measured
        using the solidworks model"""

        # If robot is not passed, create a default one
        if robot is None:
            self.robot = Robot()

        self.bento_chain = Chain(name='left_arm', links=[
            OriginLink(),
            URDFLink(
                name="base_to_post",
                origin_translation=[0, 0, 32],
                origin_orientation=[0, 0, 0],
                joint_type="fixed"
            ),
            URDFLink(
                name="extender",
                origin_translation=[12, 0, 0],  # Careful this is set by the user
                origin_orientation=[0, 0, 0],
                joint_type="fixed",
            ),
            URDFLink(
                name="shoulder",
                origin_translation=[0, 0, -6.99],
                origin_orientation=[pi, 0, 0],
                joint_type="revolute",
                rotation=[0, 0, 1],
                bounds=(self.robot.joints[0].radians_min, self.robot.joints[0].radians_max)
            ),
            URDFLink(
                name="shoulder_to_elbow",
                origin_translation=[2.9, 0, 12.503],
                origin_orientation=[0, 0, 0],
                joint_type="revolute",
                rotation=[0, 1, 0],
                bounds=(self.robot.joints[1].radians_min, self.robot.joints[1].radians_max)
            ),
            URDFLink(
                name="elbow_to_forearm",
                origin_translation=[15.123, 0, -0.225],
                origin_orientation=[0, 0, 0],
                joint_type="revolute",
                rotation=[1, 0, 0],
                bounds=(self.robot.joints[2].radians_min, self.robot.joints[2].radians_max)
            ),
            URDFLink(
                name="forearm_to_wrist",
                origin_translation=[4.38, 0, 0.065],
                origin_orientation=[0, 0, 0],
                joint_type="revolute",
                rotation=[0, 1, 0],
                bounds=(self.robot.joints[3].radians_min, self.robot.joints[3].radians_max)
            ),
            URDFLink(
                name="wrist_to_hand",
                origin_translation=[8.43, 0, 0.073],
                origin_orientation=[pi, 0, 0],
                joint_type="fixed",
            ),
            URDFLink(
                name="hand_to_finger",
                origin_translation=[10.52, -0.092, 0],
                origin_orientation=[0, 0, 0],
                joint_type="fixed",
            ), ], active_links_mask=[False, False, False, True, True, True, True, False, False])

        # Ax for plotting
        self.ax = plt.figure().add_subplot(111, projection='3d')
        self.ax.axes.set_xlim3d(left=0, right=55)
        self.ax.axes.set_ylim3d(bottom=-22.5, top=22.5)
        self.ax.axes.set_zlim3d(bottom=0, top=55)

        # Safety params
        self.max_error = 0.1

    @staticmethod
    def fill_state(state):
        """If a state of 4 values is passed, it will fill in all the fixed state values of 0"""
        # Default full state value
        if len(state) == 9:
            return state
        # Else append 4 joint state to nine joint state to account for fixed joints
        for i in range(3):
            state.insert(0, None)
        for i in range(2):
            state.append(None)
        return state

    def plot_state(self, state=[0, 0, 0, 0]):
        """
        Plots the Bento Arm in the given state in 3D

        :param state: 0 -> shoulder  1 -> elbow  2 -> forearm  3 -> wrist  (chopstick not part of state)
        :return: None
        """
        self.bento_chain.plot(self.fill_state(state), self.ax)
        plt.show(block=False)

    def forward_kinematics(self, state=[0, 0, 0, 0], matrix=False):
        position = self.bento_chain.forward_kinematics(self.fill_state(state))
        if matrix:
            # Return homogenous transform matrix
            return position
        else:
            # Return X Y Z coordinates
            return position[0][3], position[1][3], position[2][3]

    def inverse_kinematics(self, target=(0, 0, 0), plot=True):
        joints = self.bento_chain.inverse_kinematics(target)
        forward = self.forward_kinematics(joints)
        difference = []
        for i in range(3):
            difference.append(abs(target[i] - forward[i]))
        if sum(difference) > self.max_error:
            input("WARNING STATE UNREACHABLE")
        if plot:
            self.plot_state(joints)
        return joints


def test():
    # Basic test doing both forward and backwards kinematics assuring both give the same value
    ik = InverseKinematics()
    ik.plot_state([0, 0, 0, 0])
    position = ik.forward_kinematics([0, 0, 0, 0])
    print(f'Forward Kinematics Position: {position}')
    print(f'Target: {position}')
    state = ik.inverse_kinematics(target=(position))
    print(f'Required State Radians {state[3:7]}')
    position = ik.forward_kinematics(state=state)
    print(f'Forward Kinematics Position: {position}')
    ik.plot_state(state=state)
    plt.show()


if __name__ == "__main__":
    test()
