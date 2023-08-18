from socket_handler import SocketHandler
from servo import Robot
from threading import Thread
import time
import copy


class Rl:

    def __init__(self):
        self.socket_handler = SocketHandler()
        self.robot = Robot()
        self.thread = Thread(target=self.get_state)
        self.thread.start()
        self.state = None

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

            # Get the normalized robot state from known robot state
            self.state = self.robot.get_joints(normalized=True)  # Add positions to state

            time.sleep(0.001)

    def main(self):

        while True:
            if self.state is None:  # Ensure we get a first state reading
                continue

            # Get most recent state
            action = copy.deepcopy(self.state)  # Ensure it's the state at time of action
            action[0] += 0.05
            packet = self.robot.build_joints_packet(positions=action, normalized=True
                                                    )
            self.socket_handler.send_packet(packet)
            time.sleep(1)


if __name__ == "__main__":
    rl = Rl()
    rl.main()