import csv
from socket_handler import SocketHandler
from robot import Robot
import signal
import datetime
import time


class StateRecorder:

    def __init__(self, normalized=True, print_data=False, socket_handler=None):
        self.data = []
        self.socket_handler = socket_handler
        self.robot = Robot(normalized=normalized)  # Create Robot object
        self.normalized = normalized
        self.print_data = print_data
        if normalized:
            self.filename = datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S_normalized.csv')
        else:
            self.filename = datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S_dynavalues.csv')
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        """Since the program runs continuously in a while True loop you need a signal to handle writing the data to a csv
        and exiting the program once a signal (Ctrl-c) / (Stop Button IDE) is detected"""
        with open(self.filename, 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerows(self.data)
        exit(0)

    def main(self):

        input("Press enter to record robot state, press ctrl-c to finish")
        while True:
            packet = self.socket_handler.read_packet()

            if not packet:
                continue  # If no data received restart the while loop

            self.robot.update_joints_from_packet(packet)  # Read current joint positions
            if self.print_data:
                self.robot.print_joints()
            self.data.append(self.robot.get_joints())

    def test(self):
        """Records state for 10 seconds"""
        input("Press enter to record robot state, press ctrl-c to finish")
        current_time = time.time()
        while time.time() - current_time < 10:
            packet = self.socket_handler.read_packet()

            if not packet:
                continue  # If no data received restart the while loop

            self.robot.update_joints_from_packet(packet)  # Read current joint positions
            if self.print_data:
                self.robot.print_joints()
            self.data.append(self.robot.get_joints())


if __name__ == "__main__":
    # Printing is not recommended as it will eat up alot of the kernel given the rate you get data
    state_record = StateRecorder(normalized=False, print_data=False, socket_handler=SocketHandler())
    state_record.main()
