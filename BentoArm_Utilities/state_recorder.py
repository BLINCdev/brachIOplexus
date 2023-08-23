import csv
from socket_handler import SocketHandler
from robot import Robot
import signal
import datetime


class StateRecorder:

    def __init__(self, normalized=True, print_data=False):
        self.data = []
        self.normalized = normalized
        self.print_data = print_data
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        """Since the program runs continuously in a while True loop you need a signal to handle writing the data to a csv
        and exiting the program once a signal (Ctrl-c) / (Stop Button IDE) is detected"""
        if self.normalized:
            with open(datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S_normalized.csv'), 'w', newline='') as fp:
                w = csv.writer(fp)
                w.writerows(self.data)
        else:
            with open(datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S_dynavalues.csv'), 'w', newline='') as fp:
                w = csv.writer(fp)
                w.writerows(self.data)

        exit(0)

    def main(self):
        socket_handler = SocketHandler()  # Setup UDP connection
        robot = Robot()  # Create Robot object
        input("Press enter to record robot state, press ctrl-c to finish")
        while True:
            packet = socket_handler.read_packet()

            if not packet:
                continue  # If no data received restart the while loop

            robot.update_joints_from_packet(packet)  # Read current joint positions
            if self.print_data:
                robot.print_joints(normalized=self.normalized)
            self.data.append(robot.get_joints(normalized=self.normalized))


if __name__ == "__main__":
    # Printing is not recommended as it will eat up alot of the kernel given the rate you get data
    state_record = StateRecorder(normalized=True, print_data=True)
    state_record.main()
