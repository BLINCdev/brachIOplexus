import csv
from socket_handler import SocketHandler
from robot import Robot
import signal
import datetime
import time

RATE = 1 / 300  # rate = 1 / hz

class StateRecorder:

    def __init__(self, normalized=True, print_data=False, socket_handler=None):
        self.data = []
        self.socket_handler = socket_handler
        self.robot = Robot(normalized=normalized)  # Create Robot object
        self.robot.start_reading_thread(socket_handler=socket_handler)  # Start getting readings
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
        print("Signal Detected. Stopping joint read thread and exiting...")
        self.robot.stop_reading_thread()
        with open(self.filename, 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerows(self.data)
        exit(0)

    def main(self):
        input("Press enter to record robot_obj joint_positions, press ctrl-c to finish")
        while True:
            if self.print_data:
                self.robot.print_joints()
            self.data.append(self.robot.get_joint_positions())
            time.sleep(RATE)

    def test(self):
        """Records joint_positions for 10 seconds"""
        input(f"Press enter to record robot_obj joint_positions: Normalized={self.robot.normalized}")
        current_time = time.time()
        while time.time() - current_time < 10:
            if self.print_data:
                self.robot.print_joints()
            self.data.append(self.robot.get_joint_positions())
            time.sleep(RATE)


if __name__ == "__main__":
    # Printing is not recommended as it will eat up alot of the kernel given the rate you get data
    state_record = StateRecorder(normalized=True, print_data=False, socket_handler=SocketHandler())
    state_record.main()
