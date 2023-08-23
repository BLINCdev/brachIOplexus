import csv
from socket_handler import SocketHandler
from robot import Robot
import sys
import os.path
import time


def main(file, normalized=True):
    socket_handler = SocketHandler()  # Setup UDP socket communication
    robot = Robot()  # Setup Robot
    input("Press enter to playback csv file, ensure you've enabled Torque On in BracIOplexus Bento Arm Menu")

    done_check = False  #
    with open(file, mode='r') as fp:
        csv_file = csv.reader(fp)  # Read CSV File

        for state in csv_file:
            if not done_check:
                if normalized:
                    assert (0 <= float(
                        state[0]) <= 1), "Value read not in normalized range, did you set normalized correctly"
                else:
                    assert (int(float(
                        state[0])) > 1), "Value read not in dyna motor range, did you set normalized correctly?"
                done_check = True
            packet = robot.build_joints_packet(joint_positions=state, normalized=normalized)
            socket_handler.send_packet(packet)
            time.sleep(0.01)  # Approx 10 ms to allow time for some movements


if __name__ == "__main__":
    if (len(sys.argv)) != 2:
        print(
            "Please provide name of file you wish to playback such as: \npython state_player.py 2023_07_21_123316.csv")
        exit()

    assert (os.path.isfile(sys.argv[1])), f"{sys.argv[1]} Does not exist"  # Check if file given actually exists
    main(normalized=False, file=sys.argv[1])