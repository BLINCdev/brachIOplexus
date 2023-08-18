import csv
from socket_handler import SocketHandler
from servo import Robot
import sys
import os.path
import time

data = []

try:
    if (len(sys.argv)) != 2:
        print(
            "Please provide name of file you wish to playback such as: \npython state_player.py 2023_07_21_123316.csv")
        print(sys.argv[0])
        exit()

    assert (os.path.isfile(sys.argv[1])), f"{sys.argv[1]} Does not exist"

    socket_handler = SocketHandler()
    robot = Robot()
    input("Press enter to playback csv file")

    # opening the CSV file
    with open(sys.argv[1], mode='r') as fp:
        pass

        csv_file = csv.reader(fp)

        for line in csv_file:
            packet = robot.build_joints_packet(positions=line, normalized=True)
            socket_handler.send_packet(packet)
            time.sleep(0.005)  # Approx 5 ms

except Exception as e:
    print(e)
