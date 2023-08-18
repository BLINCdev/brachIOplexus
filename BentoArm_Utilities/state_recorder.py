import csv
from socket_handler import SocketHandler
from servo import Robot
import signal
import datetime

data = []


def exit_gracefully(signum, frame):
    with open(datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S.csv'), 'w', newline='') as fp:
        w = csv.writer(fp)
        w.writerows(data)
    exit(0)


signal.signal(signal.SIGINT, exit_gracefully)
signal.signal(signal.SIGTERM, exit_gracefully)

try:
    socket_handler = SocketHandler()
    robot = Robot()
    input("Press enter to record robot state, press ctrl-c to finish")
    while True:
        packet = socket_handler.read_packet()

        if not packet:
            continue

        robot.update_joints_from_packet(packet)  # Read current joint positions
        #robot.print_joints(normalized=False)
        data.append(robot.get_joints(normalized=True))

except Exception as e:
    print(e)
