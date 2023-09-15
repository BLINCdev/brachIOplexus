from robot import Robot
from socket_handler import SocketHandler
from inverse_kinematics import InverseKinematics
import time

robot = Robot(normalized=True)
socket = SocketHandler()
ik = InverseKinematics(robot_obj=robot)
robot.start_reading_thread(socket_handler=socket)

while True:
    joints = robot.get_joint_positions()
    if None not in joints:
        for joint in joints:
            print(f"{joint:.3f} ", end='')
        print(f"\n {ik.get_end_effector_position()} \n")
    time.sleep(0.2)
