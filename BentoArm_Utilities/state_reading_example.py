from robot import Robot
from socket_handler import SocketHandler
from inverse_kinematics import InverseKinematics
import time

robot = Robot(normalized=True)
socket = SocketHandler()
ik = InverseKinematics(robot_obj=robot)
robot.start_reading_thread(socket_handler=socket)

while True:
    joint_positions = robot.get_joint_positions()
    for joint_pos in joint_positions:
        print(f"{joint_pos:.3f} ", end='')
    print(f"\n {ik.get_end_effector_position_xyz()} \n")
    time.sleep(0.2)

robot.stop_reading_thread()