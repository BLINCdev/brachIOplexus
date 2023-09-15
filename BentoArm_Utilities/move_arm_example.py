from socket_handler import SocketHandler
from inverse_kinematics import  InverseKinematics
from robot import Robot
import time

socket = SocketHandler()
robot = Robot(normalized=True)
ik = InverseKinematics(robot_obj=robot)

positions = [0.5, 0.5, 0.5, 0.5, 0.5]
socket.send_packet(robot.build_joints_packet(joint_positions=positions))

time.sleep(3)

goal = [52.7, 0, 15.55]
positions = ik.get_joints_for_goal_xyz(goal, hand_state='open')
socket.send_packet(robot.build_joints_packet(joint_positions=positions))

time.sleep(3)

goal = [52.7, 0, 19.55]
positions = ik.get_joints_for_goal_xyz(goal, hand_state='closed')
socket.send_packet(robot.build_joints_packet(joint_positions=positions))

time.sleep(3)

goal = [51, 0, 25]
positions = ik.get_joints_for_goal_xyz(goal, hand_state='mid')
socket.send_packet(robot.build_joints_packet(joint_positions=positions))