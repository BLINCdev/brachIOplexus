from state_player import playback
from state_recorder import StateRecorder
from socket_handler import SocketHandler
from inverse_kinematics import test_ik
import csv
import os


def main():
    """Test inverse kinematics"""
    test_ik()

    sock = SocketHandler()

    """Test normalized joint_positions recording and playback"""
    recorder = StateRecorder(normalized=True, print_data=False, socket_handler=sock)
    recorder.test()
    recorder.robot.stop_reading_thread()

    with open(recorder.filename, 'w', newline='') as fp:
        w = csv.writer(fp)
        w.writerows(recorder.data)

    playback(file=recorder.filename, normalized=True, socket_handler=sock)
    os.remove(recorder.filename)

    """Test dyna values joint_positions recording and playback"""
    sock.sock.close()
    sock2 = SocketHandler()

    recorder2 = StateRecorder(normalized=False, print_data=False, socket_handler=sock2)
    recorder2.test()
    recorder2.robot.stop_reading_thread()

    with open(recorder2.filename, 'w', newline='') as fp:
        w = csv.writer(fp)
        w.writerows(recorder2.data)

    playback(file=recorder2.filename, normalized=False, socket_handler=sock2)
    os.remove(recorder2.filename)


if __name__ == "__main__":
    main()
