import numpy as np
import socket
import select

def checksum_fcn(packet):
    # The checksum is calculated via the following formula: ~ (LENGTH + FOR EACH SERVO[SERVO ID + PREDICTION], where ~ represents the NOT operation
    summed_packet = sum(packet)
    checksum = ~np.uint8(summed_packet)
    return checksum

class SocketHandler:
    def __init__(self):
        self.portTX = 30006  # The port that this script will send data to
        self.portRX = 30007  # The port that this script will receive data from
        self.udpIP = "127.0.0.1" # The IP address of the computer/programt that you want to send data to. Use 127.0.0.1 when communicating between two programs on the same computer.
        self.bufferSize = 1024  # buffer for incoming bytes
        self.failed_packets = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udpIP, self.portRX))
        self.sock.setblocking(0)

    def read_packet(self):
        result = select.select([self.sock], [], [], 0.0)
        if len(result[0]) == 0:
            return None
        message = result[0][0].recv(self.bufferSize)
        checksum = checksum_fcn(message[2:-1])  # Remove header and checksum
        if message[0] == 0xFF and message[1] == 0xFF and checksum == message[-1]:
            return message
        else:
            self.failed_packets += 1
            return None

    def send_packet(self, packet):
        self.sock.sendto(packet, (self.udpIP, self.portTX))
