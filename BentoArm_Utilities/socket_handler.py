import numpy as np
import socket
import select


def checksum_fcn(packet):
    """The checksum is calculated via summing up all the bytes and taking the bitwise complement (~/not operator) of
    the sum cast to an uint8 / char / 8 bits, the header (first two bytes being 0xFF) and checksum (last bytes) are not
    used for the checksum"""
    summed_packet = sum(packet)
    checksum = ~np.uint8(summed_packet)
    return checksum


class SocketHandler:
    def __init__(self):
        self.portTX = 30006  # The port that this script will send data to
        self.portRX = 30007  # The port that this script will receive data from
        self.udpIP = "127.0.0.1"  # The IP address of the computer/program that you want to send data to. Use 127.0.0.1 when communicating between two programs on the same computer.
        self.bufferSize = 1024  # Buffer size for incoming bytes, this is pretty standard
        self.failed_packets = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udpIP, self.portRX))
        self.sock.setblocking(False)  # We don't want RX to block anything

    def read_packet(self):
        """Receives a packet, checks the checksum, and returns the message"""
        result = select.select([self.sock], [], [], 0.0)
        if len(result[0]) == 0:
            return None
        message = result[0][0].recv(self.bufferSize)
        checksum = checksum_fcn(message[2:-1])  # Remove header and checksum
        if message[0] == 0xFF and message[1] == 0xFF and checksum == message[-1]:
            # If header is correct and checksum is passed return the message
            return message
        else:
            self.failed_packets += 1
            return None

    def send_packet(self, packet):
        """Sends the packet, typically in a Bento arm packet message"""
        self.sock.sendto(packet, (self.udpIP, self.portTX))
