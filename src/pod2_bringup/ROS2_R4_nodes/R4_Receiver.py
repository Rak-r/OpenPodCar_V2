#!/usr/bin/env python3

#################################################################
# Author        : Andy Perrett                                  #
# Email         : 18684092@students.lincoln.ac.uk               #
# Date          : 18th August 2023                              #
# Description   : Connect to R4 via websockets                  #
#                 Sends commands to R4. Commands come in via    #
#                 /R4_Command topic                             #
################################################################# 

# Test with 
# ros2 topic pub /R4_Command std_msgs/String "data: S:Stop" -1


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import socket
import time
import zlib

from supportFunctions import myStack

from Config import *

R4_IP = "192.168.0.224" #"192.168.142.37" #"192.168.38.37"#"172.20.10.13" #
R4_PORT = 2018
 
def getIP():
	
	# Non blocking socket
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.settimeout(0)

	try:
		# doesn't even have to be reachable
		s.connect(('10.254.254.254', 1))
		IP = s.getsockname()[0]
	except Exception:
		IP = '127.0.0.1'
	finally:
		s.close()

	return IP

thisIP = getIP()

#########
# Class #
#########
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('R4_Command')
        self.subscription = self.create_subscription(String, 'R4_Command', self.commandCB, 10)
        self.subscription  # prevent unused variable warning
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        self.sock.setblocking(False)
        # Bind the socket to the port
        self.sock.bind((thisIP, 4000))
        self.packetNum = 1
        self.stack = myStack()

    ############
    # listener #
    ############
    def commandCB(self, msg):
        self.packetNum += 1
        data = msg.data
        self.stack.add(data)
        data = self.addPNum(data)
        data = self.addTimeStamp(data)
        
        data = self.addCRC(data)

        self.stack.add(data)

        data = data.encode('utf-8')
        self.sock.sendto(data, (R4_IP, R4_PORT))
        self.get_logger().info('Sent to R4 %s:%s %s' % (R4_IP, R4_PORT, data))
 
    ################
    # addTimeStamp #
    ################
    def addTimeStamp(self, s):
        if s[-1] != ";":
            s = s + ";"
        obj = time.gmtime(0)
        curr_time = round(time.time()*1000)
        s = s + "T:" + str(curr_time) + ";"
        return s

    ###########
    # addPnum #
    ###########
    def addPNum(self, s):
        if s[-1] != ";":
            s = s + ";"
        s = s + "PNum:" + str(self.packetNum) + ";"
        return s

    ###########
    # makeCRC #
    ###########
    def makeCRC(self, s):
        s = s.encode('utf-8')
        crc = hex(zlib.crc32(s) & 0xffffffff)
        return crc
    
    ##########
    # addCRC #
    ##########
    def addCRC(self, s):
        if s[-1] != ";":
            s = s + ";"
        s = s + "^"
        crc = self.makeCRC(s)
        return s + crc + "^"

########
# Main #
########
def main(args=None):
    rclpy.init(args=args)
    R4_subscriber = MinimalSubscriber()
    rclpy.spin(R4_subscriber)
    R4_subscriber.destroy_node()
    rclpy.shutdown()

# Start properly
if __name__ == '__main__':
    main()
