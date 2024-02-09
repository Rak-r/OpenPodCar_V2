#!/usr/bin/env python3

#################################################################
# Author        : Andy Perrett                                  #
# Email         : 18684092@students.lincoln.ac.uk               #
# Date          : 18th August 2023                              #
# Description   : Connect to R4 via websockets                  #
#                 Send some UDP data to trigger R4 into action  #
#                 R4 sends UDP data to this host IP             #
################################################################# 
import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import String, Bool
import socket
import time
import zlib

#from Config import 
# *
R4ip = "192.168.0.224" #"192.168.38.37"# #
##########
# Config #
##########
hostPort = 2390
R4port = 2018
HEART_S = "H:ROS2-R4"
HEART_R = "H:R4-ROS2"
NUM_MISSING_HB = 10
RECV_BUFFER = 200

def isHeartBeat(data):
    if data == None: return False
    parts = data.strip().split(";")
    if  HEART_R in parts:
        return True
    return False

def getPacketNumber(data):
    if data == None: return False
    parts = data.strip().split(";")
    for i, data in enumerate(parts):
        p = data.split(":")
        if p[0] == "PNum":
            return int(p[1])
    return 0

def getTimeStamp(data):
    if data == None: return False
    parts = data.strip().split(";")
    for i, data in enumerate(parts):
        p = data.split(":")
        if p[0] == "T":
            return int(p[1])
    return 0

######################
# R4 Websockets Node #
######################
class R4Websockets(Node):

    
    def __init__(self, hostPort):
        super().__init__('R4_Connection')
        self.publisher_R4 = self.create_publisher(String, 'R4', 1)
        self.publisher_HB = self.create_publisher(Bool, 'R4_Heart_Beat', 1)
        self.publisher_RUN = self.create_publisher(Bool, 'R4_Active', 1)
        
        self.droppedPackets ={}
        
        self.running = False
        self.firstTimeout = 1
        self.timeout = 10
        self.heartBeat = 0.7
        self.gotHeartBeat = False
        self.crcErrorsData = 0
        self.crcErrorsHB = 0
        self.numPackets = 0
        self.found = False
        self.R4Address = None

        # Auto config host IP
        self.myIP = self.getIP()
        self.hostAddress = (self.myIP, hostPort)
        
        # Run until ctrl-c
        while True:
            try:
                self.setupSocket()
                self.waitForConnection()
                if self.found: self.receive()

            # If cntrl-c pressed        
            except KeyboardInterrupt:
                self.tidy()

            # Catch-all errors
            except Exception as e:
                print(u"---- \u001b[31mEXITED: UNKNOWN REASON\u001b[0m ----\u001b[?25h")
                print(e)
                self.tidy(True)

    #######################################
    # wait for a FIRST connection from R4 #
    #######################################
    def waitForConnection(self):
        print("This PC: " + str(self.myIP))
        print("Waiting for connection from " + str(self.R4Address) + "...")
        print()
        print(u"\u001bM\u001b[?25l"+HEART_S)

        # repeatly send a connection request until we hear back from R4
        hbt = time.time() * 1000
        while not self.running:
            heart = time.time()
            hbt = heart
            data = None

            # Check UDP buffer
            dataArray, crcArray, address = self.getUDPDataMulti()

            # Only looking for 1st heart beat
            if len(dataArray)> 0:
                data = dataArray[0]
                crc = crcArray[0]
            else:
                data = ''
                crc = ''

            if len(dataArray) > 0:
                self.R4Address = address
                self.pnum = getPacketNumber(data)
                self.numPackets += len(dataArray)
                hbt = time.time() * 1000

                # Check and deal with crc error
                if crc != self.makeCRC(data+";^"):
                    self.crcERROR(data)

                # Not interested in other data
                if not isHeartBeat(data) and data != '':
                    print("Discarded",dataArray)
                    #exit()

                # Reply to heart beat
                if isHeartBeat(data):
                    # Send to websockets
                    self.sendHB(hbt, data)
                    self.running = True
                    return
                
                if time.time() > heart + self.firstTimeout and not self.found:
                    #self.s.close()
                    print(u"---- \u001b[31mRESET\u001b[0m ----\u001b[?25h")
                    return

    
    ##################################
    # Time the incoming data packets #
    ##################################
    def receive(self):
            print("Normal receiving mode...")
            hbt = time.time() * 1000
            self.numPackets = self.pnum
            numP = 0
            lastErrorCount = 0
            heart = time.time()
            while True:
                crc, data, raw = None, "", ""
                data = None
                dataArray, crcArray, address = self.getUDPDataMulti()
                if len(dataArray) > 1:
                    print(dataArray)
                    print("2 PACKETS or more!!!!")
                    exit()
                
                # Only 1st heart beat
                if len(dataArray) > 0:
                    data = dataArray[0]
                    crc = crcArray[0]
                else:
                    data = None
                    crc = ''
                
                found = False

                if data != None:

                    if  getPacketNumber(data) == 1:
                        self.numPackets = 0

                    self.numPackets += 1
                    
                    numP = abs(self.numPackets-getPacketNumber(data))
                    
                    if lastErrorCount != numP:
                        print(dataArray)
                        print("--- ERROR --- ROS2:",self.numPackets,"R4:",getPacketNumber(data), "Error count:", numP)
                        lastErrorCount = numP
                        #exit()

                    # Check and deal with crc error
                    if crc != self.makeCRC(data+";^"):
                        self.crcERROR(data)

                    # Count non-heart beat good packets
                    if isHeartBeat(data) == False:
                        # Publish R4 data packet on topic
                        self.pubR4(data)
                        print(self.numPackets,data)
        
                    # Send heart beat
                    if isHeartBeat(data): 

                        #if self.gotHeartBeat:
                        self.sendHB(hbt, data)

                        self.gotHeartBeat = False if self.gotHeartBeat else True
                        found = True
                        self.pubHeartBeat()
                        self.pubActive()

                        if getPacketNumber(data) != self.numPackets:
                            print("--- ERROR --- ROS2:",self.numPackets,"R4:",getPacketNumber(data), "Error count:", abs(self.numPackets-getPacketNumber(data)))
                            #exit()
                    
                    # Reset timer
                    if found:
                        heart = time.time()

                # If we miss heart beats for x amount of time reset
                if time.time() > heart + self.heartBeat and not found:
                    self.s.close()
                    self.running = False
                    self.pubActive()
                    self.flushBuffer()
                    print("RESET")
                    return
    
    ##########
    # sendHB #
    ##########
    def sendHB(self, hbt, data):
        hbt2 = (time.time() *1000) - hbt
        hbWithTS =  HEART_S +  ";T:" + str(getTimeStamp(data))#  round(hbt2,4))
        msg = self.addCRC(hbWithTS)
        self.s.sendto(msg.encode('utf-8'), self.R4Address)
        self.pubActive()
        self.found = True
        print(data,"--->", msg)


    ##########
    # addCRC #
    ##########
    def addCRC(self, s):
        s = s + ";^"
        crc = self.makeCRC(s)
        return s + crc + "^"

    ##########
    # getCRC #
    ##########
    def getCRC(self, s):
        chunks = s.split(';')
        crc = chunks[-1]
        l = len(s) - len(crc) -1
        return s[:l], crc[1:-1]

    ###########
    # makeCRC #
    ###########
    def makeCRC(self, s):
        s = s.encode('utf-8')
        crc = hex(zlib.crc32(s) & 0xffffffff)
        return crc       

    #############
    # pubActive #
    #############
    def pubActive(self):
        msg = Bool()
        msg.data = self.running
        self.publisher_RUN.publish(msg)

    #########
    # pubR4 #
    #########
    def pubR4(self, data):
        msg = String()
        msg.data = str(data)
        self.publisher_R4.publish(msg)

    ################
    # pubHeartBeat #
    ################
    def pubHeartBeat(self):
        msg = Bool()
        msg.data = self.gotHeartBeat
        self.publisher_HB.publish(msg)

    ##############
    # getUDPData #
    ##############
    def getUDPDataMulti(self):
        isThereData = True
        packets, crcs = [], []
        address = None
        while isThereData:
            data = None
            # Non blocking
            try:
                data, address = self.s.recvfrom(RECV_BUFFER)
            except:
                pass
            # We got data
            if data != None:
                d, c = self.getCRC(data.decode('utf-8'))
                packets.append(d)
                crcs.append(c)
                isThereData = False
            # Buffer is empty
            else:
                break
        # Return array, either populated or empty
        return packets, crcs, address
    
    ###############
    # flushBuffer #
    ###############
    def flushBuffer(self):
        while True:
            data = None
            # Non blocking
            try:
                data, address = self.s.recvfrom(RECV_BUFFER)
            except:
                pass
            if data == None:
                return

    ############
    # crcERROR #
    ############
    def crcERROR(self, data):
        if isHeartBeat(data):
            self.crcERRORhb()
        else:
            self.crcERRORdata()

    ################
    # crcERRORdata #
    ################
    def crcERRORdata(self):
        print("data crc error:", self.crcErrorsData)
        self.tidy(True)
        
    ##############
    # crcERRORhb #
    ##############
    def crcERRORhb(self):
        print("heart beat crc error:", self.crcErrorsHB)
        self.tidy(True)

    ###############
    # setupSocket #
    ###############
    def setupSocket(self):
        # Create a UDP socket
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setblocking(False)
        self.s.bind(self.hostAddress)

    #####################
    # Get IP of this PC #
    #####################
    # Taken from https://stackoverflow.com/questions/166506/finding-local-ip-addresses-using-pythons-stdlib
    def getIP(self):
        
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
    
    ######################
    # Tidy Up connection #
    ######################
    def tidy(self, error=False):
        # Close the socket and restore cursor, then quit
        try:
            self.s.close()
        except:
            pass
        if not error:
            print(u"---- \u001b[32mEXITED NORMALLY\u001b[0m ----\u001b[?25h")
        else:
            print(u"---- \u001b[31mERROR\u001b[0m ----\u001b[?25h")
        print("Dropped packets",self.droppedPackets)
        exit()

########
# main #
########
def main(args=None):
    rclpy.init(args=args)
    R4_Connection = R4Websockets(hostPort)

    # May not get here
    R4_Connection.tidy()
    R4_Connection.destroy_node()
    rclpy.shutdown()

# Start properly
if __name__ == '__main__':
    main()
