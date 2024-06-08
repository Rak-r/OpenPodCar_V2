#!/usr/bin/env python3

import os

# Set remote IP
if os.uname()[1] == "dandy-G5-MD":
	#REMOTE_IP = "192.168.1.98"
	REMOTE_IP = "10.5.36.28"
	
else:
	REMOTE_IP = "192.168.142.37" #"172.20.10.13"

R4_PORT = 2018
REMOTE_PORT = 2390
LOCAL_PORT= 2018
R4ip = REMOTE_IP
R4_IP = REMOTE_IP