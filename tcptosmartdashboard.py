#!/usr/bin/env python
import time
import socket
from networktables import NetworkTables

TCP_IP = '192.168.1.100'
#TCP_IP = '172.22.11.1'
#TCP_IP = '192.168.1.100'
TCP_PORT = 5809
BUFFER_SIZE = 100

networkTablesServer = '127.0.0.1'
NetworkTables.initialize(server=networkTablesServer)
visionTable = NetworkTables.getTable('SmartDashboard')

visionTable.putNumber('tcpstufffffff', 7)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


connected = False

while not connected:
	try:
		s.connect((TCP_IP, TCP_PORT))
		connected = True
	except Exception as e:
		pass
		
i = 0
while True:
	data =  s.recv(1024)
	dataDecode = data.decode("utf-8")
	splitData = dataDecode.split(" ")
	if splitData[0] == "1":
		visionTable.putBoolean("hasTarget", True)
		visionTable.putNumber("timeDelta", float(splitData[1]))
		visionTable.putNumber("angle", float(splitData[2]))
		visionTable.putNumber("distance", float(splitData[3]))
	else:
		visionTable.putBoolean("hasTarget", False)
	print(splitData)

s.close()

visionTable.putNumber('fffffffffff', 8)

time.sleep(.5)
