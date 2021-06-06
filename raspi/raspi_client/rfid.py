import os
import time
#from mpu6050py import MPU6050
# a point will have an ID, a position (x, y) and a speed limit
# e.g. (0, 2, 2, 40)
points = [
	(0, 0, 80), (1, 0, 20), (2, 0, 20), (3, 0, 20), (4, 0, 20),
	(0, 1, 80), (1, 1, 20), (2, 1, 20), (3, 1, 20), (4, 1, 80),
	(0, 2, 80), (1, 2, 40), (2, 2, 40), (3, 2, 40), (4, 2, 40),
	(0, 3, 80), (1, 3, 40), (2, 3, 40), (3, 3, 40), (4, 3, 40),
	(0, 4, 80), (1, 4, 80), (2, 4, 80), (3, 4, 80), (4, 4, 80)
]
command_queue = []

def DEGREES(x):
	return x * 180 / 3.1415

"""
rotation_x = 0
rotation_y = 0
rotation_z = 0
sensor_addr = 0x68
sensor = MPU6050()
sensor.set_x_gyro_offset(92)
sensor.set_y_gyro_offset(-427)
sensor.set_z_gyro_offset(130)
delta_time_start = 0
delta_time_end = 0
while True:
	delta_time = delta_time_end - delta_time_start
	delta_time_start = time.perf_counter() # time in seconds

	delta_x = sensor.get_rotation()[0]
	delta_y = sensor.get_rotation()[1]
	delta_z = sensor.get_rotation()[2]
	if delta_time > 0:
		if 0 < delta_x < 5 or 0 > delta_z > -5:
			delta_x = 0
		if 0 < delta_y < 14 or 0 > delta_z > -5:
			delta_y = 0
		if 0 < delta_z < 5 or 0 > delta_z > -5:
			delta_z = 0
		rotation_x += delta_x / delta_time / 1000
		rotation_y += delta_y / delta_time / 1000
		rotation_z += delta_z / delta_time / 1000

	os.system('clear')
	print (rotation_x)
	print (rotation_y)
	print (rotation_z)

	delta_time_end = time.perf_counter()

"""

"""
rfid_port = ''
rfid = RFID(port=rfid_port, baud_rate=9600)
def RFIDGetID():
	return rfid.get_id()

sensor_addr = ''
sensor = mpu6050(sensor_addr)

# using RFID and the IDs of the points
def GetLocation():
	# move forward to find the RFID sticker (?)
	# id = RFIDGetID()
	# return points[id][0], points[id][1]
	return (1, 1)
"""

def GetLocation():
	return (1, 1)

# returns "up", "down", "right" or "left", based on the angle returned by the gyro
def GetDirection():
	
	return "right"

# using RFID and the IDs of the points
def GetSpeedLimit():
	id = RFIDGetID()
	return points[id][2]

def GenerateCommandQueue(position, destination):
	# initial data
	x_offset = destination[0] - position[0]
	y_offset = destination[1] - position[1]
	dir = GetDirection()

	# make the car face the corect way
	if y_offset < 0:
		# the desired direction is up
		if dir == "down":
			command_queue.append(["right", "right"])
		elif dir == "right":
			command_queue.append("left")
		elif dir == "left":
			command_queue.append("right")
	else:
		# the desired direction is down
		if dir == "up":
			command_queue.append(["right", "right"])
		elif dir == "right":
			command_queue.append("right")
		elif dir == "left":
			command_queue.append("left")

	# add the remaining commands to reach the destination
	for i in range(abs(y_offset)):
		command_queue.append("forward")
	if y_offset < 0:
		if x_offset < 0:
			command_queue.append("left")
		else:
			command_queue.append("right")
	else:
		if x_offset > 0:
			command_queue.append("left")
		else:
			command_queue.append("right")		
	for i in range(abs(x_offset)):
		command_queue.append("forward")
def Start(pos, dest):
	global destination, position
	destination = dest
	position = pos
	GenerateCommandQueue(pos, dest)
	pass
