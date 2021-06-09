import os
import time

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
	return command_queue
def Start(pos, dest):
	global destination, position
	destination = dest
	position = pos
	GenerateCommandQueue(pos, dest)
	pass
