# if a stop sign is detected, wait for it to be in sight for 3 seconds,
# then continue moving until it isn't detected anymore

import signs
import cv2
import numpy as np
import sys
import time
from gpiozero import LED

# not used for beta version
import rfid
import networking as net # server comms

move = LED(26) # gpio comms
stop_handler = signs.StopSignHandler()
last_tick_halt = 0

cap = cv2.VideoCapture(0)
delta_time_start = 0.0
delta_time_end = 0.0

move.on()

while True:
	try:
		delta_time = delta_time_end - delta_time_start
		delta_time_start = time.perf_counter() # delta time in seconds

		# Camera input
		frame = cap.read()[1]

		# if display connected show opencv feed
		'''
		cv2.imshow('frame', frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		'''

		stop_handler.Update(frame, delta_time)

		if stop_handler.halt:
			move.off()
			print('HALT')
		else:
			print('FORWARD')
			move.on()

		delta_time_end = time.perf_counter()
	except KeyboardInterrupt:
		print("Closing now")
		move.off()
		sys.exit()
