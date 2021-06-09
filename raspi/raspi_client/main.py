# if a stop sign is detected and in range, wait for it to be in sight
# for 3 seconds, then continue moving until it isn't detected anymore

import signs
from signs import cv2
import gps
import numpy as np
import serial
import time
import _thread as thread
import networking as net
from gpiozero import LED
stop_handler = signs.StopSignHandler()
last_tick_halt = 0

# 26 = comms pin
# on() -> sends 1
# off() -> sends 0
comms_pin = LED(26)
comms_pin.on()

def SendMsg_Stop():
	print("STOP RIGHT THERE U MF")
	comms_pin.off()
def SendMsg_Forward():
	print("MOVE UR FCKING ASS U PIECE OF SHIT")
	comms_pin.on()
	

cap = cv2.VideoCapture(0)
delta_time_start = 0.0
delta_time_end = 0.0
while True:
	delta_time = delta_time_end - delta_time_start
	delta_time_start = time.perf_counter() # Time in seconds

	# Camera input
	frame = cap.read()[1]

	(x, y, w, h) = stop_handler.Update(frame, delta_time, SendMsg_Stop, SendMsg_Forward)

	cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		comms_pin.off()
		break

	cv2.imshow('frame', frame)

	delta_time_end = time.perf_counter()
