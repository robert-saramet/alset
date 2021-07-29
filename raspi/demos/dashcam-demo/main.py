"""
Docc:
- the json parser
- the way the image is processed for lane detection
- the sign recog system
"""

# To enable sign detection, use "-s"

# For a more "real-life" implementation, a class
# with the basic data can be created and used in
# a "one Update() function updates all variables"
# manner

import matplotlib.pylab as plt
import cv2
import numpy as np
import sys
import signs
import math

cap = cv2.VideoCapture('test.mp4')

stop_sign_handler = signs.StopSignHandler("stop_cascade.xml")
speed_limit_handler = signs.SpeedLimitHandler("speed_limit_cascade.xml")

# used for indexing the current sign to check for in the next frame
tick_index = 0

# also shows frame but whatever
def process_frame(frame):
	global tick_index
	tick_index += 1

	image = frame
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	height = image.shape[0]
	width = image.shape[1]
	
	region_of_interest_vertices = [
		(width / 3 * 1.25, height / 2.5),
		(width / 3 * 1.5, height / 2.5),
		(width, height / 1.5),
		(0, height / 1.5)
	]

	# used to crop the detection area
	def region_of_interest(img, vertices):
		mask = np.zeros_like(img)
		match_mask_color = 255
		cv2.fillPoly(mask, vertices, match_mask_color)
		masked_image = cv2.bitwise_and(img, mask)
		return masked_image

	def draw_lines(img, lines):
		img = np.copy(img)
		blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

		for line in lines:
			for x1, y1, x2, y2 in line:
				cv2.line(blank_image, (x1,y1), (x2,y2), (0, 255, 0), thickness=10)

		img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
		return img

	gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	blur_image = cv2.blur(gray_image, (5, 5))
	canny_image = cv2.Canny(blur_image, 50, 150)
	cropped_image = region_of_interest(canny_image,
					np.array([region_of_interest_vertices], np.int32),)
	lines = cv2.HoughLinesP(cropped_image,
							rho=6,
							theta=np.pi/180,
							threshold=160,
							lines=np.array([]),
							minLineLength=40,
							maxLineGap=25)
	#highlighted_image = draw_lines(image, lines)
	highlighted_image = image
	if lines.any():
		init_pos = [lines[0][0][0], lines[0][0][1]]
		merged_vec = [lines[0][0][0], lines[0][0][1]]
		for l in lines[0]:
			dx = l[2] - l[0]
			dy = l[3] - l[1]
			merged_vec[0] += dx
			merged_vec[1] += dy
		
		highlighted_image = draw_lines(image, [[(merged_vec[0], merged_vec[1], init_pos[0], init_pos[1])]])

	# Sign recognition
	if len(sys.argv) > 1 and sys.argv[1] == '-s':
		if tick_index % 2 == 0:
			stop_sign_handler.Update(image, 0)
			if len(stop_sign_handler.stop_signs):
				for (x, y, w, h) in stop_sign_handler.stop_signs:
					cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

		elif tick_index % 2 == 1:
			speed_limit_handler.Update(image, 0)
			if len(speed_limit_handler.speed_limit_signs):
				for (x, y, w, h) in speed_limit_handler.speed_limit_signs:
					cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

	cv2.imshow("demo", highlighted_image)
	return (merged_vec[0] - init_pos[0], merged_vec[1] - init_pos[1])

default_angle = 29
while cap.isOpened():
	ret, frame = cap.read()
	line_vec = process_frame(frame)
	angle = math.degrees(math.atan2(line_vec[1], line_vec[0]))
	print(angle + default_angle)

	if cv2.waitKey(25) & 0xFF == ord('q'):
		break

