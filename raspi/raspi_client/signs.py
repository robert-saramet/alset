import cv2

width_at_50 = 200  # magic number for number of pixels for 0.5 distance
def FindDistanceToObject(object):
	(x, y, w, h) = object
	# show rect around object
	#cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
	dist = width_at_50 / w / 2
	return dist


def DetectCascade(image, cascade):
	return cascade.detectMultiScale(image, scaleFactor=1.1, minNeighbors=5, minSize=(24, 24), flags = cv2.CASCADE_SCALE_IMAGE)


class StopSignHandler:
	distance = 0
	in_range = False  # true if the sign is within 1 meter
	in_range_time = 0.0
	ready = True
	halt = False
	forward = False
	cascade = cv2.CascadeClassifier("stop_cascade.xml")

	def Update(self, frame, delta_time):
		self.halt = False
		self.forward = False
		stop_signs = DetectCascade(frame, self.cascade)
		if len(stop_signs):
			if self.in_range_time > 3 and self.ready:  # delay
				self.in_range_time = 0  # instead of in_range_time = 0, this gives room for error
				if self.in_range_time < 0:
					self.in_range_time = 0
				self.forward = True
				print("Waited 3 seconds")  # for debug purposes
				self.ready = False
			if len(stop_signs):
				self.distance = FindDistanceToObject(stop_signs[0])
				if self.distance < 1 and self.ready:
					self.in_range_time += delta_time
					if self.in_range_time < 3:
						self.halt = True
		else:
			self.ready = True



#  Not used at the moment, can recognize speed limits
#  Currently using gps.py instead
class SpeedLimitHandler():
	cascade = cv2.CascadeClassifier("speed_limits_cascade.xml")
	max_speed = 20
	time = 0
	def Update(self, frame, delta_time):
		# TODO: refactor into lambdas
		def filter_digit_2(char):
			return char == '2'
		def filter_digit_4(char):
			return char == '4'
		def filter_digit_8(char):
			return char == '8'	
		speed_limit_signs = DetectCascade(frame, self.cascade)
		rgb_frame = frame
		if len(speed_limit_signs):
			(x, y, w, h) = speed_limit_signs[0]
			rgb_frame = speed_limit_signs[y:y+h, x:x+w]
			rgb_frame = cv2.blur(frame, (20, 20))
			rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			rgb_frame = cv2.adaptiveThreshold(rgb_frame, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 5, 2)
			string = tess.image_to_string(rgb_frame, config="--psm 9 -c tessedit_char_whitelist=0248")
			string = string[0:-(len(string) - 2)]
			if len(list(filter(filter_digit_2, string))):
				self.max_speed = 20
			elif len(list(filter(filter_digit_4, string))):
				self.max_speed = 40
			elif len(list(filter(filter_digit_8, string))):
				self.max_speed = 80
		print("Speed limit update")

class CrosswalkHandler:
	should_slow_down = 20
	cascade = cv2.CascadeClassifier("crosswalk_cascade.xml")
	def Update(self, frame):
		pass

