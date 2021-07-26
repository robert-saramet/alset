import cv2

# magic number for number of pixels for 0.5 distance: width_at_50
# only works with our stop sign at the moment, a new
# magic number has to be set to work with street signs
# TODO: use height, so that rotation will not be a problem
width_at_50 = 200
def FindDistanceToObject(object):
	(x, y, w, h) = object
	d = width_at_50 / w / 2
	return d

def DetectCascade(image, cascade):
	return cascade.detectMultiScale(image, scaleFactor=1.1, minNeighbors=5, minSize=(24, 24), flags = cv2.CASCADE_SCALE_IMAGE)

class StopSignHandler:
	distance = 0
	in_range = False  # true if the sign is within 1 meter
	in_range_time = 0.0
	ready = True
	halt = False
	forward = False
	stop_signs = []
	def __init__(self, cascade_filename):
		self.cascade = cv2.CascadeClassifier(cascade_filename)
	def Update(self, frame, delta_time):
		self.halt = False
		self.forward = False
		self.stop_signs = DetectCascade(frame, self.cascade)
		if len(self.stop_signs):
			if self.in_range_time > 3 and self.ready:  # delay
				self.in_range_time = 0  # instead of in_range_time = 0, this gives room for error
				if self.in_range_time < 0:
					self.in_range_time = 0
				self.forward = True
				self.ready = False
			if len(self.stop_signs):
				self.distance = FindDistanceToObject(self.stop_signs[0])
				if self.distance < 1 and self.ready:
					self.in_range_time += delta_time
					if self.in_range_time < 3:
						self.halt = True
		else:
			self.ready = True


# Not used at the moment, can recognize speed limits
# uses tesseract, quite heavy on the CPU :\\
# import tesseract as tess is required (plus setting the path)
class SpeedLimitHandler():
	def __init__(self, cascade_filename):
		self.cascade = cv2.CascadeClassifier(cascade_filename)
		self.speed_limit_signs = []
	def Update(self, frame, delta_time):
		self.speed_limit_signs = DetectCascade(frame, self.cascade)

class CrosswalkHandler:
	cascade = cv2.CascadeClassifier("crosswalk_cascade.xml")
	# returns whether the car should start slowing down
	def Update(self, frame):
		pass
