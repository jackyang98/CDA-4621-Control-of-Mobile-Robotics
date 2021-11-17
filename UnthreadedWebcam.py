import cv2 as cv

class UnthreadedWebcam:
	def __init__(self, src=0, name="ThreadedWebcam"):
		self.stream = cv.VideoCapture(src)
		self.stream.set(cv.CAP_PROP_FRAME_WIDTH, 640)
		self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
		#self.stream.set(cv.CAP_PROP_BUFFERSIZE, 1)
		
		if not self.stream.isOpened():
			print("Failed to open camera!")
			exit()
		
	def start(self):
		pass
		
	def read(self):
		(self.grabbed, self.frame) = self.stream.read()
		return self.frame
	
	def stop(self):
		pass