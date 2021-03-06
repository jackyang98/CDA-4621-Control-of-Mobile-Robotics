from threading import Thread
import cv2 as cv

class ThreadedWebcam:
	def __init__(self, src=0, name="ThreadedWebcam"):
		self.stream = cv.VideoCapture(src)
		self.stream.set(cv.CAP_PROP_FRAME_WIDTH, 640)
		self.stream.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
		#self.stream.set(cv.CAP_PROP_BUFFERSIZE, 1)
		
		if not self.stream.isOpened():
			print("Failed to open camera!")
			exit()
			
		(self.grabbed, self.frame) = self.stream.read()
		
		self.name = name
		
		self.stopped = False
		
	def start(self):
		t = Thread(target=self._update, name=self.name, args=())
		t.daemon = True
		t.start()
		return self
		
	def _update(self):
		while True:
			if self.stopped:
				return
				
			(self.grabbed, self.frame) = self.stream.read()
	
	def read(self):
		return self.frame
	
	def stop(self):
		self.stopped = True