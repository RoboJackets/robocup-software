import struct
from LogFrame_pb2 import *

class RobocupLog:
	def __init__(self, filename):
		self.f = file(filename)

	def readFrame(self):
		data = self.f.read(4)
		if len(data) == 0:
			return None
		num_bytes = struct.unpack('<I', data)[0]
		data = self.f.read(num_bytes)
		frame = LogFrame()
		frame.ParseFromString(data)
		return frame

