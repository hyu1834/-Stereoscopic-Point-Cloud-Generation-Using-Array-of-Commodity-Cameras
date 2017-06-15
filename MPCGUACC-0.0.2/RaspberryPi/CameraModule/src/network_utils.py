#Standard Import
import socket
import struct
import time

#3rd Parties Import

#Local Import


class Network_Utils(object):
	def __init__(self, host, port, server = False):
		#class variables
		self.host = host
		self.port = port
		
		#create socket object
		try:
			self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		except socket.error, msg:
			print("Unable to create socket: %s"%str(msg))
			
		
		if server:
			self.socket.bind((host, port))
			self.socket.listen(0)
			# accept connection and file like object out of connection
			self.connection = server_socket.accept()[0].makefile('rb')
		else:
			self.socket.connect((host, port))
			# file like object out of connection
			self.connection = self.socket.makefile('wb')

	def send_data(self, data):
		pass

	def send_image(self, data, len):
		# first send the length of the image
		self.connection.write(struct.pack('<L', len))
		self.connection.flush()

		# send the actual image
		self.connection.write(data)

	def receive_image(self):
		# read the len of image
		image_len = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
		return self.connection.read(image_len)


	def close_connection(self):
		# notify the other side closing connection
		self.connection.write(struct.pack('<L', 0))
		self.connection.close()

	def close_socket(self):
		self.socket.close()

	def close_all(self):
		self.connection.write(struct.pack('<L', 0))
		self.connection.close()
		self.socket.close()
