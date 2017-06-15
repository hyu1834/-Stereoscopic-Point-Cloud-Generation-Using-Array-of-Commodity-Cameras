#Standard Libraries
from picamera.array import PiRGBArray
from picamera import PiCamera
import io
import time

#Third Parties Libraries
from PIL import Image
import numpy as np

#Local Libraries
import opencv_utils



class Pi_Camera_Utils(object):
	def __init__(self, resolution, send_via_network = False, rotation = 180):
		self.resolution = resolution
		self.send_via_network = send_via_network
		# init camera instance
		self.pi_camera = PiCamera()
		# self.pi_camera = PiCamera(stereo_mode = 'side-by-side')

		# 
		# Camera Setting
		#
		# camera resolution
		self.pi_camera.resolution = resolution.value
		# raw image format
		self.pi_camera.raw_format = 'bgr'
		# rotate image by N degree
		self.pi_camera.rotation = rotation
		# frame rate
		# self.pi_camera.framerate = 30
		# shutter speed
		self.pi_camera.shutter_speed = self.pi_camera.exposure_speed
		self.pi_camera.exposure_mode = 'off'
		# This time out will make the magic happen, it prevent the image
		# from being all green, because of the white balancing 
		time.sleep(2)
		g = self.pi_camera.awb_gains
		self.pi_camera.awb_mode = 'off'
		self.pi_camera.awb_gains = g

		# stream for the image, so we dont need to write it tho I/O
		self.capture_stream = PiRGBArray(self.pi_camera, size = resolution.value)
		
		if self.send_via_network:
			pass

	def get_image_height(self, image):
		return image.shape[:2][1]

	def get_image_width(self, image):
		return image.shape[:2][0]

	def get_image_center(self, image):
		width, height = image.shape[:2]
		return (width / 2, height / 2)

	def capture_image(self, image_format = "bgr", fast_capture = True):
		#capture a single frame image
		#for faster capture speed enable video port
		self.pi_camera.capture(self.capture_stream, format = image_format, use_video_port = fast_capture)
		image = self.capture_stream.array
		return image, len(image)

	def capture_opencv_image(self, image_format = 'bgr', fast_capture = True):
		if not any(image_format == supported_format for supported_format in ['bgr']):
			return None

		self.capture_stream.truncate()
		self.pi_camera.capture(self.capture_stream, format = image_format, use_video_port = fast_capture)
		# convert to opencv image format
		image = opencv_utils.numpy_to_opencv(self.capture_stream.array)
		return image

	def capture_N_images(self, N, image_format = "bgr", fast_capture = True, send = False):
		if send:
			start = time.time()
			self.pi_camera.capture_sequence(self.stream_image(N), image_format, use_video_port = fast_capture)
			finished = (time.time() - start)
		else:
			self.capture_stream = [PiRGBArray(self.pi_camera, size = self.resolution.value) for i in range(N)]
			start = time.time()
			self.pi_camera.capture_sequence(self.bgr_stream, image_format, use_video_port = fast_capture)
			finished = (time.time() - start)
		print("capture time: %.2ffps at %.3f"%(N/finished, finished))
		
	def capture_save_image(self, filename, image_format = "jpeg", image_ext = 'jpeg', fast_capture = True):
		self.pi_camera.capture("%s.%s"%(filename, image_ext), image_format, use_video_port = fast_capture)

	def capture_show_image(self, image_format = "bgr", fast_capture = True):
		#show captured image using openCV
		image, image_len = self.capture_image(image_format, fast_capture)
		opencv_utils.show_image("image", image)
	
	def stream_image(self, N):
		for i in N:
			yield self.capture_stream
				
			#clear stream
			self.clear_stream()

	def start_preview_stream(self):
		self.pi_camera.start_preview()

	def stop_preview_stream(self):
		self.pi_camera.stop_preview()
		
	def clear_stream(self):
		self.capture_stream.seek(0)
		self.capture_stream.truncate(0)

	def close(self):
		self.pi_camera.close()
