#Standard Libraries
from picamera.array import PiRGBArray
from picamera import PiCamera
from enum import Enum
import io
import time

#Third Parties Libraries
from PIL import Image
import numpy as np

#Local Libraries
import io_utils
import opencv_utils

class PiCamera_Version(Enum):
	V1 = 1.3
	V2 = 2.1

class Camera_Sensor_Mode(Enum):
	V1_5MP = 2
	V2_8MP = 2

class Supported_Camera_Sensor_Mode(Enum):
	V1 = [Camera_Sensor_Mode.V1_5MP]
	V2 = [Camera_Sensor_Mode.V2_8MP]

class Image_Resolution(Enum):
	VGA = (640, 480)
	HD720 = (1280, 720)
	HD1080 = (1920, 1080)
	HD5MP = (2592, 1944)
	HD8MP = (3280, 2464)

class Supported_Image_Resolution(Enum):
	V1 = [Image_Resolution.VGA, Image_Resolution.HD720, Image_Resolution.HD1080, Image_Resolution.HD5MP]
	V2 = [Image_Resolution.VGA, Image_Resolution.HD720, Image_Resolution.HD1080, Image_Resolution.HD5MP, Image_Resolution.HD8MP]


class Image_Format(Enum):
	JPEG = 'jpeg'
	PNG = 'png'
	GIF = 'gif'
	BMP = 'bmp'
	YUV = 'yuv'
	RGB = 'rgb'
	RGBA = 'rgba'
	BGR = 'bgr'
	BGRA = 'bgra'
	RAW = 'raw'

class Supported_Image_Format(Enum):
	RAW = [Image_Format.RGB, Image_Format.RGBA, Image_Format.BGR, Image_Format.BGRA, Image_Format.RAW]
	OTHERS = [Image_Format.JPEG, Image_Format.BMP]

class Supported_Image_Extension(Enum):
	RAW = [Image_Format.JPEG, Image_Format.BMP]
	OTHERS = [Image_Format.JPEG, Image_Format.BMP]

class Capture_Mode(Enum):
	VIDEO = 'video'
	IMAGE = 'image'



class PiCamera_Utils(object):
	def __init__(self, resolution, picamera_version = PiCamera_Version.V1,
				 camera_sensor_mode = Camera_Sensor_Mode.V1_5MP, capture_mode = Capture_Mode.IMAGE,
				 rotation = 180, use_video_port = False):

		# Check if PiCamera Version matches support mode
		if(not camera_sensor_mode in Supported_Camera_Sensor_Mode[picamera_version.name].value):
			io_utils.stderr("PiCamera Version: %s does not support Sensor Mode: %s"%(picamera_version, camera_sensor_mode), terminate = True)

		if(not resolution in Supported_Image_Resolution[picamera_version.name].value):
			io_utils.stderr("PiCamera Sensor Mode: %s does not support Resolution: %s"%(camera_sensor_mode, resolution), terminate = True)

		self.capture_mode = capture_mode

		if capture_mode == Capture_Mode.IMAGE:
			# image/video resolution
			self.resolution = resolution
			# image rotation
			self.rotation = rotation
			#use video port for rapid capture
			self.use_video_port = use_video_port

			# Init Picamera Instance
			self.picamera = PiCamera(sensor_mode = camera_sensor_mode.value)
			# Disable camera LED
			self.picamera.led = False
			# Camera Setting
			self.picamera.resolution = resolution.value
			self.picamera.rotation = rotation
			self.picamera.raw_format = 'bgr'
			
			if use_video_port:
				# shutter speed
				self.picamera.shutter_speed = self.picamera.exposure_speed
				self.picamera.exposure_mode = 'off'
				time.sleep(1)
				g = self.picamera.awb_gains
				self.picamera.awb_mode = 'off'
				self.picamera.awb_gains = g
		else:
			# currently not support video mode
			pass
			
	def get_image_height(self, image):
		return image.shape[:2][1]

	def get_image_width(self, image):
		return image.shape[:2][0]

	def get_image_center(self, image):
		width, height = image.shape[:2]
		return (width / 2, height / 2)
	
	def capture_save_raw_image(self, filepath, image_format, image_extension):
		# Must be using Image Capturing mode
		if self.capture_mode != Capture_Mode.IMAGE:
			return
		#Must be in supported image format and extension
		if not image_format in Supported_Image_Format.RAW.value or not image_extension in Supported_Image_Extension.RAW.value:
			return

		stream = PiRGBArray(self.picamera, size = self.resolution.value)
		self.picamera.capture(stream, image_format.value, use_video_port = self.use_video_port)
		opencv_utils.save_image(stream.array, "%s.%s"%(filepath, image_extension.value))

	def capture_save_image(self, filepath, image_format, image_extension):
		# Must be using Image Capturing mode
		if self.capture_mode != Capture_Mode.IMAGE:
			return
		#Must be in supported image format and extension
		if not image_format in Supported_Image_Format.OTHERS.value or not image_extension in Supported_Image_Extension.OTHERS.value:
			return

		self.picamera.capture("%s.%s"%(filepath, image_extension.value), image_format.value, use_video_port = self.use_video_port)

	def start_preview_stream(self):
		self.picamera.start_preview()

	def stop_preview_stream(self):
		self.picamera.stop_preview()

	def close(self):
		self.picamera.close()
