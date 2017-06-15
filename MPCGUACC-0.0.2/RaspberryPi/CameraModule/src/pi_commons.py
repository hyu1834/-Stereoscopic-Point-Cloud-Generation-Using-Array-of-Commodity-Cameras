from enum import Enum

class Resolution(Enum):
	VGA = (640, 480)
	HD720 = (1280, 720)
	HD1080 = (1920, 1080)
	HD5MP = (2592, 1944)
	HD8MP = (3280, 2464)

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
