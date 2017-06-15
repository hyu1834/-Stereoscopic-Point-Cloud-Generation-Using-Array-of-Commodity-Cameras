#Standard Import
import re
import os
import sys
import time

#3rd Parties Import
import RPi.GPIO as GPIO

#Local Import
import picamera_utils
import opencv_utils
# import network_utils

terminate_pin = 20
capture_pin = 21


def getrevision():
	revision = "unknown"
	with open('/proc/cmdline', 'r') as f:
		line = f.readline()
		m = re.search('bcm2708.boardrev=(0x[0123456789abcdef]*) ', line)
		revision = m.group(1)
	return revision

def print_usage():
	print("Usage: python main.py [options value]")
	print("Options:")
	print("\t\t-r, --resolution:")
	print("\t\t\tVGA/480 - Image with VGA/480p")
	print("\t\t\tHD720/720 - Image with HD720/720p")
	print("\t\t\tHD1080/1080 - Image with HD1080/1080p")
	print("\t\t\t5MP - Image with full 5MP")
	print("\t\t\t8MP - Image with full 8MP (Only available on Raspberry Pi camera V2.3")
	print("\t\t-id, --camera_id:")
	print("\t\t\tCamera ID")
	print("\t\t-o, --output_directory:")
	print("\t\t\tImage output directory")
	exit(0)


def prase_arguement(args):
	resolution = picamera_utils.Image_Resolution.HD720
	camera_id = "1"
	output_directory = "./"

	index = 0
	arg_len = len(args)
	# prase arguements
	while(index < arg_len):
		if "-h" == args[index] or "--help" == args[index]:
			print_usage()
		elif "-r" == args[index] or "--resolution" == args[index]:
			index += 1
			if(index < arg_len):
				if args[index] == "VGA" or args[index] == "480":
					resolution = picamera_utils.Image_Resolution.VGA
				elif args[index] == "HD720" or args[index] == "720":
					resolution = picamera_utils.Image_Resolution.HD720
				elif args[index] == "HD1080" or args[index] == "1080":
					resolution = picamera_utils.Image_Resolution.HD1080
				elif args[index] == "5MP":
					resolution = picamera_utils.Image_Resolution.HD5MP
				elif args[index] == "8MP":
					resolution = picamera_utils.Image_Resolution.HD8MP
				else:
					print("Resolution: %s not supported"%args[index])
					exit(0)
			else:
				print("Da")
		elif "-id" == args[index] or "--camera_id" == args[index]:
			index += 1
			if(index < arg_len):
				camera_id = args[index]
		elif "-o" == args[index] or "--output_directory" == args[index]:
			index += 1
			if(index < arg_len):
				output_directory = args[index]
		else:
			print("Unsupported option: %s"%args[index])
			exit(0)

		index += 1

	return resolution, camera_id, output_directory



def main():
	preview_stream = True
	image_format = picamera_utils.Image_Format.BGR
	image_extension = picamera_utils.Image_Format.BMP

	# prase arguements
	resolution, camera_id, output_directory = prase_arguement(sys.argv[1:])

	#GPIO pin setup
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(terminate_pin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
	GPIO.setup(capture_pin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

	# Picamera Instance
	picamera = picamera_utils.PiCamera_Utils(resolution, 
											 picamera_version = picamera_utils.PiCamera_Version.V1,
											 camera_sensor_mode = picamera_utils.Camera_Sensor_Mode.V1_5MP,
											 capture_mode = picamera_utils.Capture_Mode.IMAGE,
											 rotation = 180, use_video_port = False)

	# preview stream for easier capture
	if preview_stream:
		picamera.start_preview_stream()

	count = 1
	print("Done Init, waiting for input")
	try:
		while(1):
			if GPIO.input(capture_pin):
				print("Capturing")
				picamera.capture_save_raw_image(os.path.join(output_directory, "%s_%s"%(camera_id, count)),
												image_format = image_format,
												image_extension = image_extension
												)
				print("Done Capture: %s"%count)
				count += 1
			if GPIO.input(terminate_pin):
				break
	except KeyboardInterrupt:
		print("All Completed")

	# stop all stream before killing the camera process
	picamera.stop_preview_stream()
	picamera.close()
	
if __name__ == '__main__':
	main()
