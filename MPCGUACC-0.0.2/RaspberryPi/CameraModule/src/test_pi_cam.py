from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

resolution = (2592, 1944)
camera = PiCamera()
camera.resolution = resolution
rawCapture = PiRGBArray(camera, size = resolution)

camera.capture(rawCapture, format = "bgr", use_video_port = True)
image = rawCapture.array

image_width, image_height = image.shape[:2]
image_center = (image_width / 2, image_height / 2)

#rotate image by 180 degree
rotation_matrix = cv2.getRotationMatrix2D(image_center, 180, 1.0)
image = cv2.warpAffine(image, rotation_matrix, (image_width, image_height))

cv2.imshow("Image", image)
cv2.waitKey(0)
camera.close()
