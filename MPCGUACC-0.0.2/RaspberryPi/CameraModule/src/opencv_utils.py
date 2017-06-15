#Standard Libraries

#Third Parties Libraries
import cv2
import numpy as np

#Local Libraries


def rotate_image(image, degree):
	# get image width, height, and center
	width, height = image.shape[:2]
	center = (width / 2, height / 2)

	# calculate rotation matrix for a given degree by openCV
	rotation_matrix = cv2.getRotationMatrix2D(center, degree, 1.0)
	return cv2.warpAffine(image, rotation_matrix, (width, height))

def save_image(image, dest):
	cv2.imwrite(dest, image)

def read_image(src):
	return cv2.imread(src)

def show_image(window_name, image):
	cv2.imshow(window_name, image)
	cv2.waitKey(0)

def decode_image(data):
	return cv2.imdecode(data, 1)

def find_chessboard(image, ny, nx):
	return cv2.findChessboardCorners(image (ny, nx), flags=cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

def draw_chessboard(image, ny, nx, corners, found):
	# make a copy of the image in grayscale
	temp_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	cv2.drawChessboardCorners(temp_img, (ny, nx), corners, found)
	return temp_img

def numpy_to_opencv(src):
	# because am not sure how to convert it, so we will just switch color
	image = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
	image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
	return image