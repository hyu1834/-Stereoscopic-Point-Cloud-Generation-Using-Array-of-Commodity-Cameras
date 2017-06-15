import os
import sys

def get_images_paths(base_path):
	images_path = []
	for path in os.listdir(base_path):
		full_path = os.path.join(base_path, path)
		if not os.path.isfile(full_path):
			continue
		extension = os.path.splitext(os.path.basename(path))[1]
		if not extension == ".bmp":
			continue

		images_path.append(full_path)

	return sorted(images_path)

base_path = sys.argv[1]

# get all image from calibration folder
left_images_path = get_images_paths(os.path.join(base_path, "L_calibration"))
right_images_path = get_images_paths(os.path.join(base_path, "R_calibration"))



# Generate list for Left images
with open(os.path.join(base_path, "left_images.txt"), 'w') as f:
	for path in left_images_path:
		f.write("%s\n"%path)

with open(os.path.join(base_path, "right_images.txt"), 'w') as f:
	for path in left_images_path:
		f.write("%s\n"%path)

with open(os.path.join(base_path, "all_images.txt"), 'w') as f:
	for image1, image2 in zip(left_images_path, right_images_path):
		f.write("%s\n"%image1)
		f.write("%s\n"%image2)




