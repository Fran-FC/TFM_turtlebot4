import cv2 as cv
import sys
import numpy as np

occupied_value = 0
free_value = 254
unknown_value = 205

image = cv.imread(sys.argv[1], -1)

#remove unknown values
image = image.astype(float)
image[image == unknown_value] = np.nan

# count image size without unknown values
img_size = np.count_nonzero(~np.isnan(image))

# get proportions
occupied_proportion = np.sum(image == occupied_value) / img_size
free_proportion = np.sum(image == free_value) / img_size

print("occupied:" , occupied_proportion)
print("free:" , free_proportion)

