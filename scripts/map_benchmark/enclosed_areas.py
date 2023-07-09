import cv2 as cv
import sys
import numpy as np

occupied_value = 0
free_value = 254
unknown_value = 205

image = cv.imread(sys.argv[1], -1)