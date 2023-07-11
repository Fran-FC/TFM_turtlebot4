
import cv2 as cv
import sys
import numpy as np

occupied_value = 0
free_value = 254
unknown_value = 205

for i in range(1,len(sys.argv)):
    image = cv.imread(sys.argv[i],-1)


    _, binary_image = cv.threshold(image, unknown_value, free_value, cv.THRESH_BINARY)#remove unknown values
    image = np.float32(binary_image)
    # Apply corner detection
    dst = cv.cornerHarris(image, blockSize=3, ksize=3, k=0.04)

    # Threshold the corner response
    threshold = 0.01 * dst.max()
    corners = np.where(dst > threshold)

    print(len(corners[0]))
    # # Draw circles around the detected corners
    # Draw circles around the detected corners
    rgb_image = cv.cvtColor(image, cv.COLOR_GRAY2RGB)

    radius = 2
    color = (0, 255, 0)  # Green color (BGR format)
    for corner in zip(corners[1], corners[0]):
        cv.circle(rgb_image, corner, radius, color, 1)

    # Print the number of corners
    num_corners = len(corners[0])
    print("Number of corners:", num_corners)

    # Display the result
    cv.imwrite("%s.png" % sys.argv[i], rgb_image)
