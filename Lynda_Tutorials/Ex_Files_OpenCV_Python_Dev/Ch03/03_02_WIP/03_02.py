# Segmentation can be done through a variety of different ways but the typical output is a binary image.
# A binary image is something that has values of zero or one.
# Essentially a one indicates the piece of the image that we want to use and a zero is everything else.
# Binary images are a key component of many image processing algorithms. These are pure, non alias black and white images.

import numpy as np
import cv2

bw = cv2.imread('detect_blob.png', 0)
height, width = bw.shape[0:2]
cv2.imshow("Original BW", bw)

binary = np.zeros([height, width, 1], 'uint8')

thresh = 85

for row in range(0, height):
    for col in range(0, width):
        if bw[row][col] > thresh:
            binary[row][col] = 255

cv2.imshow("Slow Binary", binary)

ret, thresh = cv2.threshold(bw, thresh, 255, cv2.THRESH_BINARY)
cv2.imshow("CV Threshold", thresh)

cv2.waitKey(0)
cv2.destroyAllWindows()