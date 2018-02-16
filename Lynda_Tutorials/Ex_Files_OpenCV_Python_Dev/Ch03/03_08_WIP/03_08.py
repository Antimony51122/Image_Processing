# Canny Edges is one type of edge detection algorithm that works quite well to help
# create better separation of objects within the image
# Generally, edge detection algorithms look at the rate or speed at which colour changes across the image

import numpy as np
import cv2

img = cv2.imread("tomatoes.jpg", 1)

# simple threshold
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
res, thresh = cv2.threshold(hsv[:, :, 0], 25, 255, cv2.THRESH_BINARY_INV)
cv2.imshow("Thresh", thresh)

# canny edges
edges = cv2.Canny(img, 100, 70)
cv2.imshow("Canny", edges)

cv2.waitKey(0)
cv2.destroyAllWindows()
