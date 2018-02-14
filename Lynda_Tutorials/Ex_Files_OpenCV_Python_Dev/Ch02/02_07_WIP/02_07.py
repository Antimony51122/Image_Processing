import numpy as np
import cv2

img = cv2.imread("players.jpg", 1)

# Scale
img_half = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)  # this will result in an image that is half the size in both dimensions of the original
img_stretch = cv2.resize(img, (600, 600))
img_stretch_near = cv2.resize(img, (600, 600), interpolation=cv2.INTER_NEAREST)
# This will indicate we are going to use the nearest interpolation mode instead of the default

cv2.imshow("Half", img_half)
cv2.imshow("Stretch", img_stretch)
cv2.imshow("Stretch near", img_stretch_near)

# Rotation
M = cv2.getRotationMatrix2D((0, 0), -30, 1)  # (0, 0) passing the origin we want the rotation to happen around
rotated = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))
# pass in the shape of our image: img.shape[1] to indicate the second value of the shape of original image
cv2.imshow("Rotated", rotated)

cv2.waitKey(0)
cv2.destroyAllWindows()
