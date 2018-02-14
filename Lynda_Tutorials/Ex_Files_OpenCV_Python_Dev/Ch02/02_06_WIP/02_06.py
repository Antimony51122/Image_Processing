# The Gaussian Blur filter smooths an image by averaging pixel values with ist neighbours.
# It's ca;;ed a Gaussian Blur because the averaging has a Gaussian falloff effect
# Pixels that are closer to the target pixel have a higher impact with the average than pixels that are faraway

import numpy as np
import cv2

image = cv2.imread("thresh.jpg")
cv2.imshow("Original", image)

blur = cv2.GaussianBlur(image, (5, 55), 0)  # 5, 55 will blur the image a little bit on the x-axis and a lot on the y-axis
cv2.imshow("Blur", blur)

# Dilation and Erosion filter are two operations that look to expand or contract the foreground pixels of an image
# to help remove or accentuate small pixel details, such as speckles.
# They work by sliding a kernel template, a small square, across an image
# - Dilation effect works to turn black pixels, or background pixels, into white pixels;
# - Erosion filter looks to turn white pixels into black pixels, essentially eating away at the foreground
# The small structuring element that was moved across the image is called the kernel and defines where and how to make a pixel changed by that filter

kernels = np.ones((5, 5), 'uint8')

dilate = cv2.dilate(image, kernels, iterations=1)
erode = cv2.erode(image, kernels, iterations=1)

cv2.imshow("Dilate", dilate)
cv2.imshow("Erode", erode)

cv2.waitKey(0)
cv2.destroyAllWindows()