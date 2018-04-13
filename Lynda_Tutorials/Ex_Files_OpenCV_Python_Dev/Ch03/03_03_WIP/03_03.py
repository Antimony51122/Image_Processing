import numpy as np
import cv2

img = cv2.imread('sudoku.png', 0)
cv2.imshow("Original", img)

ret, thresh_basic = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)
cv2.imshow("Basic Binary", thresh_basic)

# adaptive threshold
thresh_adapt_0 = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 0)
thresh_adapt = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
# 115 is the neigbourhood parameter indicating how ar or what the localisation of where the adaptive thresholding will act over
# 1 is mean subtraction from the end result
cv2.imshow("Adaptive Threshold without subtraction", thresh_adapt_0)
cv2.imshow("Adaptive Threshold", thresh_adapt)

cv2.waitKey(0)
cv2.destroyAllWindows()
