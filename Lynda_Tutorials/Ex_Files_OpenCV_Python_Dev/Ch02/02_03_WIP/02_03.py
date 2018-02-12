import numpy as np
import cv2

black = np.zeros([150, 200, 1], 'uint8')
cv2.imshow("Black", black)
print(black[0, 0, :])

ones = np.ones([150, 200, 3], 'uint8')
cv2.imshow("Ones", ones)
print(ones[0, 0, :])

white = np.ones([150, 200, 3], 'uint16')
white += (2 ** 16 - 1)
cv2.imshow("White", white)
print(ones[0, 0, :])

colour = ones.copy()  # a deep copy means it is completely copied all its memory space, meaning the two are no longer connected to each other at all
colour[:, :] = (255, 0, 0)
cv2.imshow("Bluee", colour)
print(colour[0, 0, :])

cv2.waitKey(0)
cv2.destroyAllWindows()  # this will make sure the user interface will hang until we have taken a look at the image before closing all the windows

