import numpy as np
import cv2

colour = cv2.imread("butterfly.jpg", 1)

gray = cv2.cvtColor(colour, cv2.COLOR_RGB2GRAY)
cv2.imwrite("gray.jpg", gray)
# when you run the script, instead of showing it in the user interface it's going to simply write this file out to the disc

b = colour[:, :, 0]
g = colour[:, :, 1]
r = colour[:, :, 2]

rgba = cv2.merge((b, g, r, g))
# re-use existing channel and make the non-green parts of the image transparent,
# pass in the G channel where a high value or a very green pixel will show as a high alpha layer, meaning not transparent
cv2.imwrite("rgba.png", rgba)