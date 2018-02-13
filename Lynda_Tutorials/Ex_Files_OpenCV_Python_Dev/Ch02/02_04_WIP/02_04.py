import numpy as np
import cv2

colour = cv2.imread("butterfly.jpg", 1)  # ming that if you didn't have the '1', it would still be the default behaviour
cv2.imshow("Image", colour)
cv2.moveWindow("Image", 0 ,0)  # when cv2 runs the script, it will place the window in the top left hand corner
print(colour.shape)  # giving key parameters such as the height, width and channels
# We can also state out those variables for later use:
height, width, channels = colour.shape

# --> (356, 493, 3)

b, g, r = cv2.split(colour)  # this command will split our channels of the colour image into each of its components as individual matrix

rgb_split = np.empty([height, width*3, 3], 'uint8')

rgb_split[:, 0: width] = cv2.merge([b, b, b])
rgb_split[:, width: width*2] = cv2.merge([g, g, g])
rgb_split[:, width*2: width*3] = cv2.merge([r, r, r])

cv2.imshow("Channels", rgb_split)
cv2.moveWindow("Channels", 0, height)

# --> three pictures with 3 channels independently printed out

hsv = cv2.cvtColor(colour, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(hsv)
hsv_split = np.concatenate((h, s, v), axis=1)  # axis=1 tells these 3 images appear side by side
cv2.imshow("Split HSV", hsv_split)

# --> three pictures with 3 channels independently printed out
# - LHS: hue channel which indicates the type of colour we see in a 360 degree format
# - mid: saturation value which has an indication of how saturated an individual colour is
# - RHS: value channel which indicates only how luminous the channel is

# notice in the hue section all the leaf appears to be about the same colour, the same shade of dark ->
# this is one way that it may be more efficient to operate in the hsv space than the rgb space if we want to isolate a particular colour

cv2.waitKey(0)
cv2.destroyAllWindows()