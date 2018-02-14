import numpy as np
import cv2

# Global variables
canvas = np.ones([500, 500, 3], 'uint8')*255
radius = 3
colour = (0, 255, 0)
pressed = False  # to know whether the key is pressed or not

# click callback
def click(event, x, y, flags, param):
    global canvas, pressed
    if event == cv2.EVENT_LBUTTONDOWN:
        pressed = True
        cv2.circle(canvas, (x, y), radius, colour, -1)
    elif event == cv2.EVENT_MOUSEMOVE and pressed == True:
        cv2.circle(canvas, (x, y), radius, colour, -1)
    elif event == cv2.EVENT_LBUTTONUP:
        pressed = False

# we want to draw circles when the EVENT_LBUTTONDOWN is triggered

# window initialization and callback assignment
cv2.namedWindow("canvas")
cv2.setMouseCallback("canvas", click)

# Forever draw loop
while True:

    cv2.imshow("canvas", canvas)

    # key capture every 1ms
    ch = cv2.waitKey(1)
    if ch & 0xFF == ord('q'):
        break
    elif ch & 0xFF == ord('b'):
        colour = (255, 0, 0)
    elif ch & 0xFF == ord('g'):
        colour = (0, 255, 0)

cv2.destroyAllWindows()
