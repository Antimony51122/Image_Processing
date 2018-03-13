import cv2
import numpy as np

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0)  # external webcam

while (1):

    _, frame = cap.read()

    # Create our shapening kernel, we don't normalize since the values in the matrix sum to 1
    kernel_sharpening = np.array([[-1, -1, -1],
                                  [-1, 9, -1],
                                  [-1, -1, -1]])
    sharpened = cv2.filter2D(frame, -1, kernel_sharpening)

    # dst = cv2.fastNlMeansDenoisingColored(sharpened, None, 11, 6, 7, 21)  # so lag
    # cv2.imshow('Fast Means Denoising', dst)  # so lag

    # canny edges block
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([30, 150, 50])
    upper_red = np.array([255, 255, 180])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('Original', frame)
    cv2.imshow('Sharpened', sharpened)

    edges = cv2.Canny(frame, 50, 120)
    cv2.imshow('Edges', edges)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
