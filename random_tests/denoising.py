import numpy as np
import cv2

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture(1)  # external webcam

while (1):

    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cv2.imshow('Original', frame)

    # dst = cv2.fastNlMeansDenoisingColored(frame, None, 11, 6, 7, 21)  # This is costing enormous amount of memory
    # cv2.imshow('Fast Means Denoising', dst)

    edges = cv2.Canny(frame, 100, 200)
    cv2.imshow('Edges', edges)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
