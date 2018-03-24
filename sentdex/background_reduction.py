import numpy as np
import cv2

# In this OpenCV with Python tutorial, we're going to be covering how to reduce the background of images,
# by detecting motion. This is going to require us to re-visit the use of video, or to have two images,
# one with the absense of people/objects you want to track, and another with the objects/people there.

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture('people-walking.mp4')  # people walking sample video
fgbg = cv2.createBackgroundSubtractorMOG2()

while (1):
    ret, frame = cap.read()

    # Convert image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Smooth and Clean up image noises using Guassian Blur
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # # Extract edges
    # canny_edges = cv2.Canny(gray_blur, 10, 70)

    fgmask = fgbg.apply(gray_blur)

    cv2.imshow('fgmask', frame)
    cv2.imshow('frame', fgmask)

    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
