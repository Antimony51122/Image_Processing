import numpy as np
import cv2

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture(1)  # external webcam

while (1):

    _, frame = cap.read()
    cv2.imshow('Original', frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Gray', gray)

    # Threshold the image
    ret, thresh = cv2.threshold(gray, 176, 255, 0)

    # Find contours
    _, contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    # Sort Contours by area and then remove the largest frame contour
    n = len(contours) - 1  # if deleting this, the program will take the square outline of the image as a contour

    contours = sorted(contours, key=cv2.contourArea, reverse=False)[:n]

    count = 0
    # Iterate through contours and draw the convex hull
    for c in contours:
        hull = cv2.convexHull(c)
        cv2.drawContours(frame, [hull], 0, (0, 255, 0), 2)
        cv2.imshow('Convex Hull', frame)
        # count += 1
        # cv2.imwrite('images/hand_contour{num}.jpg'.format(num=count), frame)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:  # Esc key
        break

cv2.destroyAllWindows()
cap.release()






