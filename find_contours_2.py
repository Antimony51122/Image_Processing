import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while (1):
    _, frame = cap.read()

    # Grayscale and binarize
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

    # Find contours
    _, contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    count1 = 0
    # Iterate through each contour and compute the bounding rectangle
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.imshow('Bounding Rectangle', frame)
        # count1 += 1
        # cv2.imwrite('images/house_boundRect{num}.jpg'.format(num=count1), frame)

    count2 = 0
    # Iterate through each contour and compute the approx contour
    for c in contours:
        # Calculate accuracy as a percent of the contour perimeter
        accuracy = 0.03 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, accuracy, True)
        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
        cv2.imshow('Approx Poly DP', frame)
        # count2 += 1
        # cv2.imwrite('images/house_approxPoly{num}.jpg'.format(num=count2), frame)

    cv2.waitKey(0)

cv2.destroyAllWindows()
cap.release()