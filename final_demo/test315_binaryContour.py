# Goal: for all objects in the image, segment them out, draw them on a blank image and print the perimeter adn area

import numpy as np
import cv2
import random

cap = cv2.VideoCapture(0)

while (1):

    _, frame = cap.read()
    cv2.imshow("Original", frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)

    thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 205, 1)
    # inversing the threshold because the foreground is white and we want to take out the objects which are going to be darker than this colour
    cv2.imshow("Binary", thresh)

    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # underscore to not use the first return
    print(len(contours))

    filtered = []
    for c in contours:
        if cv2.contourArea(c) < 1000:continue
        filtered.append(c)

    print(len(filtered))

    objects = np.zeros([frame.shape[0], frame.shape[1], 3], 'uint8')
    for c in filtered:
        col = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        cv2.drawContours(objects, [c], -1, col, -1)
        area = cv2.contourArea(c)
        p = cv2.arcLength(c, True)
        print(area, p)

    cv2.imshow("Contours", objects)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()