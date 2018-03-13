import cv2
import numpy as np

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0)  # external webcam

while (1):

    _, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    """Harris Corner Detection"""
    # # The cornerHarris function requires the array datatype to be float32
    # gray = np.float32(gray)
    #
    # harris_corners = cv2.cornerHarris(gray, 3, 3, 0.05)
    #
    # # We use dilation of the corner points to enlarge them\
    # kernel = np.ones((7, 7), np.uint8)
    # harris_corners = cv2.dilate(harris_corners, kernel, iterations=2)
    #
    # # Threshold for an optimal value, it may vary depending on the image.
    # frame[harris_corners > 0.025 * harris_corners.max()] = [255, 127, 127]
    #
    # cv2.imshow('Harris Corners', frame)

    """Improved Corner Detection using - Good Features to Track"""
    # We specific the top 50 corners
    corners = cv2.goodFeaturesToTrack(gray, 100, 0.01, 15)

    # returns
    for corner in corners:
        x, y = corner[0]
        x = int(x)
        y = int(y)
        cv2.rectangle(frame, (x - 10, y - 10), (x + 10, y + 10), (0, 255, 0), 2)

    cv2.imshow("Corners Found", frame)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
