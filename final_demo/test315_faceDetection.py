import numpy as np
import cv2


cap = cv2.VideoCapture(0)
path = "haarcascade_frontalface_default.xml"

while (1):

    _, frame = cap.read()
    cv2.imshow("Original", frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    face_cascade = cv2.CascadeClassifier(path)

    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.05, minNeighbors=5, minSize=(40, 40))
    print(len(faces))

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.imshow("Image", frame)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()