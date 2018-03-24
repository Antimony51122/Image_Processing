import numpy as np
import cv2

img = cv2.imread("faces.jpeg", 1)
path = "haarcascade_frontalface_default.xml"

gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

face_cascade = cv2.CascadeClassifier(path)

faces = face_cascade.detectMultiScale(gray, scaleFactor=1.05, minNeighbors=5, minSize=(40, 40))
print(len(faces))

for (x, y, w, h) in faces:
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

cv2.imshow("Image", img)

cv2.waitKey(0)
cv2.destroyAllWindows()