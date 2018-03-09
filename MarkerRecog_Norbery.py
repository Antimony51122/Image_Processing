import cv2
import numpy as np
import imutils


def detect(c):
    peri = cv2.arcLength(c, True)
    area = cv2.contourArea(c)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    return (len(approx), peri, area)

def MarkerDetect(img):

    markers = []
    imghsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    cv2.imshow('frame', img)

    lower_red1 = np.array([170, 70, 50])
    upper_red1 = np.array([180, 255, 255])
    lower_red2 = np.array([0, 70, 50])
    upper_red2 = np.array([10, 255, 255])

    mask1 = cv2.inRange(imghsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(imghsv, lower_red2, upper_red2)
    mask = mask1 | mask2
    # mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((1,3),dtype='uint8'),iterations=3)

    cv2.imshow('mask', mask)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    for c in cnts:

        shape = detect(c)
        # print shape

        #14 > shape[0] > 10 and 200 > shape[1] > 50 and 500 > shape[2] > 150:
        if 14 > shape[0] > 10 and 400 > shape[1] > 50 and 1000 > shape[2] > 150:  # old 820 100
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]))
            cY = int((M["m01"] / M["m00"]))
            cv2.circle(img, (cX, cY), 3, (0, 255, 255), -1)
            markers.append((cX, cY))

            cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
            cv2.putText(img, str(shape[0]), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            cv2.imshow("Image", img)

    #print markers
    # pts = np.array(markers)q
    # cv2.polylines(imgsized, [pts], True, (255, 0, 255), 3)
    # cv2.imshow("Image", imgsized)

    cv2.waitKey(0)

    return markers

def MarkerSort(MarkersList):

    MarkersAvg = [sum(y) / len(y) for y in zip(*MarkersList)]
    marker1 = []
    marker2 = []

    for m in MarkersList:
        if m[1] > (MarkersAvg[1] + 70): marker1.append(m)

    for m in MarkersList:
        if m[1] < (MarkersAvg[1] - 70): marker2.append(m)

    marker1.sort(key=lambda tup: tup[0])
    marker2.sort(key=lambda tup: tup[0])

    return (marker1 + marker2)

if __name__ == '__main__':
    img = cv2.imread('img16.jpg')
    imgsized = imutils.resize(img, width=640)

    print(MarkerDetect(imgsized))
    cv2.destroyAllWindows()