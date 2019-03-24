import sys
import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(1):

    lower_blue = np.array([100,60,60])
    upper_blue = np.array([110,255,255])
    lower_green = np.array([165,20,25])
    upper_green = np.array([180,255,255])
    lower_red = np.array([165,20,25])
    upper_red = np.array([180,255,255])
    threshold = 1000

    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    g_blur = cv2.GaussianBlur(hsv, (5, 5), 0)

    mask = cv2.inRange(g_blur, lower_blue, upper_blue)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours):
        c = max(contours, key = cv2.contourArea)
        area = cv2.contourArea(c)
        if (area > threshold):
            x,y,w,h = cv2.boundingRect(c)
            cv2.drawContours(frame, c, -1, (0, 255, 0), 3)

            print(area)


    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
