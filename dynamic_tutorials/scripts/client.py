#!/usr/bin/env python

import sys
import numpy as np
import cv2
import rospy
import dynamic_reconfigure.client

lower = np.array([0, 0, 0])
upper = np.array([0, 0, 0])

def callback(config):
    global lower
    global upper
    lower = np.array([config["h_lower"], config["s_lower"], config["v_lower"]])
    upper = np.array([config["h_upper"], config["s_upper"], config["v_upper"]])

if __name__ == "__main__":
    rospy.init_node("dynamic_client")
    client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback)
    cap = cv2.VideoCapture(0)
    threshold = 1000
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        _, frame = cap.read()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        g_blur = cv2.GaussianBlur(hsv, (5, 5), 0)
        mask = cv2.inRange(g_blur, lower, upper)
        _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
        r.sleep()

    cv2.destroyAllWindows()
    cap.release()
