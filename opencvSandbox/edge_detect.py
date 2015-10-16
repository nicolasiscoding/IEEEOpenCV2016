#!/usr/bin/env python
import cv2
import numpy as np

'''

This script opens two windows, one with the original image and another one with the hsv
form of the image. It also opens a little gui with trackbars that can be used to find the 
minimum and maximum hsv values. 

'''

class edge_detector():

    def __init__(self):

        # create trackbars for hsv control
        self.switch_window = cv2.namedWindow("SWITCHES")
        self.low_thresh = cv2.createTrackbar('LOWER', 'SWITCHES', 0, 255, self.nothing)
        self.up_thresh = cv2.createTrackbar('UPPER', 'SWITCHES', 0, 255, self.nothing)

    def detect_edges(self, cv_image):
        
        #turn image to hsv image
        #hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #define threshold from trackbars

        min_thresh = cv2.getTrackbarPos('LOWER','SWITCHES')
        max_thresh = cv2.getTrackbarPos('UPPER','SWITCHES')

        print min_thresh, max_thresh

        blur = cv2.GaussianBlur(gray,(5,5),0)

        blur = cv2.Canny(blur, min_thresh, max_thresh)

        #show both original and EDGE images

        cv2.imshow("EDGE", blur)

        cv2.imshow("ORIGINAL", cv_image)

        cv2.waitKey(20)

    def nothing(self, x):
        pass 

def main():

    vc = cv2.VideoCapture(0)

    #creates an instance of image_test
    ed = edge_detector()

    while True: 

        _, cv_image = vc.read()

        ed.detect_edges(cv_image)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()                 