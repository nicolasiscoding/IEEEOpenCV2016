#!/usr/bin/env python
import cv2
import numpy as np

'''

This script opens two windows, one with the original image and another one with the hsv
form of the image. It also opens a little gui with trackbars that can be used to find the 
minimum and maximum hsv values. 

'''

class hsv_adjuster():

    def __init__(self):

        # create trackbars for hsv control
        self.switch_window = cv2.namedWindow("SWITCHES")
        self.hmin_bar = cv2.createTrackbar('H_MIN', 'SWITCHES', 0, 255, self.nothing)
        self.smin_bar = cv2.createTrackbar('S_MIN', 'SWITCHES', 0, 255, self.nothing)
        self.vmin_bar = cv2.createTrackbar('V_MIN', 'SWITCHES', 0, 255, self.nothing)   
        self.hmax_bar = cv2.createTrackbar('H_MAX', 'SWITCHES', 0, 255, self.nothing)
        self.smax_bar = cv2.createTrackbar('S_MAX', 'SWITCHES', 0, 255, self.nothing)
        self.vmax_bar = cv2.createTrackbar('V_MAX', 'SWITCHES', 0, 255, self.nothing)

         

    def adjust_hsv(self, cv_image):
        
        #turn image to hsv image
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #define hue, saturation and value from trackbars

        while 1:

            hue_min = cv2.getTrackbarPos('H_MIN','SWITCHES')
            sat_min = cv2.getTrackbarPos('S_MIN','SWITCHES')
            val_min = cv2.getTrackbarPos('V_MIN','SWITCHES')
            hue_max = cv2.getTrackbarPos('H_MAX','SWITCHES')
            sat_max = cv2.getTrackbarPos('S_MAX','SWITCHES')
            val_max = cv2.getTrackbarPos('V_MAX','SWITCHES')


            print  hue_min, sat_min, val_min

            #setting up lower and upper hsv bounds
            lower = np.array([hue_min,sat_min,val_min])
            upper = np.array([hue_max,sat_max,val_max])

            #erode rectangle elementof size 3x3 pixels
            erode_element = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))

            #dilate rectangle element of size 8x8 pixels
            dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT,(8,8))

            mask = cv2.inRange(hsv, lower, upper)

            #erode hsv image to get rid of outliers
            mask = cv2.erode(mask, erode_element)

            #dilate hsv to recover lost points of interest
            mask = cv2.dilate(mask, dilate_element)

            #show both original and HSV images
            cv2.imshow("ORIGINAL", cv_image)

            cv2.imshow("HSV", mask) 
            
            cv2.waitKey(20)

    def nothing(self, x):
        pass 

def main():

    vc = cv2.VideoCapture(0)

    #creates an instance of image_test
    ha = hsv_adjuster()

    if vc.isOpened():

        _, cv_image = vc.read()

        ha.adjust_hsv(cv_image)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()                 