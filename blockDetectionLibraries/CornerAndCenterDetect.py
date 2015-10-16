import cv2
import numpy as np
 
'''
#############################################################
#                    IEEE Hardware Team 2016                    #
#       Created by: David Watts AND Will Hsiung                                                                 #
#       Email:          davidwatts94@ufl.edu hsiung.william@ufl.edu                                          #
#                                                                                                                       #
#   Created for:    Corner and center detection             #
#   Created Date:       September 29th, 2015                                    #
#                                                                                                                       #
#   Modified by:                                                                        #
#       Modified date:                                                          #
#       Reason:                                                                 #
#############################################################
'''
 
def SampleTemplate(image, blurRadius = 5, minLineLength = 50, maxLineGap = 130, lowerThresh = 35, upperThresh = 110, houghLineThreshold = 10, cannylow = 50, cannyhigh = 100):
 
        '''
        Given-when-then is an agile TESTING method, but it can be used in software development as well
        It organizes the code where we have something 'Given to us', something we have to do, then
        '''
 
        #Blur Image
        blur = cv2.GaussianBlur(image,(blurRadius,blurRadius),0) 
       
        #Thresholding out to the good stuff and figuring out lines
        ret,thresh1 = cv2.threshold(blur,lowerThresh,upperThresh,cv2.THRESH_BINARY)
        cv2.namedWindow("Thresh", cv2.WINDOW_NORMAL)
        cv2.imshow("Thresh", thresh1)
        edges = cv2.Canny(thresh1, cannylow, cannyhigh)
        rgbEdges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        lines = np.array([])
        lines = cv2.HoughLinesP(edges,1,np.pi/180,houghLineThreshold,lines, minLineLength,maxLineGap)

        #Thicken Lines
        if lines != None:
                for line in lines:
                        for actualLine in line:
                   
                            # print line
                            # print line[0][1]
                            x1 = actualLine[0]
                            y1 = actualLine[1]
                            x2 = actualLine[2]
                            y2 = actualLine[3]
                            # cv::Vec4i v = lines[i];
                            # lines[i][0] = 0;
                            # lines[i][1] = ((float)v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1];
                            # lines[i][2] = src.cols;
                            # lines[i][3] = ((float)v[1] - v[3]) / (v[0] - v[2]) * (src.cols - v[2]) + v[3];
                            cv2.line(rgbEdges,(x1,y1),(x2,y2),(0,255,0),5)

        #Harris Corner Detection
        grayOutline = cv2.cvtColor(rgbEdges,cv2.COLOR_BGR2GRAY)

        grayOutline = np.float32(grayOutline)
        dst = cv2.cornerHarris(grayOutline,9,7,0.10)
        dst = cv2.dilate(dst,None)
        
        image[dst>0.01*dst.max()]=[0,0,255]

        #Coordinates
        for y in range(0,grayOutline.shape[0]):
        	for x in range(0, grayOutline.shape[1]):
        		harris = cv2.cv.Get2D(cv2.cv.fromarray(dst),y,x)#(x,y) values
        		if harris[0] > 0.01*dst.max():
        			print x,y
        			center = (x**2+y**2)**(0.5)
        			
        			cv2.circle(image, (x,y),2,(0,0,255))
        


        cv2.namedWindow("Blur", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Canny Edge Detection", cv2.WINDOW_NORMAL)
        cv2.imshow("Original", image)
        cv2.imshow('Canny Edge Detection', rgbEdges)
        cv2.imshow('Blur', blur)
        cv2.waitKey(100)
 
def main():
        #initialize class
 
        #Use default camera
        camera = cv2.VideoCapture(-1)
 
        while camera.isOpened():
                # _,image = camera.read()
 
                image = cv2.imread('b.JPG', cv2.CV_LOAD_IMAGE_COLOR)
                SampleTemplate(image)
 
 
        #Given I have a  OpenCV image frame
        #imageSource = cv2.imread("../blockImageRepository/yellow/longblock/TopView.JPG")
 
        #SampleTemplate(imageSource)
 
 
        #close openCv windows gracefully
        cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main()