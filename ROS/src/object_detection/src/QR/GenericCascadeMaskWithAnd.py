#!/usr/bin/env python
import cv2
import rospy
import roslib
import numpy as np
import os.path
roslib.load_manifest('object_detection')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


'''
#############################################################
#   Created by:     IEEE Hardware Team 2016                 #
#   Created for:    The use of detecting images with a      #
#                   Cascade Classifier given a trained file #
#                                                           #    
#   Modified Date:  Aug. 27th, 2015                         #
#   Modified by:    Nicolas Fry                             #        
#   Reason Mod.:    Cleaning up code for commit             #
#                                                           #
#############################################################
'''

'''

To see the output, open a terminal. Type "rqt" (without quotes) click on plugins > 
visualization, then find the topic that the image is being published to
the topic name is /camera/GenericCascade

'''

cascade_path = '/home/nicolas/objectorientation/src/object_detection/src/QR/QRCodeCascadeRC1/cascade.xml'

if os.path.isfile(cascade_path) is not True:
    print '\n\n***********!!!No training file present\n\n'
loadedCascadeClassifier = cv2.CascadeClassifier(cascade_path)

##Added this for debug purposes of creating a window
cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)

class CascadeClassifier():

    def __init__(self):
        #create opencv instance
        self.bridge = CvBridge()

        #create a message subscriber to the topic "/camera/rgb/image_raw" which receives a message of type Image from sensor_msgs    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.display_callback)

        #create a message publisher to the topic "/camera/GenericCascade" that publishes sensor_msgs of type Image
        self.image_pub = rospy.Publisher("/camera/GenericCascade",Image)

    #callback function that takes the image feed that is being published on "/camera/rgb/image_raw"     
    def display_callback(self,data):
        try:
		    # the method .imgmsg_to_cv2() transforms the ros image into an opencv image
         	# and the parameters that it takes in are the data, which the stuff that is being published
         	# to the topic and the encoding of the image, in this case it is "bgr8"
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError, e:
            print e

		#convert image to greyscale

        cv_imagecopy = cv_image.copy()
        greyimage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        greyimage = cv2.equalizeHist(greyimage)

		#Look at greyscale image and find occurances of object create a locatedcount variable for debug purposes
        located = loadedCascadeClassifier.detectMultiScale(greyimage,scaleFactor=5.5,minNeighbors=48,minSize=(38,38), flags = cv2.CASCADE_SCALE_IMAGE)
        locatedcount = 0

        #Place rectangles on the colored frame where the located objected *should be*
        #X and Y should be the bottom left corner of the detected figure, adding the width
        #and the height should yielf the top right corner. Using this, draw a rectangle that is green
        #around the image
        for (x,y,w,h) in located:
            locatedcount = locatedcount + 1
            r = cv2.rectangle(cv_image, (x-25 ,y -5), (x+w + 5, y+h + 5), (0,255,0), -1)

        #create HSV image for thresholding
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        ###print hsv

        #create thresholding variable to look for shades of green (found objects)
        lower_green = np.array([60,50,50], np.uint8)
        upper_green = np.array([60,255,255], np.uint8)

        #create a mask from the HSV to show only the original image
        mask = cv2.inRange(hsv, lower_green, upper_green)
        #cv2.imshow("Mask", mask)

        greenboxtracked = cv2.bitwise_and(cv_image,cv_image, mask= mask)
        cv2.imshow("AND", greenboxtracked)

        finalthresh = cv2.bitwise_and(greenboxtracked, cv_imagecopy)
        cv2.imshow("finalthresh", finalthresh)

        #Show cv_image with rectangles in window because publisher below is not working for me
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

        #publish data to topic listed above
        try:
           self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
           
        except CvBridgeError, e:
            print e

        

def main():
    #initializes node of name 'CascadeClassifier'
    rospy.init_node('CascadeClassifier', anonymous = True)

    #creates an instance of CascadeClassifier
    ic = CascadeClassifier()

    #keeps python from exiting
    rospy.spin()

    #Destroy open windows to prevent memory leaks
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()    



