#!/usr/bin/env python
import cv2
import rospy
import roslib
import numpy as np
import os.path
import zbar
roslib.load_manifest('object_detection')
from sensor_msgs.msg import Image
from PIL import Image as pilImg
from cv_bridge import CvBridge, CvBridgeError



'''
#############################################################
#   Created by:     IEEE Hardware Team 2016                 #
#   Created for:    The use of detecting images with a      #
#                   Cascade Classifier given a trained file #
#                                                           #    
#   Modified Date:  Sep. 08th, 2015                         #
#   Modified by:    Nicolas Fry                             #        
#   Reason Mod.:    Created Cropping of images instead of   #
#                   Binary add, also integrating zbar for   #
#                   Decoding purposes                       #
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
        #around the image. 

        #initialize ZBAR by creating a reader 
        scanner = zbar.ImageScanner()
        scanner.parse_config('enable')

        #The list cropped serves to hold the different instances of QR codes detected.
        cropped = []

        indexOfCropped = 0
        for (x,y,w,h) in located:
            locatedcount = locatedcount + 1
            cropped.append(cv_image[(y - 5):(y + h + 5), (x - 5):(x + w + 5)])
            cv2.rectangle(cv_image, (x-25 ,y -5), (x+w + 5, y+h + 5), (0,255,0), 1)
	
	#Debug to see how many are in cropped
        #print len(cropped)
        if(len(cropped) > 0 and len(cropped) < 6):
            imageIndex = 0

            for image in cropped:

                imgRGB = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		#height, width, channels = imgRGB.shape
		height, width = imgRGB.shape
		raw = imgRGB.tostring()
                zbar_image = zbar.Image(width, height, 'Y800', raw)
		
		scanner.scan(zbar_image)

		#print zbar_image
		# For debug purposes 
		for symbol in zbar_image:
			print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data

		
		
                #cv2.imshow(str(imageIndex), image)
                imageIndex += 1


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



