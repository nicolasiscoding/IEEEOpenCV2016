#!/usr/bin/env python
import cv2
import rospy
import roslib
import numpy as np
roslib.load_manifest('object_detection')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#### WORKING EXAMPLE ####

'''

To see the output, open a terminal. Type "rqt" (without quotes) click on plugins > 
visualization, then find the topic that the image is being published to
the topic name is /camera/test_opencv

'''


cascade_path = '/path/to/cascade.xml'
candy_cascade = cv2.CascadeClassifier(cascade_path)

class image_test():

    def __init__(self):
        #create opencv instance
        self.bridge = CvBridge()

        #create a message subscriber to the topic "/camera/rgb/image_raw" which receives a message of type Image from sensor_msgs    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.display_callback)

        #create a message publisher to the topic "/camera/test_opencv" that publishes sensor_msgs of type Image
        self.image_pub = rospy.Publisher("/camera/candyfinder",Image)

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
        greyimage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		#Look at greyscale frame and find the rectangles
        candy = candy_cascade.detectMultiScale(greyimage)
        candycount = 0
		
        #Place rectangles on the colored frame where the candy box *should be*
        #X and Y should be the bottom left corner of the detected figure, adding the width
        #and the height should yielf the top right corner. Using this, draw a rectangle thats blue
        #around the image
        for (x,y,w,h) in candy:
            candycount = candycount + 1
            cv2.rectangle(cv_image, (x,y), (x+w, y+h), (255,0,0), 2)

        #Show cv_image with rectangles in window because publisher below is not working for me
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

        #publish data to topic listed above
        try:
           self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
           
        except CvBridgeError, e:
            print e

        

def main():
    #initializes node of name 'image_test'
    rospy.init_node('image_test', anonymous = True)

    #creates an instance of image_test
    ic = image_test()

    #keeps python from exiting
    rospy.spin()

    #Destroy open windows to prevent memory leaks
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()    



