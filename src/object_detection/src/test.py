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
class image_test():

    def __init__(self):
        #create opencv instance
        self.bridge = CvBridge()
        #create a message subscriber to the topic "/camera/rgb/image_raw" which receives a message of type Image from sensor_msgs    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.display_callback)
        #create a message publisher to the topic "/camera/test_opencv" that publishes sensor_msgs of type Image
        self.image_pub = rospy.Publisher("/camera/test_opencv",Image)

    #callback function that takes the image feed that is being published on "/camera/rgb/image_raw"     
    def display_callback(self,data):
        try:
            # the method .imgmsg_to_cv2() transforms the ros image into an opencv image
            # and the parameters that it takes in are the data, which the stuff that is being published
            # to the topic and the encoding of the image, in this case it is "bgr8"
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # this prints the text "HELLO IMAGE!" on the video feed.
            # args that .putText takes in
            # cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
            cv2.putText(cv_image, "HELLO IMAGE!", (50,70), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 0), 2, 1)
            # this turns the opencv video feed back into a ros video feed and publishes to the topic

            ###### Attempt to find balck basic shapes on image ######
            lower = np.array([0, 0, 0])
            upper = np.array([15, 15, 15])
            shapeMask = cv2.inRange(cv_image, lower, upper)
            # find the contours in the mask
            (cnts, _) = cv2.findContours(shapeMask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
            cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]
            for c in cnts:
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                # if our approximated contour has four points, then
                # we can assume that we have found our square
                global screenCnt 
                if len(approx) == 4:
                    screenCnt = approx
                    break
                # draw the contour and show it    
                cv2.drawContours(cv_image, [screenCnt], -1, (0, 255, 0), 2)
            ###########################################################
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
    #no idea what this does. It seems to be an opencv standard
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()    



