#!/usr/bin/env python
import cv2
import rospy
import roslib
import numpy as np
roslib.load_manifest('object_detection')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

'''

This script uses hsv (hue, saturation and value) thresholds to attempt
and identify colors. It the proceeds to find the contour of the identified
shape, the finds the centroid of said contour and draws a rectangle around it.

'''

class color_identifier():

    def __init__(self):
        #create opencv instance
        self.bridge = CvBridge()
        #create a message subscriber to the topic "/camera/rgb/image_raw" which receives a message of type Image from sensor_msgs    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.identify_colors)
        #create a message publisher to the topic "/camera/test_opencv" that publishes sensor_msgs of type Image
        #self.image_pub = rospy.Publisher("/camera/test_opencv",Image)


    def identify_colors(self, ros_image):

        try:
        
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

            #turn image to hsv image
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            #setting up lower and upper hsv bounds
            #these bounds were found using the hue_adjust script
            lower_green = np.array([22,88,0])
            upper_green = np.array([65,255,255])

            lower_blue = np.array([66,47,0])
            upper_blue = np.array([149,255,255])

            lower_yellow = np.array([15,159,164])
            upper_yellow = np.array([107,255,255])


            #erode rectangle elementof size 3x3 pixels
            erode_element = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))

            #dilate rectangle element of size 8x8 pixels
            dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT,(8,8))

            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            mask_yellow = cv2.inRange(hsv,lower_yellow, upper_yellow)

            #erode hsv image to get rid of outliers
            mask_green = cv2.erode(mask_green, erode_element)
            #dilate hsv to recover lost points of interest
            mask_green = cv2.dilate(mask_green, dilate_element)

            #erode hsv image to get rid of outliers
            mask_blue = cv2.erode(mask_blue, erode_element)
            #dilate hsv to recover lost points of interest
            mask_blue = cv2.dilate(mask_blue, dilate_element)

            #erode hsv image to get rid of outliers
            mask_yellow = cv2.erode(mask_yellow, erode_element)
            #dilate hsv to recover lost points of interest
            mask_yellow = cv2.dilate(mask_yellow, dilate_element)

            ###################### FIND YELLOW ###################################           

            (cnts, nah) = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
            cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]

            for c in cnts:

                #get the perimeter of the countour
                peri = cv2.arcLength(c, True)
                #approximate perimeter to generate smooth shape
                approx = cv2.approxPolyDP(c, 0.01 * peri, True)
                screenCnt = approx
                #get the area bounded by the contour
                area = cv2.contourArea(c)

                #if the area is bigger than 30x30 pixels
                if area >= 30 * 30:

                    moment = cv2.moments(c)

                    #getting centroid coordinates 
                    cx = int(moment['m10']/moment['m00'])
                    cy = int(moment['m01']/moment['m00'])
                    h = int(peri / 5)
                    w = int(peri / 20)

                    #Draw yellow rectangle around yellow block
                    cv2.rectangle(cv_image, (cx-h,cy-w), (cx+h, cy+w), (10,240,240), 2)
                    
                    # Write on image
                    #write 'yellow block' and the coordinates of the crentroid
                    yellow_caption = "YELLOW BLOCK (%s,%s)" %(cx,cy)
                    cv2.putText(cv_image, yellow_caption, (cx,cy), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 0, 0), 1, 1)
                    #cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 2)

            ######################################################################################################
            
            ############################### FIND BLUE  #################################

            (blue_cnts, nah2) = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
            cnts = sorted(blue_cnts, key = cv2.contourArea, reverse = True)[:1]

            for c in blue_cnts:

                #get the perimeter of the countour
                peri = cv2.arcLength(c, True)
                #approximate perimeter to generate smooth shape
                approx = cv2.approxPolyDP(c, 0.01 * peri, True)
                screenCnt = approx
                #get the area bounded by the contour
                area = cv2.contourArea(c)

                #if the area is bigger than 30x30 pixels
                if area >= 30 * 30:

                    moment = cv2.moments(c)

                    #getting centroid coordinates 
                    cx = int(moment['m10']/moment['m00'])
                    cy = int(moment['m01']/moment['m00'])
                    h = int(peri / 5)
                    w = int(peri / 20)

                    #Draw yellow rectangle around yellow block
                    cv2.rectangle(cv_image, (cx-h,cy-w), (cx+h, cy+w), (255,0,0), 2)
                    
                    # Write on image
                    #write 'yellow block' and the coordinates of the crentroid
                    blue_caption = "BLUE BLOCK (%s,%s)" %(cx,cy)
                    cv2.putText(cv_image, blue_caption, (cx,cy), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 0, 0), 1, 1) 

            ############################################################################################################
            
            ######################### FIND GREEN ###############################################

            (green_cnts, nah3) = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
            green_cnts = sorted(green_cnts, key = cv2.contourArea, reverse = True)[:1]

            for c in green_cnts:

                #get the perimeter of the countour
                peri = cv2.arcLength(c, True)
                #approximate perimeter to generate smooth shape
                approx = cv2.approxPolyDP(c, 0.01 * peri, True)
                screenCnt = approx
                #get the area bounded by the contour
                area = cv2.contourArea(c)

                #if the area is bigger than 30x30 pixels
                if area >= 30 * 30:

                    moment = cv2.moments(c)

                    #getting centroid coordinates 
                    cx = int(moment['m10']/moment['m00'])
                    cy = int(moment['m01']/moment['m00'])
                    h = int(peri / 5)
                    w = int(peri / 20)

                    #Draw yellow rectangle around yellow block
                    cv2.rectangle(cv_image, (cx-h,cy-w), (cx+h, cy+w), (0,255,0), 2)
                    
                    # Write on image
                    #write 'yellow block' and the coordinates of the crentroid
                    green_caption = "GREEN BLOCK (%s,%s)" %(cx,cy)
                    cv2.putText(cv_image, green_caption, (cx,cy), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 0, 0), 1, 1)

            ####################################################################################################################  
            # Why is it not publishing??   
            #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

            cv2.imshow("COLORS", cv_image)

            key = cv2.waitKey(20)

            if key == 27: 
                exit()

        except CvBridgeError, e:
            print e 

def main():
    #initializes node of name 'image_test'
    rospy.init_node('color_finder', anonymous = True)
    #creates an instance of image_test
    ci = color_identifier()
    #keeps python from exiting
    rospy.spin()
   
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()    

