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
#                IEEE Hardware Team 2016                    #
#   Created by: Austin Seber                                #
#   Email:      aseber@techsouth.cc                         #
#                                                           #
#   Created for:    Generic publisher for cameras           #
#   Created Date:   September 27th, 2015                    #
#                                                           #
#   Modified by:                                            #
#   Modified date:                                          #
#   Reason:                                                 #
#############################################################
'''

def sendCameraFrame(image):

    self.image_pub = rospy.Publisher("/camera/rgb/image_color",image)

def main():

    rospy.init_node('GenericCameraSender', anonymous = True)
    camera = cv2.VideoCapture(-1)

    while(true):

        _, image = camera.read()
        sendCameraFrame(image)

if __name__ == '__main__':
    main()