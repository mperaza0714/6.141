#!/usr/bin/env python2

import cv2 as cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

# Instantiate CvBridge
bridge = CvBridge()

#import computer_vision.color_segmentation as color_segmentation

class ConeDetector:
    #Semantic RGB of the cone object
    SEG_LABEL = "model"
    CONE_COLOR = np.asarray([194,253,94])
    SUB_TOPIC = "/tesse/seg_cam/rgb/image_raw"
    PUB_TOPIC = "/relative_cone_px"

    def __init__(self):
        self.sub = rospy.Subscriber(self.SUB_TOPIC, Image, self.callback);
        self.pub = rospy.Publisher(self.PUB_TOPIC, PointStamped, queue_size=5)
        self.debugpub = rospy.Publisher("/debug", Image, queue_size=5)
    def callback(self, msg):
        
        img = bridge.imgmsg_to_cv2(msg)

        #rospy.loginfo(msg)
       # img = np.array(msg)
        bounding_box = ((0,0),(0,0))
        #rospy.loginfo(type(msg))
        #kernel = np.ones((3,3),np.uint8)
        #erosion = cv2.erode(img,kernel,iterations = 2)

        # Convert BGR to HSV
       # hsv = cv2.cvtColor(erosion, cv2.COLOR_BGR2HSV)
        #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #color_range = np.array([8, 40, 35])
        color_range = np.array([16, 80, 70])
        #lower_cone = np.array(self.CONE_COLOR - color_range)
       # upper_cone = np.array(self.CONE_COLOR + color_range)

        lower_cone = np.array([193,250,90])
        upper_cone = np.array([195,255,98])
        #CONE_COLOR = np.asarray([194,253,94])
        mask = cv2.inRange(img, lower_cone, upper_cone)
        #rospy.loginfo(img)
        self.debugpub.publish(bridge.cv2_to_imgmsg(mask, 'passthrough'))

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img,img, mask= mask)
        original = res.copy()

        ROI_number = 0
        imgray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray,255,255,255)
        image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours_max = 0
        rospy.loginfo(len(contours))
        for c in range(len(contours)):
            if cv2.contourArea(contours[c]) > 25:
                #x,y,w,h = cv2.boundingRect(c)
                #cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
                if cv2.contourArea(contours[filtered_contours_max]) < cv2.contourArea(contours[c]):
                        filtered_contours_max = c


        x,y,w,h = cv2.boundingRect(contours[filtered_contours_max])
        cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
        #image_print(original)

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
        rospy.loginfo(x+w/2, y)
	self.pub.publish(x+w/2, y)

if __name__ == "__main__":
    rospy.init_node("cone_detector");
    node = ConeDetector()
    rospy.spin()
