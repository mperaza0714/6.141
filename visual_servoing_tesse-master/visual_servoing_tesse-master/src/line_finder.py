#!/usr/bin/env python2

import numpy as np
import rospy
import csv
import cv2
from visual_servoing_tesse.msg import LaneLine
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from tf import TransformListener

class LineFinder:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("scan_topic", "/tesse/front_lidar/scan")
    SEG_CAM_TOPIC = rospy.get_param("seg_cam_topic", "/tesse/seg_cam/rgb/image_raw")
    
    LANE_LINE_TOPIC = rospy.get_param("lane_line_topic", "/lane_line")
    STEERING_RADIUS = 2*np.pi/9 #tesse car 2pi/9
    
    
    def __init__(self, lane_rgba=[0,0,0,0]):
      self.lane_rgba = lane_rgba

      #Publishers
      #self._drive_publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
      self._lane_line_publisher = rospy.Publisher(self.LANE_LINE_TOPIC, LaneLine, queue_size=10)

      #Subscribers
      rospy.Subscriber(self.SEG_CAM_TOPIC, Image, self._onSegCamDataReceived)
      
      #CV Bridge
      self.bridge = CvBridge() #Converts between ROS images and OpenCV Images
       
    def _onSegCamDataReceived(self, data):
        ## YOUR CODE
	cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	np_image = np.array(cv_image)
	
	#Mask for line detection
	line_color = np.array(self.lane_rgba[0:3])
	mask = cv2.inRange(np_image, line_color, line_color)
	#image_print(np_image)
	#image_print(mask)

	#Hough Transform
	lines = cv2.HoughLines(mask,1,np.pi/180.0,30) # (mask, rho accuracy, theta accuracy, threshold)
	
	"""
	#Probabilistic Hough Transform (might not need)
	minLineLength = 100
	maxLineGap = 10
	lines = cv2.HoughLines(mask,1,np.pi/180,200, minLineLength, maxLineGap) # (mask, rho accuracy, theta accuracy, threshold)
	"""

	#Publish Lane Line
	m, b = line_averager(np_image, lines)
	lane_line = LaneLine()
	lane_line.m = m
	lane_line.b = b
	self._lane_line_publisher.publish(lane_line)

def line_averager(img, lines):
	"""
	Helper function to go through Hough Transform Lines and find the average line
	"""
	slopes = []
	intercepts = []
	for line in lines:
		for rho,theta in line:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))
			cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 2)
			slope = float(y2-y1)/float(x2-x1)
			intercept = float(y1)-slope*float(x1)
			slopes.append(slope)
			intercepts.append(intercept)

	#rospy.loginfo(intercepts)	
	m = sum(slopes)/len(slopes)
	b = sum(intercepts)/len(intercepts)
	#avg_x0, avg_y0 = -200, int(m*-200+b)
	#avg_x1, avg_y1 = 3000, int(m*3000+b)
	#cv2.line(img, (avg_x0, avg_y0), (avg_x1, avg_y1), (0, 255, 0), 2)
	#image_print(img)
	return m, b

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	
def load_lane_color(path, semantic_label):
	"""
	Helper function to extract color associated with semantic label
	"""
    	with open(path) as csvfile:
      		reader = csv.DictReader(csvfile, fieldnames = ['label','r','g','b','a'])
      		for row in reader:
        		if row['label'] == semantic_label:
          			return [int(row['r']), int(row['g']), int(row['b']), int(row['a'])]

if __name__ == "__main__":
    rospy.init_node('visual_servoing_tesse')
    #csv file and semantic label to extract color
    csv_path = rospy.get_param("/line_finder/csv_path")
    lane_semantic_label = "Decal_Road_Border_left(Clone)"
    color = load_lane_color(csv_path, lane_semantic_label)
    rospy.loginfo("lane color found: " + str(color))
    
    line_finder = LineFinder(lane_rgba = color)

    rospy.spin()
