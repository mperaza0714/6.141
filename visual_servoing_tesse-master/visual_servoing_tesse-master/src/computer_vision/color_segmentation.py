import cv2
import imutils
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########

	bounding_box = ((0,0),(0,0))

        kernel = np.ones((3,3),np.uint8)
        erosion = cv2.erode(img,kernel,iterations = 2)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(erosion, cv2.COLOR_BGR2HSV)

        lower_orange = np.array([8, 175, 175])
        upper_orange = np.array([25, 255, 250])
		# TODO: Use 'template' to generate the expected min/max HSV values.

        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img,img, mask= mask)
        original = res.copy()

        ROI_number = 0
        img_gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(img_gray,255,255,255)
        image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours_max = 0

        for c in range(len(contours)):
            if cv2.contourArea(contours[c]) > 25:
                #x,y,w,h = cv2.boundingRect(c)
                #cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
                if cv2.contourArea(contours[filtered_contours_max]) < cv2.contourArea(contours[c]):
                        filtered_contours_max = c


        x,y,w,h = cv2.boundingRect(contours[filtered_contours_max])
        cv2.rectangle(img, (x, y), (x + w, y + h), (36,255,12), 2)
        image_print(img)      


	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return ((x, y), (x + w, y + h))
