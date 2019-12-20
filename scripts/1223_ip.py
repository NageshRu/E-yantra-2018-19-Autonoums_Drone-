#!/usr/bin/env python

'''
* Team Id : 1223
* Author List : Uday Barkade,Vishal Desai,Hemant Koli,Nagesh Rupnar
* Filename: 1223_ip.py
* Theme: Pollinator Bot (PB)
* Functions: image_callback(),image_callback_whycon,draw_contours()
* Global Variables: None
'''

#The required packages are imported here
import rospy,cv2, cv_bridge
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

class ImageFlowerdetect():
	"""
		This class is used to detetct red,green and blue color objects and draw rectangle contour around of that 
	""" 
	def __init__(self):

		rospy.init_node('image_detect', disable_signals = True)

		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()

		# Subscribe to USB Camera image_rect_color
		self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)

		# Subscribe to whycon image_out 
		self.image_sub = rospy.Subscriber('/whycon/image_out', Image, self.image_callback_whycon)
	
		# This is the final image which shows the whycon co-ordinates and detetcted flowers using contours	
		self.final_image=None

		#for blue detection
		self.red=0
		self.color_publish = rospy.Publisher('/detect',Int32,queue_size=10)	


	''' 
	* Function Name: image_callback()
	* Input: Image is send by subcriber when image is available
	* Output:It finds the red,green and blue colors flowers and draw contour around of that
	* Logic: using lower limit and upper limit create the mask image and find the color
	* Example Call: self.image_callback()
	'''  
	def image_callback(self,msg):

		# 'image' is now an opencv frame
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		#Lower and Upper limit for Red Flowers
		param1 = [0,0,100]
		param2 = [80,80,255]
		self.draw_contours(image,param1,param2,1) 
		
		#Lower and Upper limit for Green Flowers
		param1 = [0,100,0]
		param2 = [80,255,80]
		self.draw_contours(image,param1,param2,2)

		#Lower and Upper limit for Blue Flowers
		param1 = [100,0,0]
		param2 = [255,80,80]
		self.draw_contours(image,param1,param2,3) 
	
	''' 
	* Function Name: image_callback_whycon()
	* Input: Image is send by subcriber when image is available
	* Output:It assign the image to final_image 
	* Logic: using ros bridge convert the image message to cv2 
	* Example Call: self.image_callback_whycon()
	''' 
	def image_callback_whycon(self,msg):
		self.final_image=self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

	''' 
	* Function Name: draw_contours()
	* Input: cv2 image,lower limit color array,upper limit array,color value
	* Output: It draws the rectangle contour around the detetcts color on final_image
	* Logic: using masking ,erosion and dilate function detetcts and draw contour  
	* Example Call: self.image_callback_whycon()
	'''
	def draw_contours(self,image,param1,param2,color):
	
		lower = np.array(param1) 
		upper = np.array(param2)
	
		#create mask image using lower and upper limit
		mask = cv2.inRange(image, lower, upper)
		
		#kernel for erosion
		kernele = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
		
		#kernel for dilate		
		kerneld = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10,10))

		#It removes objects which pixels are small
		erosion = cv2.erode(mask,kernele,iterations = 1)
		
		#It Increases the object area
		opening = cv2.dilate(erosion, kerneld, iterations =3)
		
		#width for contour rectangle
		w=20
 
		im2,contours,_ = cv2.findContours(opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
		for c in contours:
		    	#check contour area is big
			if cv2.contourArea(c)>10:
				M = cv2.moments(c)
				
				#cx and cy is the centroid of finded contour
				cx = int(M["m10"] / M["m00"])
				cy = int(M["m01"] / M["m00"])

				#check color for draw appropriate color around contour 1 for Red,2 for Green and 3 For Blue
				if color==1:
				    cv2.rectangle(self.final_image,(cx-w,cy-w),(cx+w,cy+w),(0,0,255),2)
				    self.red=1
				if color==2:
				    cv2.rectangle(self.final_image,(cx-w,cy-w),(cx+w,cy+w),(0,255,0),2)
				if color==3:
				    cv2.rectangle(self.final_image,(cx-w,cy-w),(cx+w,cy+w),(255,0,0),2)
				    
				
		#Display image processed image				
		cv2.imshow("Final_image",self.final_image)
		self.color_publish.publish(self.red);
		cv2.waitKey(1)

if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = ImageFlowerdetect()
		rospy.spin()
