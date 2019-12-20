#!/usr/bin/env python

#The required packages are imported here
import rospy,cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image

class ImageFlowerdetect():
    def __init__(self):

        rospy.init_node('image_detect', disable_signals = True)
        # Create a ROS Bridge
        self.ros_bridge = cv_bridge.CvBridge()

        # Subscribe to USB Camera image rect color
        self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)
        
    def image_callback(self,msg):

        # 'image' is now an opencv frame
        image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        #For Red Flowers
        param1 = [0,0,100]
        param2 = [80,80,255]
        self.draw_contours(image,param1,param2,1) 
                
        #For Green Flowers
        param1 = [0,100,0]
        param2 = [80,255,80]
        self.draw_contours(image,param1,param2,2)
        
        #For Blue Flowers
        param1 = [100,0,0]
        param2 = [255,80,80]
        self.draw_contours(image,param1,param2,3) 
        
        #rospy.sleep()

    #draw contours around flowers from given parametes     
    def draw_contours(self,image,param1,param2,color):
        lower = np.array(param1) 
        upper = np.array(param2)
        mask = cv2.inRange(image, lower, upper)
        cx=0
        cy=0
        w=20
        im2,contours,_ = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c)>30:
                M = cv2.moments(c)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if color==1:
                    cv2.rectangle(image,(cx-w,cy-w),(cx+w,cy+w),(0,0,255),2)
                if color==2:
                    cv2.rectangle(image,(cx-w,cy-w),(cx+w,cy+w),(0,255,0),2)
                if color==3:
                    cv2.rectangle(image,(cx-w,cy-w),(cx+w,cy+w),(255,0,0),2)
        cv2.imshow("Flower_detetct", image)
        cv2.waitKey(1)
if __name__ == '__main__':
    while not rospy.is_shutdown():
        temp = ImageFlowerdetect()
        rospy.spin()