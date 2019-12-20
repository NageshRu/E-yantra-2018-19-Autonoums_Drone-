#!/usr/bin/env python

#The required packages are imported here
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy,cv2, cv_bridge
import time
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist

class DroneFly():
	"""docstring for DroneFly"""
	def __init__(self):
		
		rospy.init_node('pluto_fly', disable_signals = True)

		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)	
	
		self.puberrorpitch = rospy.Publisher('/pitch_error', Float64, queue_size=10) #For publish pitch error value
		self.puberrorthrot = rospy.Publisher('/alt_error', Float64, queue_size=10)	 #For publish throat error value
		self.puberrorroll = rospy.Publisher('/roll_error', Float64, queue_size=10)	 #For publish roll error value
		self.pubzerodata = rospy.Publisher('/zero_line', Float64, queue_size=10)	 #For publish Zero value
		
		self.pubred=rospy.Publisher('/red',Int32,queue_size=10)						 #For publish red patches values
		self.pubblue=rospy.Publisher('/blue',Int32,queue_size=10)					 #For Publish Blue Patches Values
		self.pubgreen=rospy.Publisher('/green',Int32,queue_size=10)					 #For Publish Green Patches Values
		
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)

		# To tune the drone during runtime
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)

		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()

		# Subscribe to vision sensor image rect
		self.image_sub = rospy.Subscriber('visionSensor/image_rect', Image, self.image_callback)
		
		self.cmd = PlutoMsg()

		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0
		
		# Given waypoint coordinates
		self.waypoints=[( -5.63, -5.63, 30), ( 5.57, -5.63, 30), ( 5.55, 5.54, 30), ( -5.6, 5.54, 30), (0.0, 0.0, 30)]

		self.bluecount=0
		self.redcount=0
		self.greencount=0
	
		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0

		#PID constants for Roll
		self.kp_roll = 7
		self.ki_roll = 0.0
		self.kd_roll = 0.5

		#PID constants for Pitch
		self.kp_pitch =7
		self.ki_pitch = 0.0
		self.kd_pitch = 0.5
		
		#PID constants for Yaw
		self.kp_yaw = 0.0
		self.ki_yaw = 0.0
		self.kd_yaw = 0.0

		#PID constants for Throttle
		self.kp_throt = 7.0
		self.ki_throt = 0.0
		self.kd_throt = 7.0

		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

		# Error sum values
		self.roll_errorsum=0.0
		self.pitch_errorsum=0.0
		self.yaw_errorsum=0.0
		self.throt_errorsum=0.0

		# Current Error values
		self.roll_error=0.0
		self.throt_error=0.0
		self.pitch_error=0.0
		self.yaw_error=0.0		

		# Previous Error values 
		self.roll_lasterror=0.0
		self.pitch_lasterror=0.0
		self.yaw_lasterror=0.0
		self.throt_lasterror=0.0

		# Loop time for PID computation.
		self.last_time = 0.0
		self.loop_time = 0.032
		self.current_time=0.0

		# For Tracking current waypoint
		self.i=0

		rospy.sleep(.1)

	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)

		while True:
			
			self.calc_pid()

			roll_value = int(1500 - self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
						
			throt_value = int(1500 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)

			pitch_value = int(1500 - self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
			
			self.pluto_cmd.publish(self.cmd)
	
	def calc_pid(self):
		self.seconds = time.time()
		self.current_time = self.seconds - self.last_time
		if(self.current_time >= self.loop_time):

			# Check is drone on required waypoint or not
			if(abs(self.roll_error)<0.2 and abs(self.pitch_error)<0.2 and abs(self.throt_error) <1.5): 
				
				# If the drone near to ground then landing
				if(self.i==6):
					self.disarm()

				#for landing the drone goes to near to ground
				if(self.i==5):
					self.wp_z=38
					self.clear_all()
					self.i+=1
				#change waypoint destination point from 0 to 4 for 5 points 	
				if(self.i<=4):
					self.wp_x,self.wp_y,self.wp_z=self.waypoints[self.i]   
					self.i+=1
					self.clear_all()
			self.pid_roll()
			self.pid_pitch()
			self.pid_throt()
			self.publish_data() # publish  values
			
			self.last_time = self.seconds

	def clear_all(self):
		# clear all previous error values computaion
		self.roll_lasterror=self.roll_errorsum=self.pitch_lasterror=self.pitch_errorsum=self.throt_lasterror=self.throt_errorsum=0	 

	def pid_roll(self):

		#Compute Roll PID here
		self.roll_error=self.wp_y-self.drone_y
		self.roll_errorsum += self.roll_error*self.current_time
		diff_error=(self.roll_error-self.roll_lasterror)/self.current_time
		self.correct_roll=self.kp_roll*self.roll_error+self.kd_roll*diff_error+self.ki_roll*self.roll_errorsum;
		self.roll_lasterror=self.roll_error
		
	
	def pid_pitch(self):

		#Compute Pitch PID here
		self.pitch_error=self.wp_x-self.drone_x
		self.pitch_errorsum += self.pitch_error*self.current_time
		diff_error=(self.pitch_error-self.pitch_lasterror)/self.current_time
		self.correct_pitch=self.kp_pitch*self.pitch_error+self.kd_pitch*diff_error+self.ki_pitch*self.pitch_errorsum;
		self.pitch_lasterror=self.pitch_error

	def pid_throt(self):

		#Compute Throttle PID here
		self.throt_error=self.wp_z-self.drone_z
		self.throt_errorsum += self.throt_error*self.current_time
		diff_error=(self.throt_error-self.throt_lasterror)/self.current_time
		self.correct_throt=self.kp_throt*self.throt_error+self.kd_throt*diff_error+self.ki_throt*self.throt_errorsum;
		self.throt_lasterror=self.throt_error
		
	def limit(self, input_value, max_value, min_value):

		# To limit the maximum and minimum values send to drone
		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

	def image_callback(self,msg):

		# 'image' is now an opencv frame
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		#cv2.imshow("Input Image", image)
		
		#For Red Patches
		param1 = [0,0,200]
		param2 = [0,0,255]
		self.redcount=self.calculate_patches(image,param1,param2) 
		
		#For Green Patches
		param1 = [0,200,0]
		param2 = [0,255,0]
		self.greencount=self.calculate_patches(image,param1,param2)
		
		#For Blue Patches
		param1 = [200,0,0]
		param2 = [255,0,0]
		self.bluecount=self.calculate_patches(image,param1,param2) 
		
		rospy.sleep(0.2)
		#cv2.waitKey(0)

	#calcute pataches form given parametes 	
	def calculate_patches(self,image,param1,param2):
		lower = np.array(param1) 
		upper = np.array(param2)
		mask = cv2.inRange(image, lower, upper)
		im2,contours,_ = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(image,contours,-1,upper,3)
		cv2.imshow("Input Image", image)
		cv2.waitKey(0)
		return len(contours) 

	#This function to publish different information 
	def publish_data(self):

		#publish error values
		self.puberrorthrot.publish(self.throt_error)
		self.puberrorroll.publish(self.roll_error)
		self.puberrorpitch.publish(self.pitch_error)
		self.pubzerodata.publish(0)

		#publish color count of patches
		self.pubblue.publish(self.bluecount)
		self.pubgreen.publish(self.greencount)
		self.pubred.publish(self.redcount)

		rospy.sleep(0.2)

	def set_pid_alt(self,pid_val):
		
		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

		self.kp_throt = pid_val.Kp
		self.ki_throt = pid_val.Ki
		self.kd_throt = pid_val.Kd

	def set_pid_roll(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

		self.kp_roll = pid_val.Kp
		self.ki_roll = pid_val.Ki
		self.kd_roll = pid_val.Kd
		#print "I am in roll function"
		
	def set_pid_pitch(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

		self.kp_pitch = pid_val.Kp
		self.ki_pitch = pid_val.Ki
		self.kd_pitch = pid_val.Kd
		
	def set_pid_yaw(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

		self.kp_yaw = pid_val.Kp
		self.ki_yaw = pid_val.Ki
		self.kd_yaw = pid_val.Kd
		
	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
		
if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.position_hold()
		rospy.spin()