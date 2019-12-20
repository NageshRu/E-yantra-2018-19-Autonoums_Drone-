#!/usr/bin/env python

'''
* Team Id 	    : 1223
* Author List	    : Uday Barkade,Vishal Desai,Hemant Koli,Nagesh Rupnar
* Filename    	    : 1223.py
* Functions   	    : arm(),disarm(),position_hold(),calc_pid(),pid_roll(),pid_pitch(),pid_throt(),limit,get_pose()
* Global Variables  : None
* Description       : Drone(Bee) visits the red flower location and touches to the open terminals and when flower circuits 			LED glows then using detectd by using image processing and drone(bee) goes to (x=0.0,y=0.0,z=25.0) point.     
* Theme		    : Pollinator Bee (PB)
'''

#The required packages are imported here
from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy
import time

class DroneFly():
		
	def __init__(self):
		
		rospy.init_node('pluto_fly',disable_signals = True)

		self.pluto_cmd = rospy.Publisher('/drone_command',PlutoMsg,queue_size=1)	
		
		rospy.Subscriber('whycon/poses',PoseArray,self.get_pose)

		#subscriber for blue color detection
		rospy.Subscriber('/detect',Int32,self.get_detect)


		self.cmd = PlutoMsg()
		self.red=0
		self.pre_red=0


		# Position to hold
		self.wp_x = 0
		self.wp_y = 0
		self.wp_z = 0.0

		# Given waypoint coordinates
		self.waypoints=[(-5.20,-0.20,20.50),(0,0,25)]

		
		#Drone Motion Commands
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0

		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0
		
		#PID constants for Roll
		self.kp_roll = 11.3
		self.ki_roll = 0.6
		self.kd_roll = 17

		#PID constants for Pitch
		self.kp_pitch = 5.8
		self.ki_pitch = 0.6
		self.kd_pitch = 15
					

		#PID constants for Throttle
		self.kp_throt = 40	
		self.ki_throt = 0.0
		self.kd_throt = 10.0

		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_throt = 0.0

		# Error sum values
		self.roll_errorsum=0.0
		self.pitch_errorsum=0.0
		self.throt_errorsum=0.0

		# Current Error values
		self.roll_error=0.0
		self.throt_error=0.0
		self.pitch_error=0.0
		
		# Previous Error values 
		self.roll_lasterror=0.0
		self.pitch_lasterror=0.0
		self.throt_lasterror=0.0

		# Loop time for PID computation
		self.last_time = 0.0
		self.loop_time = 0.08	
		self.flower_time = 25
		self.flower_flag=False
		self.flower_last_time = 0.0
		self.flag_time=True
		self.current_time=0.0
		self.lastpoint=False
		self.m=2.5
		# For Tracking current waypoint
		self.i=0
		self.j=1
		rospy.sleep(.1)


	''' 
	* Function Name: arm()
	* Input: None
	* Output: drone should arm
	* Logic: Set the drone command values to arm the drone and publish the topic
	* Example Call: self.arm()
	'''
	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	''' 
	* Function Name: disarm()
	* Input: None
	* Output: drone should disarm
	* Logic: Set the drone command values to disarm the drone and publish the topic
	* Example Call: self.disarm()
	'''
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def drone_start_loc(self):
		self.wp_x=self.drone_x
		self.wp_y=self.drone_y
		_,_,self.wp_z=self.waypoints[0] 
		self.wp_z-=2.5
		# z location is 1st visiting flowers above point
		#print "starting location: ",self.wp_x,self.wp_y,self.wp_z 

	''' 
	* Function Name: position_hold()
	* Input: It requires the correct roll ,correct throt and correct pitch value to move the drone appropriate position
	* Output: Drone should hold the postion of specified Point
	* Logic: Set the drone Motion commands values to move the drone frorward,backward,left and right and publish the topic
	* Example Call: self.position_hold()
	'''
	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		self.drone_start_loc()
		print "arm"
		self.arm()
		rospy.sleep(.1)
		
		while True:
			
			self.calc_pid()

			roll_value = int(1500 - self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
						
			throt_value = int(1500 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1850,1400)

			pitch_value = int(1500 - self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
		
			self.pluto_cmd.publish(self.cmd)
	
	''' 
	* Function Name: calc_pid()
	* Input: It requires time in seconds
	* Output: It should calculate the correct roll,pitch and throt values to move the drone
	* Logic: it calls functions if current time is greater than specified loop time to compute correct roll,pitch and throt values 
	* Example Call: self.calc_pid()
	'''
	def calc_pid(self):
		self.seconds = time.time()
		self.current_time = self.seconds - self.last_time
		if(self.current_time >= self.loop_time):

			# Check is drone on required waypoint or not
			
			self.pid_throt()
			self.pid_roll()
			self.pid_pitch()

				
			#print self.roll_error
			if(self.flower_flag==True ):
				if(self.i<len(self.waypoints)):
					self.wp_x,self.wp_y,self.wp_z=self.waypoints[self.i]
					#print self.waypoints[self.i] 
					if(self.i<len(self.waypoints)-1):
						self.wp_z-=2.9
					else:
						print "Pollination done! Pollinated",self.pre_red,"Red Daylily."
						self.wp_x,self.wp_y,self.wp_z=self.waypoints[self.i-1]
						self.wp_z-=4
						
					self.i+=1
					#print "red deteted ",self.red
				self.flower_flag=False			
							
			if(abs(self.roll_error)<1.0 and abs(self.pitch_error)<1.0 and abs(self.throt_error)<2):
				 
								
				if(self.lastpoint==True):
					self.wp_x,self.wp_y,self.wp_z=self.waypoints[self.i-1]		


				if(self.flag_time and self.i==1):			
					self.flower_last_time=self.seconds
					#print "time",self.flower_last_time
					self.flag_time=False
					self.j=0
						
				#change waypoint destination point from 0 to 4 for 5 points 	
				
					#self.clear_all()
				if((self.seconds - self.flower_last_time)>self.flower_time or self.i==0):
					self.flower_flag=True
					#print "time complted *********",self.i
								
				if((self.seconds - self.flower_last_time)>self.j*4):
					self.wp_z=self.wp_z-self.m
					self.m=-self.m
					self.j+=1
				#print "altitude change  *********",self.m

			if(self.i==len(self.waypoints)):				
				self.lastpoint=True			

			self.last_time = self.seconds
	
	def clear_all(self):
		# clear all previous error values computaion
		self.roll_lasterror=self.roll_errorsum=self.pitch_lasterror=self.pitch_errorsum=self.throt_lasterror=self.throt_errorsum=0	 


	''' 
	* Function Name: pid_roll()
	* Input: It requires target y axis drone point and current drone point on y axis
	* Output: It should calculate error value to move drone left or right to hold on appropriate point
	* Logic: using PID controller compute the error value to drone move to target point at y axis
	* Example Call: self.pid_roll()
	'''


	def pid_roll(self):

		#Compute Roll PID here
		self.roll_error=self.wp_y-self.drone_y
		self.roll_errorsum += self.roll_error*self.loop_time
		diff_error=(self.roll_error-self.roll_lasterror)/self.current_time
		self.correct_roll=self.kp_roll*self.roll_error+self.kd_roll*diff_error+self.ki_roll*self.roll_errorsum;
		self.roll_lasterror=self.roll_error
	
	''' 
	* Function Name: pid_pitch()
	* Input: It requires target x axis drone point and current drone point on x axis
	* Output: It should calculate error value to move drone forward or backward to hold on appropriate point
	* Logic: using PID controller compute the error value to drone move to target point at x axis
	* Example Call: pid_pitch()
	'''
	def pid_pitch(self):

		#Compute Pitch PID here
		self.pitch_error=self.wp_x-self.drone_x
		self.pitch_errorsum += (self.pitch_error*self.loop_time)
		diff_error=(self.pitch_error-self.pitch_lasterror)/self.current_time
		self.correct_pitch=self.kp_pitch*self.pitch_error+self.kd_pitch*diff_error+self.ki_pitch*self.pitch_errorsum;
		self.pitch_lasterror=self.pitch_error

	''' 
	* Function Name: pid_throt()
	* Input: It requires target z axis drone point and current drone point on z axis
	* Output: It should calculate error value to increase or decrease height of the drone to hold on appropriate point
	* Logic: using PID controller compute the error value to drone move to target point at z axis
	* Example Call: self.pid_pitch()
	'''
	def pid_throt(self):

		#Compute Throttle PID here
		self.throt_error=self.wp_z-self.drone_z
		self.throt_errorsum +=(self.throt_error*self.loop_time)
		diff_error=(self.throt_error-self.throt_lasterror)/self.current_time
		self.correct_throt=self.kp_throt*self.throt_error+self.kd_throt*diff_error+self.ki_throt*self.throt_errorsum;
		self.throt_lasterror=self.throt_error
	
	''' 
	* Function Name: get_throt()
	* Input: It requires target z axis drone point and current drone point on z axis
	* Output: It should calculate error value to increase or decrease height of the drone to hold on appropriate point
	* Logic: using PID controller compute the error value to drone move to target point at z axis
	* Example Call: self.pid_pitch()
	'''
	def get_detect(self,value):

		#check red flower detetcted
		self.red=value.data
		if(self.red>self.pre_red ):
			self.flower_flag=True
			self.pre_red=self.red
			#print "previous red change ",self.pre_red


	''' 
	* Function Name: pid_limit()
	* Input: It requires input value,maximum value,minimum value
	* Output: It returns the value between max value and min value
	* Logic: if input value less than min value then min value returns,if input value more than max value then max value returns otherwise input value as it is return
	* Example Call: self.pid_pitch()
	'''	
	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values send to drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value
	''' 
	* Function Name: get_pose()
	* Input: It requires poseArray data which contains co-ordinates of x,y and z of current drone position
	* Output: It sets the values x,y and z to varibles
	* Logic: This is the subscriber function to get the whycon poses.The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
	* Example Call: self.get_pose(PoseArray_varible)
	'''
	def get_pose(self,pose):

		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
					
if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.position_hold()
		rospy.spin()




