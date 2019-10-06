#!/usr/bin/env python

from plutodrone.msg import *
from pid_tune.msg import *
import rospy
import time
from std_msgs.msg import Float64
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32


class WayPoint:


	def __init__(self):
		rospy.init_node('pluto_fly', disable_signals = True)
		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)	
		self.roll=rospy.Publisher('/drone_roll_error',Float64, queue_size=10)
                self.pitch=rospy.Publisher('/drone_pitch_error',Float64, queue_size=10)
                self.throt=rospy.Publisher('/drone_throt_error',Float64, queue_size=10)
		self.zero=rospy.Publisher('/zero_line',Float64, queue_size=10)
		self.p_1=rospy.Publisher('/plus_1_5',Float64, queue_size=10)
		self.m_1=rospy.Publisher('/minus_1_5',Float64, queue_size=10)
		self.p_0=rospy.Publisher('/plus_0_2',Float64, queue_size=10)
		self.m_0=rospy.Publisher('/minus_0_2',Float64, queue_size=10)
		self.p_8=rospy.Publisher('/plus_0_8',Float64, queue_size=10)
		self.m_8=rospy.Publisher('/minus_0_8',Float64, queue_size=10)
                self.yaw=rospy.Publisher('/drone_yaw_error',Float64, queue_size=10)
		self.loop=rospy.Publisher('/count',Float64, queue_size=10)
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		rospy.Subscriber('yaw_p', Float64, self.get_theta)
		self.cmd = PlutoMsg()
		

################################################################################################
#################################################################################################
	



		
	



###############################################################################################
##################for way points###########################################################


#####################################################################

# landing should be done at the height of 28.5

################################################################################
#enter the coordinates here#####################################################
#p_1--> 00,01,02
#p_2--> 10,11,12
#p_3--> 20,21,22
#p_4--> 30,31,32
#p_5--> 40,41,42		#4 th point is pollinating point.


		self.Plant_location=[[4.45,2.98,23.5],[4.23,-2.46,25.16],[-7.00,3.31,27.0],[6.5,-3.28,32.4]]
#add extra .5 to x value
#subtract extra .5 from y value

#self.Plant_location[][]


################################################################################


		#PID constants for Roll
		self.kp_roll = 55 #
		self.ki_roll = 0#
		self.kd_roll = 1200 #

		#PID constants for Pitch

		self.kp_pitch =80#  
		self.ki_pitch = .7#
		self.kd_pitch =1400#
		
		#PID constants for Yaw
		self.kp_yaw = 100#
		self.ki_yaw = 0#
		self.kd_yaw =100 #

		#PID constants for Throttle
		self.kp_throt = 705# 175
		self.ki_throt =  .1 #
		self.kd_throt = 1140#100
##################################################################################


		
####################################################################################


		self.hold=[-1.85,5.93,23.5]
		self.hold1=[-.96,1.21,25.16]





	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)







	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)



 

	def calc_pid(self):
		self.zero.publish(0)
		self.p_1.publish(1)
		self.m_1.publish(-1)
		self.p_8.publish(.5)
		self.m_8.publish(-.5)
			
		self.seconds = time.time()
		current_time = self.seconds - self.last_time
		if(current_time >= self.loop_time):
			self.pid_yaw()
			self.pid_throt()
			
			self.pid_pitch()
			self.pid_roll()
				
			self.last_time = self.seconds






	def pid_roll(self):
		
		#Compute Roll PID here
               	self.current_error_roll=(self.wp_y - self.drone_y)
               	self.iterm_roll=(self.iterm_roll + self.current_error_roll) * self.ki_roll
               	self.correct_roll=str(self.kp_roll*self.current_error_roll + self.iterm_roll + self.kd_roll*(self.current_error_roll-self.previous_error_roll))
               			
		self.roll.publish(Float64(self.current_error_roll))
		self.previous_error_roll=self.current_error_roll
               	




	
	def pid_pitch(self):

		#Compute Pitch PID here
               	self.current_error_pitch=(self.wp_x - self.drone_x)
               	self.iterm_pitch=(self.iterm_pitch + self.current_error_pitch) * self.ki_pitch
               	self.correct_pitch=str(self.kp_pitch*self.current_error_pitch + self.iterm_pitch + self.kd_pitch*(self.current_error_pitch-self.previous_error_pitch))
       		self.pitch.publish(Float64(self.current_error_pitch))
               	self.previous_error_pitch=self.current_error_pitch





	def pid_throt(self):

		#Compute Throttle PID here
               	self.current_error_throt=(self.wp_z - self.drone_z)
               	self.iterm_throt=(self.iterm_throt + self.current_error_throt) * self.ki_throt
               	self.correct_throt= str(self.kp_throt*self.current_error_throt+self.iterm_throt+self.kd_throt*(self.current_error_throt-self.previous_error_throt))
		self.throt.publish(Float64(self.current_error_throt))
               	self.previous_error_throt=self.current_error_throt


               	





	def pid_yaw(self):

		#Compute yaw PID here	
		
               	self.current_error_yaw=(self.wp_theta - self.drone_yaw_data)
               	self.iterm_yaw=(self.iterm_yaw + self.current_error_yaw) * self.ki_yaw
               	self.correct_yaw= str(self.kp_yaw*self.current_error_yaw+self.iterm_yaw+self.kd_yaw*(self.current_error_yaw-self.previous_error_yaw))
		self.yaw.publish(Float64(self.current_error_yaw))
               	self.previous_error_yaw=self.current_error_yaw
		




               	


		
	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value
	
		#You can use this function to publish different information for your plots
		# def publish_plot_data(self):
          	


	
	
	def get_pose(self,pose):

			#This is the subscriber function to get the whycon poses
			#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z

          	

		
	def get_theta(self,data):
		
		self.drone_yaw_data=data.data
		
			

####################################################################################################

#####################################################################################################


###################color detection code#################################################################


		
########################################################################################################


	



	



##########################################################################################################

###########################################################################################################


##############################################color detection code ends here##############################			

##########################################################################################################



# initialising block for different variables of drone.

	
	#for point-1






	def initialise_p_1(self):

		# Position to hold.

		#print "i was called"

		self.wp_x =self.hold[0]

		self.wp_y = self.hold[1]

		self.wp_z =self.hold[2]

		self.wp_theta=0
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500		
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0
		


		self.n=0.000
		

		self.drone_yaw_data=0.0
		
			
		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

                # Current error
               
                self.current_error_roll=0.0
               	self.current_error_pitch=0.0
               	self.current_error_throt=0.0
 		self.current_error_yaw=0.0
	
                # Previous error
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

               	# Iterm
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0

	# Loop time for PID computation. You are free to experiment with this		
		self.last_time = 0.0
		self.loop_time = 0.032






	#for point-2
	def initialise_p_2(self):


		# Position to hold.

		self.wp_x = self.Plant_location[0][0]

		self.wp_y = self.Plant_location[0][1]

		self.wp_z = self.Plant_location[0][2]

		self.wp_theta=0.000000
		
		
	
#n:by using this variable we are controlling drone's holding time at a position , it's range is equivalent to range of a float value .
		
		self.n=0.000
		
			
	
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm
             
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0


		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
		


	
			
	#for point-3
	def initialise_p_3(self):



		# Position to hold.

		self.wp_x =  self.hold1[0]
		self.wp_y = self.hold1[1]

		self.wp_z = self.hold1[2]

		self.wp_theta=0.000000
			
					
	
	
		self.n=0.000
		
               	# Current error
                
                # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm
             
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0
             

		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		


 			



	#for point-4
	def initialise_p_4(self):



		# Position to hold.

		self.wp_x = self.Plant_location[1][0]

		self.wp_y = self.Plant_location[1][1]

		self.wp_z = self.Plant_location[1][2]

		self.wp_theta=0.000000
			
#n:by using this variable we are controlling drone's holding time at a position , it's range is equivalent to range of a float value .
		
		
		self.n=0.000
		
                # Current error
                # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm
             
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0

		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
			



	#for point-5
	def initialise_p_5(self):



		# Position to hold.
		self.wp_x =  self.hold1[0]
		self.wp_y = self.hold1[1]

		self.wp_z = self.hold1[2]

		self.wp_theta=0.000000
		# Correction values after PID is computed
#n:by using this variable we are controlling drone's holding time at a position , it's range is equivalent to range of a float value .
		self.n=0.000
		
                # Current error
                
                # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm       
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0


		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
################################################################################################





	#for point-6
	def initialise_p_6(self):



		# Position to hold.


		
		self.wp_x = self.Plant_location[2][0]


		self.wp_y = self.Plant_location[2][1]

		self.wp_z = self.Plant_location[2][2]

		self.wp_theta=0.000000
		# Correction values after PID is computed
		
		self.n=0.000  
		             # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
				self.previous_error_yaw=0.0

                # Iterm       
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
				self.iterm_yaw=0.0


		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
################################################################################################




	#for point-7
	def initialise_p_7(self):



		# Position to hold.
		
		self.wp_x =  self.hold1[0]
		self.wp_y = self.hold1[1]

		self.wp_z = self.hold1[2]

		self.wp_theta=0.000000
		# Correction values after PID is computed
		

                # Current error

		self.n=0.000
		
                # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm       
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0


		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
################################################################################################






	#for point-8
	def initialise_p_8(self):



		# Position to hold.
		self.wp_x = self.Plant_location[3][0]

		self.wp_y = self.Plant_location[3][1]

		self.wp_z = self.Plant_location[3][2]

		self.wp_theta=0.000000
		# Correction values after PID is computed

		self.n=0.000
		
                # Current error
                
                # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm       
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0


		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
################################################################################################




	#for point-9
	def initialise_p_9(self):



		# Position to hold.

		self.wp_x =  self.hold1[0]
		self.wp_y = self.hold1[1]

		self.wp_z = self.hold1[2]

		self.wp_theta=0.000000

		self.n=0.000
		
                # Current error
                
                # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm       
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0


		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
################################################################################################




	#for point-10
	def initialise_p_10(self):




		self.wp_x = 0

		self.wp_y = 0

		self.wp_z = 20

		self.wp_theta=0.000000
		
#n:by using this variable we are controlling drone's holding time at a position , it's range is equivalent to range of a float value .
		self.n=0.000
		
             
                
                # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm       
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0


		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
################################################################################################


		
################################################################################################





	#for point-11
	def initialise_p_11(self):



		# Position to hold.

		self.wp_x = self.Plant_location[10][0]

		self.wp_y = self.Plant_location[10][1]

		self.wp_z = self.Plant_location[10][2]

		self.wp_theta=0.000000
		
		self.n=0.000
	
		self.j=0.0
		

                # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm       
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0


		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
################################################################################################
################################################################################################






	#for point-12
	def initialise_p_12(self):



		# Position to hold.

		self.wp_x = self.Plant_location[11][0]
#wp_y: it is the waypoint's y coordinate with reference to camera  .
		self.wp_y = self.Plant_location[11][1]
#wp_z: it is the waypoint's z coordinate with reference to camera  .
		self.wp_z = self.Plant_location[11][2]
#wp_theta: it is the required yaw value of the drone   .
		self.wp_theta=0.000000
		# Correction values after PID is computed
#n:by using this variable we are controlling drone's holding time at a position , it's range is equivalent to range of a float value .
		self.n=0.000
#j: we are using this variable as a flag , once the drone reaches the final way point it gets updated to 1 and STOP is printed on the output screen  only once		
		self.j=0.0
		

                # Previous error
                	
               	
               	self.previous_error_roll=0.0
               	self.previous_error_pitch=0.0
               	self.previous_error_throt=0.0
		self.previous_error_yaw=0.0

                # Iterm       
                	
                self.iterm_roll=0.0
               	self.iterm_pitch=0.0
               	self.iterm_throt=0.0
		self.iterm_yaw=0.0


		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.032
		
################################################################################################


	#functions for drone control to move at desired point


#####################################for point 1 #############################################################
		



			
	def position_hold_1(self):
		
		self.initialise_p_1()
		rospy.sleep(2)
		print "inside position hold"
		print "disarm"
		self.disarm()
		print"START"
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)
		
		while True:
			
			self.calc_pid()
			
			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
	 		pitch_value = int(1500+float(self.correct_pitch))	#not sure of the -sign it should be + 				#forward rcPitch=1600
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 +float(self.correct_roll))			
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
															
			throt_value = int(1500-float(self.correct_throt))	#it's correct rcThrottle=1600 upward motion
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
				
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)		#rcYaw=1600 clockwise rotation vice versa
			
															
			self.pluto_cmd.publish(self.cmd)
			
			
##########################################################################
			#self.get_image()
		# calculate for how much time drone should hover over that point
			if (self.current_error_throt <=.8) and (self.current_error_throt >=-.8) :				
				if (self.current_error_yaw<=1) and (self.current_error_yaw >=-1) :
					if (self.current_error_pitch<=1) and (self.current_error_pitch>=-1) :
						if (self.current_error_roll<=1) and ( self.current_error_roll>=-1) :
							
							self.n=self.n+1
							print self.n
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
				
				
			
			if self.n==2:
				print"called position_hold 2"
				print self.drone_yaw_data
				print self.drone_x
				print self.drone_y
				print self.drone_z
				print "    "
				self.position_hold_2()
				
				

				
			



#####################################for point 2 ################################################################

		
			
	def position_hold_2(self):
		self.initialise_p_2()
		print "inside position hold 2"
		print self.drone_yaw_data
		while True:
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
	 		pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
				
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																
			self.pluto_cmd.publish(self.cmd)

		
##########################################################################
			#self.get_image(self)
		# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=1) and (self.current_error_throt>=-1) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=1) and (self.current_error_pitch>=-1) :
						if (self.current_error_roll<=1) and ( self.current_error_roll>=-1) :
							
							self.n=self.n+1
							print self.n
							
						else:
							
							self.n=0
							
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
				
			
			if self.n==1:
				print"called position_hold 3"
				print self.drone_yaw_data
				print self.drone_x
				print self.drone_y
				print self.drone_z
				print "    "
				
				self.position_hold_3()
				



		

#####################################for point 3 ####################################################################
		

	


		
	def position_hold_3(self):
		self.initialise_p_3()
		#print "inside position_hold_3"
		#print self.drone_yaw_data
		

		while True:
			
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
		 	pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
				
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																
			self.pluto_cmd.publish(self.cmd)
			
		
##########################################################################
			#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=1) and (self.current_error_throt>=-1) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=1) and (self.current_error_pitch>=-1) :
						if (self.current_error_roll<=1) and ( self.current_error_roll>=-1) :
							
							self.n=self.n+1
							#print self.n
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if self.n==1:
				#print"called position_hold 4"
				print self.drone_yaw_data
				print self.drone_x
				print self.drone_y
				print self.drone_z
				#print "    "
				
				self.position_hold_4()
				



		
#####################################for point 4  ################################################
			


	






	def position_hold_4(self):
		self.initialise_p_4()
		print "inside pos_hold_4"
	
		while True:
			
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
		 	pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
														
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
				
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																
			self.pluto_cmd.publish(self.cmd)

		
##########################################################################
			#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=.4) and (self.current_error_throt>=-.4) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=0) and (self.current_error_pitch>=-.7) :
						if (self.current_error_roll<=.7) and ( self.current_error_roll>=0) :
							
							self.n=self.n+1
							print self.n

							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if self.n==1 :
				print"called position_hold 5"
				print self.drone_yaw_data
				print self.drone_x
				print self.drone_y
				print self.drone_z
				print "    "
			
				#self.position_hold_5()

	


		
		
		
#####################################for point 5 #############################################################
		

			
		






	def position_hold_5(self):
		print "inside position_hold_5"
		self.initialise_p_5()

		while True:
			
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
	 		pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
				
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																
			self.pluto_cmd.publish(self.cmd)
			#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=1) and (self.current_error_throt>=-1) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=1) and (self.current_error_pitch>=-1) :
						if (self.current_error_roll<=1) and ( self.current_error_roll>=-1) :
							
							self.n=self.n+1
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if self.n==1 :
				print self.drone_yaw_data
				print self.drone_x
				print self.drone_y
				print self.drone_z
				print "called position_hold_6"
				self.position_hold_6()




		
###############################################################################################################



	def position_hold_6(self):
		print"inside position_hold_6"
		self.initialise_p_6()

		while True:
			
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
	 		pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
				
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																
			self.pluto_cmd.publish(self.cmd)
			#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=1) and (self.current_error_throt>=-1) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=1) and (self.current_error_pitch>=-1) :
						if (self.current_error_roll<=1) and ( self.current_error_roll>=1) :
							
							self.n=self.n+1
							print self.n
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if self.n==1 :
				print self.drone_yaw_data
				print "called position_hold_7"
				self.position_hold_7()




		
###############################################################################################################

	def position_hold_7(self):
		#print"inside position_hold_7"
		self.initialise_p_7()

		while True:
			
			self.calc_pid()
	
			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
	 		pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
					
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																	
			self.pluto_cmd.publish(self.cmd)
				#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=1) and (self.current_error_throt>=-1) :
				if (self.current_error_yaw<=2) and (self.current_error_yaw>=-2) :
					if (self.current_error_pitch<=1) and (self.current_error_pitch>=-1) :
						if (self.current_error_roll<=1) and ( self.current_error_roll>=-1) :
							
							self.n=self.n+1
							#print self.n
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if self.n==1 :
				#print self.drone_yaw_data
				#print"called position_hold_8"
				self.position_hold_8()




		
###############################################################################################################




	def position_hold_8(self):
		#print "inside position_hold_8"
		self.initialise_p_8()
	
		while True:
					
			self.calc_pid()
		
					# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
			pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
																	
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																		
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
						
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																		
			self.pluto_cmd.publish(self.cmd)
			#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=1) and (self.current_error_throt>=-1) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=1) and (self.current_error_pitch>=-1) :
						if (self.current_error_roll<=1) and ( self.current_error_roll>=-1) :
							
							self.n=self.n+1
							#print self.n
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if self.n==1 :
				#print self.drone_yaw_data
				#print"called position_hold_9"
				self.position_hold_9()
##########################################################################################################
###############################################################################################################





	def position_hold_9(self):
		#print"inside positin_hold_9"
		self.initialise_p_9()
	
		while True:
					
			self.calc_pid()
		
					# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
			pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
																	
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																		
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
						
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																		
			self.pluto_cmd.publish(self.cmd)
			#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=.8) and (self.current_error_throt>=-.8) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=0) and (self.current_error_pitch>=-.7) :
						if (self.current_error_roll<=.7) and ( self.current_error_roll>=0) :
							
							self.n=self.n+1
							#print self.n
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if self.n==1 :
				#print self.drone_yaw_data
				#print "calling position hold 10"
				self.disarm()	



##########################################################################################################
###############################################################################################################


	def position_hold_10(self):
		#print "inside position hold 10"
		self.initialise_p_10()
	
		while True:
					
			self.calc_pid()
		
					# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
			pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
																	
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																		
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
						
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																		
			self.pluto_cmd.publish(self.cmd)
			#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=.8) and (self.current_error_throt>=-.8) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=0) and (self.current_error_pitch>=-.7) :
						if (self.current_error_roll<=.7) and ( self.current_error_roll>=0) :
							
							self.n=self.n+1
							#print self.n
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if self.n==4 :
				print self.drone_yaw_data
				
				#print "calling position hold 11"
				#self.position_hold_11()	




##############################################################################################


	def position_hold_11(self):
		#print"inside positin_hold_11"
		self.initialise_p_11()
	
		while True:
					
			self.calc_pid()
		
					# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
			pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
																	
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																		
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
						
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																		
			self.pluto_cmd.publish(self.cmd)
			#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=.8) and (self.current_error_throt>=-.8) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=0) and (self.current_error_pitch>=-.7) :
						if (self.current_error_roll<=.7) and ( self.current_error_roll>=0) :
							
							self.n=self.n+1
							self.j=self.j+1
							#print self.n
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if (self.n==2) :
				#print self.drone_yaw_data
				self.position_hold_12()

	
######################################################################################################


	def position_hold_12(self):
		#print"inside positin_hold_12"
		self.initialise_p_12()
	
		while True:
					
			self.calc_pid()
		
					# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
			pitch_value = int(1500+float(self.correct_pitch))
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
																	
			roll_value = int(1500 +float(self.correct_roll))
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
																		
			throt_value = int(1500-float(self.correct_throt))
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
						
			yaw_value = int(1500 +float(self.correct_yaw))
			self.cmd.rcYaw = self.limit(yaw_value, 1600,1400)
																		
			self.pluto_cmd.publish(self.cmd)
			#self.get_image(self)
			# calculate for how much time drone should hover over that point
			if (self.current_error_throt<=.8) and (self.current_error_throt>=-.8) :
				if (self.current_error_yaw<=1) and (self.current_error_yaw>=-1) :
					if (self.current_error_pitch<=0) and (self.current_error_pitch>=-.7) :
						if (self.current_error_roll<=.7) and ( self.current_error_roll>=0) :
							
							self.n=self.n+1
							self.j=self.j+1
							#print self.n
							
						else:
							
							self.n=0
					else:
						
						self.n=0
				else:
					
					self.n=0
			else:
				
				self.n=0
			
			if (self.n==1) and (self.j==1) :
				#print self.drone_yaw_data
				print "STOP"
				


	
######################################################################################################
				

#############################################################################################################

	
	
		




if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = WayPoint()
		temp.position_hold_1()
		rospy.spin()
	

