#!/usr/bin/env python
import rospy
import numpy as np
import math
from turtlebot3_auto_msgs.msg import  Twist2DStamped, LanePose, ImgSignals
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from time import sleep

class motor_controller(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.cmd_vel_line_reading = None
		self.cmd_vel_parking_reading = None
		self.cmd_vel_tunnel_reading = None
		self.signal_reading = None
		self.odom_reading = None

		self.con_state ="LINE" # LINE, BAR, PARKING, TUNNEL, LED,
		#self.con_state = "TUNNEL"  # LINE, BAR, PARKING, TUNNEL, LED,
		#self.prev_con_state = "LINE"
		self.motor_stop_state = False
		self.led_state = None
		self.bar_state = None
		self.pub_counter = 0
		self.tunneltimetrigger = False

		# Setup parameters
		#self.setGains()

		# Publicaiton
		self.pub_car_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.pub_con_state = rospy.Publisher('/con_state', String,queue_size=1)

		# Subscriptions
		# self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
		#self.sub_odom_reading = rospy.Subscriber("/odom", Odometry, self.cbOdometry, queue_size=1)
		self.sub_cmd_vel_line_reading = rospy.Subscriber("/cmd_vel_line", Twist, self.cbCmdVelLine, queue_size=1)
		self.sub_cmd_vel_parking_reading = rospy.Subscriber("/cmd_vel_parking", Twist, self.cbCmdVelParking, queue_size=1)
		self.sub_cmd_vel_tunnel_reading = rospy.Subscriber("/cmd_vel_tunnel", Twist, self.cbCmdVelTunnel, queue_size=1)
		self.sub_signal_reading = rospy.Subscriber("/signals", ImgSignals, self.cbSignal, queue_size=1)
		self.sub_state_change = rospy.Subscriber("/con_state_change", String, self.cbConStateChange, queue_size=1)
		self.sub_line_state_reading = rospy.Subscriber("/line_state", String, self.cbLineState, queue_size=1)

		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)

		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))


	def vehicleStop(self):
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
		self.publishCmd(twist)

	def custom_shutdown(self):
		# Stop listening
		self.sub_state_change.unregister()
		self.sub_signal_reading.unregister()
		self.sub_line_state_reading.unregister()
		self.sub_cmd_vel_line_reading.unregister()
		self.sub_cmd_vel_parking_reading.unregister()
		self.sub_cmd_vel_tunnel_reading.unregister()

		# Send stop command
		self.vehicleStop()
		# Send stop timmer
		#self.opencr_timer.shutdown()
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def publishCmd(self,twist):
		if self.motor_stop_state == True :
			twist.linear.x = 0 ; twist.linear.y = 0 ; twist.linear.z = 0 ;
			twist.angular.x = 0 ; twist.angular.y = 0 ; twist.angular.z = 0 ;
		#print( "final test  test cbCmdVelParking  %d, %d " , twist.linear.x, twist.angular.z)
		self.pub_car_cmd.publish(twist)

	def cbCmdVelLine(self,cmd_vel_line_msg):
		twist = Twist()
		twist.linear.x = cmd_vel_line_msg.linear.x
		twist.angular.z = cmd_vel_line_msg.angular.z
		if self.con_state == "LINE" :
			print( "LINECmdVel   %d, %d  " , twist.linear.x, twist.angular.z)
			self.publishCmd(twist)

	def cbCmdVelParking(self,cmd_vel_parking_msg):
		twist = Twist()
		twist.linear.x = cmd_vel_parking_msg.linear.x
		twist.angular.z = cmd_vel_parking_msg.angular.z
		if self.con_state == "PARKING" :
			#print( "ParkingCmdVel   %d, %d  " , twist.linear.x, twist.angular.z)
			self.publishCmd(twist)

	def cbCmdVelTunnel(self,cmd_vel_tunnel_msg):
		twist = Twist()
		twist.linear.x = cmd_vel_tunnel_msg.x
		twist.angular.z = cmd_vel_tunnel_msg.z
		if self.con_state == "TUNNEL" :
			self.publishCmd(twist)
	'''
	def cbOdometry(self,odom_msg):
		self.odom_reading = odom_msg
		#rospy.loginfo('x: %f , y:%f , w:%f ',self.odom_reading.pose.pose.position.x ,  self.odom_reading.pose.pose.position.y, self.odom_reading.pose.pose.orientation.w )
	'''
	def cbLineState(self, line_state_msg ):
		return
		if line_state_msg.data == "DOT_LINE" :
			sleep(2)
			self.con_state = "PARKING"

		if line_state_msg.data == "TUNNEL_LINE"  :
			self.con_state = "TUNNEL"
		self.pub_con_state.publish(self.con_state)

	def cbSignal(self,signal_msg):
		self.signal_reading = signal_msg.data
		return
		rospy.loginfo('Signal: %s , LED : %s ',self.signal_reading.SIGNAL , self.signal_reading.LED )
		if self.signal_reading.SIGNAL == "BAR" :
			self.con_state = "BAR"
			self.bar_state = self.signal_reading.BAR #VER, HOR
		elif self.signal_reading.SIGNAL == "PARKING" :
			self.con_state = "PARKING"
		elif self.signal_reading.SIGNAL == "TUNNEL" :
			self.con_state = "TUNNEL"
		elif self.signal_reading.SIGNAL == "LED" :
			self.con_state = "LED"
			self.led_state = self.signal_reading.LED #RED,YELLOW,GREEN
		self.pub_con_state.publish(self.con_state)
		#sleep(0.1)

		if self.con_state == "LED":
			self.controlByLed(self.led_state)
		elif self.con_state == "BAR":
			self.controlByLed(self.bar_state)

	def cbConStateChange(self,con_state_change):
		if con_state_change.data == "TUNNEL_OUT" :
			self.con_state = "LINE"
		elif con_state_change.data == "PARKING_OUT" :
			self.con_state = "LINE"
		elif con_state_change.data == "PARKING2" :
			self.con_state = "PARKING"
		self.pub_con_state.publish(self.con_state)
	''''
	def cbMotorStop(self,MotorStop):
		if MotorStop :
			self.motor_stop_state = True
		else :
			self.motor_stop_state = False
		sleep(0.5)
		self.vehicleStop()
	'''
	def controlByLed(self,led_state):
		if led_state == "LED" or led_state == "YELLOW" :
			self.vehicleStop()
		elif led_state == "GREEN" :
			self.con_state = "LINE"
		self.pub_con_state.publish(self.con_state)

	def controlByBar(self,bar_state):
		if bar_state == "HOR" :
			self.vehicleStop()
		elif bar_state == "VER" :
			self.con_state = "LINE"
		self.pub_con_state.publish(self.con_state)

if __name__ == "__main__":
	rospy.init_node("motor_controller",anonymous=False)
	motor_control_node = motor_controller()
rospy.spin()
