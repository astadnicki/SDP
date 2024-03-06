from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

import serial

open_ser = 1
WHEEL_DIST = 0.4318
MAX = 0.301


class Subscriber(Node):
	def __init__(self):
		super().__init__('odom_convert')
		self.subscriber = self.create_subscription(Twist,'/cmd_vel_nav', self.callback,10)
		if open_ser == 1:
			self.ser = serial.Serial('/dev/ttyACM0')
		self.send = ""
		self.create_timer(0.1,self.timer_callback)
	
	def callback(self,msg):
		'''
		B = Left
		C = Right
		
		1. Recieve Twist message
		'''
		
		speed = 0 #50 - 99 for forward, 0-49 for reverse
		
		x_vel = msg.linear.x
		y_vel = msg.linear.y
		z_vel = msg.linear.z
		
		x_ang = msg.angular.x
		y_ang = msg.angular.y
		z_ang = msg.angular.z
		
		
		if(x_vel > MAX):
			print("x_vel greater than set MAX")
			x_vel = MAX
		rot_max = max(((z_ang*WHEEL_DIST)/2 + MAX),(MAX - (z_ang*WHEEL_DIST)/2))
		
		motor_r = (z_ang*WHEEL_DIST)/2 + x_vel
		motor_l = x_vel - (z_ang*WHEEL_DIST)/2
		
		
		print("Right Motor:",motor_r)
		print("Left Motor:",motor_l)
		
	
		if (motor_r == motor_l and abs(motor_r) >0):
			self.send = "AA "
			
			speed =  int(abs(motor_r)/rot_max*50)
			
			if speed > 50:
				speed = 50
			elif speed == 1:
				speed = 2
			if (motor_r > 0):
				speed+=49
			self.send+= str(speed)+" 00\r\n"
			#self.send += "99 00\r\n"
			#print("Send:",self.send)
				
		elif (z_ang != 0):
			if (z_ang > 0):
				self.send = "BB "
				r_per = int(abs(motor_r)/rot_max*50)+49
				l_per = int(abs(motor_l)/rot_max*50)
				
				
				if (r_per > 99):
					print("r_per greater than 99",r_per)
					r_per = 99
				if (l_per > 49):
					print("l_per greater than 99",l_per)
					l_per = 49
				if (motor_l > 0):
					l_per+=49
			else:
				self.send = "CC "
				r_per = int(abs(motor_r)/rot_max*50)
				l_per = int(abs(motor_l)/rot_max*50)+49
				
				
				if (r_per > 49):
					print("r_per greater than 99",r_per)
					r_per = 49
				if (l_per > 99):
					print("l_per greater than 99",l_per)
					l_per = 99
				
				if (motor_r > 0):
					r_per +=49
		
			self.send += str(l_per)+" "+str(r_per)+"\r\n"
			#print("Send:",self.send)
		else:
			self.send = "DD 00 00\r\n"
			
	
	def timer_callback(self):
		if self.send !="":
			if open_ser == 1:
				self.ser.write(self.send.encode("utf-8"))
			print(self.send) 
		
		
def main(args=None):
	rclpy.init(args=args)
	sub = Subscriber()
	rclpy.spin(sub)

if __name__ == '__main__':
	main()
	

#Linear x + (angular z * Wheel Separation (Robot width)/2) 		
		
