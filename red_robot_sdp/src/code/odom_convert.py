from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import rclpy
from rclpy.node import Node

import serial

class Subscriber(node):
	def __init__(self):
		super().__init__('odom_convert')
		self.subscriber = self.create_subscription(Path,'/plan', self.callback,10)
		self.path_buffer = [] #Stores incoming waypoint positions utill 2 are present
		self.send_buffer = [] #Stores commands untill they are sent
		
	def callback(self,msg):
		'''
		1. When msg is recieved add to path_buffer
		2. Compare with current pose
		3. Add rotatation to send_buffer if required
		4. Add forward/reverse commands as needed
		
		A = Forward/Reverse B = Left C = Right
		<50 = Reverse >50 = Forward
		00-99 = 0-180 degrees
		'''
		path_buffer.append(msg.pose)
		
		
			
