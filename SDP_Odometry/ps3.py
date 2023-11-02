'''
Grabs joystick data from USB controllor and sends commands to odom
'''


#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import Joy
from std_msgs.msg import String

def callback(data):
    left_updown = data.axes[1]
    left_rightleft = data.axes[0]
    cmd=''

    if left_rightleft>0.15:
        cmd+='2 '+str(3.14*left_rightleft)
    elif left_rightleft<-0.15:
        cmd+='1 '+str(3.14*abs(left_rightleft))
    
    if abs(left_updown)>0.15:
        if cmd =='':
            cmd ='0 0.0 ' + str(left_updown)
        else:
            cmd += str(left_updown)
    
    pub.publish(cmd)
    
def Subscriber():
    rospy.Subscriber('/joy', Joy, callback)

def talker():
    pub = rospy.Publisher('odom-cmd', String, queue_size=10)
    rospy.init_node('ps3-cont', anonymous=True)
    #rate = rospy.Rate(1) # 1hz
    #i=0

    while not rospy.is_shutdown():
        Subscriber()
        rospy.spin()
