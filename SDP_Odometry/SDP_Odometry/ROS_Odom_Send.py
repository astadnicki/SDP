'''
Odometry 2022-2023
John Boesen

Grabs data from a ROS node and sends it through the USB 
connection to the arduino.

Routine:
-Listen to odom-cmd node 
-Send data through to Arduino
'''

#!/usr/bin/env python
import rospy
from std_msgs.msg import String 
import usb.core
import usb.util 

def callback(data,ep): # Write data to usb
    ep.write(data)

def listener(): 
    rospy.init_node('odom-listener', anonymous=True)  # INIT listener node
    dev = usb.core.find(idVendor=0xfffe, idProduct=0x0001) #INIT usb
    if dev is None:
        raise ValueError('Device not found')
    cfg = dev.get_active_configuration()
    intf = cfg[(0,0)]
    ep = usb.util.find_descriptor(
        intf,
        # match the first OUT endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT)


    rospy.Subscriber('odom-cmd', String, callback,ep) 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin() 
if __name__ == '__main__':
    listener()

