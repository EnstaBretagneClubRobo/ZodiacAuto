#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Float64

rospy.init_node("depth_publisher")
detph_pub = rospy.Publisher("depth", Float64, queue_size=1)

ser = serial.Serial(port="/dev/sonar", baudrate=9600) #, parity=serial.PARITY_ODD, stopbits=serial.STOPBITS_TWO, bytesize=serial.SEVENBITS)

try:
    ser.open()
except Exception, e:
    ROS_WARN("Error open sonar serial port : " + str(e))
    exit()

rate = rospy.Rate(5)
if ser.isOpen():
    while not rospy.is_shutdown():
        line = ser.readline()
        depth_msg = Float64()
        depth_msg.data = float(line[6:9]) #TODO To be changed
        depth_pub.publish(depth_msg)
        rate.sleep()
ser.close()
