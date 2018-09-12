#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, TwistStamped


gamma = 1000*np.eye(3)
gamma_a = 0.00001
gamma_b = 0.2

state = Vector3()

def kalman_predict(xup, Gup, u, gamma_a, A):
	gamma_1 = A.dot(Gup).dot(A.T) + gamma_a
	x1 = A.dot(xup) + u
	return x1, gamma_1

def kalman_correct(x0, gamma_0, y, gamma_b, C):
	S = C.dot(gamma_0).dot(C.T) + gamma_b
	K = gamma_0.dot(C.T).dot(np.linalg.inv(S))
	ytilde = y - C.dot(x0)
	Gup = (np.eye(len(x0)) - K.dot(C)).dot(gamma_0)
	xup = x0 + K.dot(ytilde)
	return xup, Gup

def kalman(x0, gamma_0, u, y, gamma_a, gamma_b, A, C):
	xup, Gup = kalman_correct(x0, gamma_0, y, gamma_b, C)
	x1, gamma1 = kalman_predict(xup, Gup, u, gamma_a, A)
	return x1, gamma1


def vel_cb(msg):
	global state
	state.x = msg.twist.linear.x
	state.y = msg.twist.linear.y
	C = np.array([[np.cos(state.z), 1, 0], [np.sin(state.z), 0, 1]])
	y = np.array([[state.x], [state.y]])
	phat, gamma = kalman(phat, gamma, 0, y, gamma_a*np.eye(3), gamma_b*np.eye(2), np.eye(3), C)
	currents = Vector3()
	currents.x, currents.y, currents.z = phat.flatten()
def heading_cb(msg):
	global state
	state.z = msg.data

rospy.init_node("current_estimation")

vel_gps_sub = rospy.Subscriber('/zodiac_auto/vel', TwistStamped, vel_cb)
boat_heading_sub = rospy.Subscriber('/zodiac_auto/boat_heading', Float64, heading_cb)

currents_pub = rospy.Publisher('currents', Vector3, queue_size=10)

rospy.spin()
