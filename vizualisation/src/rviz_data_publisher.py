#!/usr/bin/env python

import rospy, tf
import numpy as np
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from zodiac_command.msg import WaypointListMission
from nav_msgs.msg import Path

isAuto = False
pos = PoseStamped()
pos.header.frame_id = "map"
dc = PoseStamped()
dc.header.frame_id = "map"
line = Path()
line.header.frame_id = "map"
trace = Path()
trace.header.frame_id = "map"

startingPos = [0,0,0]
startingPos_isSet = False

def auto_fb(msg):
    global isAuto
    isAuto = msg.data

def bh_fb(msg):
    global pos
    q = tf.transformations.quaternion_from_euler(0, 0, ((-msg.data+90)%360)*np.pi/180.)
    pos.pose.orientation.x = q[0]
    pos.pose.orientation.y = q[1]
    pos.pose.orientation.z = q[2]
    pos.pose.orientation.w = q[3]

def dc_fb(msg):
    global dc
    q = tf.transformations.quaternion_from_euler(0, 0, ((-msg.data+90)%360)*np.pi/180.)
    dc.pose.orientation.x = q[0]
    dc.pose.orientation.y = q[1]
    dc.pose.orientation.z = q[2]
    dc.pose.orientation.w = q[3]

def fix_fb(msg):
    global startingPos_isSet, startingPos, pos, dc, trace
    if not startingPos_isSet:
        startingPos = [msg.longitude, msg.latitude, msg.altitude]
        startingPos_isSet = True
    pos.pose.position.x = (msg.longitude - startingPos[0]) * 10000
    pos.pose.position.y = (msg.latitude - startingPos[1]) * 10000
    pos.pose.position.z = (msg.altitude - startingPos[2]) * 1
    pos.header.stamp = dc.header.stamp = rospy.Time.now()
    dc.pose.position = pos.pose.position
    pos_save = PoseStamped()
    pos_save.header = pos.header
    pos_save.pose.position.x = pos.pose.position.x
    pos_save.pose.position.y = pos.pose.position.y
    pos_save.pose.position.z = pos.pose.position.z
    trace.poses.append(pos_save)

def wp_fb(msg):
    global line
    if len(msg.waypoints) > 0:
        pos_msg_1 = PoseStamped()
        pos_msg_1.header.frame_id = "map"
        pos_msg_1.header.stamp = rospy.Time.now()
        pos_msg_1.pose.position.x = (msg.waypoints[0].longitude - startingPos[0]) * 10000
        pos_msg_1.pose.position.y = (msg.waypoints[0].latitude - startingPos[1]) * 10000
        pos_msg_2 = PoseStamped()
        pos_msg_2.header.frame_id = "map"
        pos_msg_2.header.stamp = rospy.Time.now()
        pos_msg_2.pose.position.x = (msg.waypoints[1].longitude - startingPos[0]) * 10000
        pos_msg_2.pose.position.y = (msg.waypoints[1].latitude - startingPos[1]) * 10000
        line.poses = [pos_msg_1, pos_msg_2]
        line.header.stamp = rospy.Time.now()
        line_pub.publish(line)

rospy.init_node('rviz_data_publisher', anonymous=True)

auto_sub = rospy.Subscriber('/zodiac_auto/helm_motorOn', Bool, auto_fb)
bh_sub = rospy.Subscriber('/zodiac_auto/boat_heading', Float64, bh_fb)
dc_sub = rospy.Subscriber('/zodiac_auto/desired_course', Float64, dc_fb)
fix_sub = rospy.Subscriber('/zodiac_auto/fix', NavSatFix, fix_fb)
wp_sub = rospy.Subscriber('/zodiac_auto/waypoint_line', WaypointListMission, wp_fb)

pos_pub = rospy.Publisher('boat_pose', PoseStamped, queue_size=10)
dc_pub = rospy.Publisher('desired_course', PoseStamped, queue_size=10)
line_pub = rospy.Publisher('line', Path, queue_size=10)
trace_pub = rospy.Publisher('trace', Path, queue_size=10)

rate = rospy.Rate(3)
while not rospy.is_shutdown():
    if isAuto:
        trace_pub.publish(trace)
    else:
        trace.poses = []

    trace.header.stamp = rospy.Time.now()
    dc_pub.publish(dc)
    pos_pub.publish(pos)

    rate.sleep()
