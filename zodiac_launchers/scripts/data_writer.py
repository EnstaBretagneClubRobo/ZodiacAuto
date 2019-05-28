#coding: utf-8
#from scipy import signal
import numpy as np
import sys, os

import rospy
import rosbag

# from gmplot import gmplot
#import yaml

EARTH_RADIUS = 6371000.
lat0 = 48.281331
long0 = -4.630436


def GPS2RefCoordSystem(lat, lon):
	return (np.pi/180.)*EARTH_RADIUS*(lon-long0)*np.cos((np.pi/180.)*lat), (np.pi/180.)*EARTH_RADIUS*(lat-lat0)

if len(sys.argv) < 2:
	print("Please enter at least one rosbag as argument")
	exit()
else:
	bags_names = []
	for b_name in sys.argv[1:]:
		if b_name.split('.')[1] == "bag":
			bags_names.append(b_name)

sep = " ; "

for bag_nb in range(len(bags_names)):
	print str(bag_nb+1) + "/" + str(len(bags_names)) + " : " + bags_names[bag_nb]
	bag = rosbag.Bag(bags_names[bag_nb], 'r')
	startTime = rospy.Time.from_sec(bag.get_start_time())
	endTime = rospy.Time.from_sec(bag.get_end_time())

	desiredCourse = 0
	signedDistance = 0
	latitude = 0
	longitude = 0
	x, y = 0, 0
	boat_heading = 0
	theta = 0
	helm_angle = 0
	lat, lon = 0, 0
	aLat = 0
	aLon = 0
	bLat = 0
	bLon = 0
	ax, ay = 0, 0
	bx, by = 0, 0
	motorOn = False
	timeStamp = 0
	timeRef = 0

	with open(bags_names[bag_nb][:-bags_names[bag_nb][::-1].find(os.sep)] + "data_" + bags_names[bag_nb].split(os.sep)[-1] + ".txt", 'w') as f:
		f.write("counter ; t (in s) ; lat0 (in decimal degrees) ; long0 (in decimal degrees) ; roll (in rad) ; pitch (in rad) ; yaw (in rad) ; winddir (in rad) ; windspeed (in m/s) ; filteredwinddir (in rad) ; filteredwindspeed (in m/s) ; heading (in rad) ; theta (in rad) ; psi (in rad) ; latitude (in decimal degrees) ; longitude (in decimal degrees) ; x (in m) ; y (in m) ; ax (in m) ; ay (in m) ; bx (in m) ; by (in m) ; CurWP ; wpslat[CurWP] (in decimal degrees) ; wpslong[CurWP] (in decimal degrees) ; e (in m) ; norm_ma (in m) ; norm_bm (in m) ; state ; deltag (in rad) ; deltavmax (in rad) ; phi+gammabar (in rad) ; gps time (in s) ;\n")

		n = 1
		for topic, msg, t in bag.read_messages(topics=["/zodiac_auto/desired_course", "/zodiac_auto/boat_heading", "/zodiac_auto/signedDistance", "/zodiac_auto/fix", "/zodiac_auto/waypoint_line", "/zodiac_auto/helm_angle_cmd", "/zodiac_auto/helm_motorOn", "/zodiac_auto/time_reference"], start_time=startTime):

			time = t.to_sec()-startTime.to_sec()
			if topic=="/zodiac_auto/desired_course":
				desiredCourse = (-msg.data+90)*np.pi/180.
			if topic=="/zodiac_auto/signedDistance":
				signedDistance = msg.data
			if topic=="/zodiac_auto/fix":
				lat = msg.latitude
				lon = msg.longitude
				x, y = GPS2RefCoordSystem(lat, lon)
			if topic=="/zodiac_auto/boat_heading":
				boat_heading = msg.data
				theta = (-boat_heading+90)*np.pi/180.
			if topic=="/zodiac_auto/helm_angle_cmd":
				helm_angle = -msg.data*np.pi/180.
			if topic=="/zodiac_auto/waypoint_line" and len(msg.waypoints)==2:
				aLat = msg.waypoints[0].latitude
				bLat = msg.waypoints[1].latitude
				aLon = msg.waypoints[0].longitude
				bLon = msg.waypoints[1].longitude
				ax, ay = GPS2RefCoordSystem(aLat, aLon)
				bx, by = GPS2RefCoordSystem(bLat, bLon)
			if topic=="/zodiac_auto/helm_motorOn":
				motorOn = msg.data
			if topic=="/zodiac_auto/time_reference":
				timeStamp = msg.header.stamp.secs
				timeRef = msg.time_ref.secs

			if time >= n:
				f.write(str(n) + sep + str(timeStamp) + sep + str(lat0) + sep + str(long0) + sep + "0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0" + sep + str(theta) + sep + "0" + sep + str(lat) + sep + str(lon) + sep + str(x) + sep + str(y) + sep + str(ax) + sep + str(ay) + sep + str(bx) + sep + str(by) + sep + "0 ; 0 ; 0" + sep + str(signedDistance) + sep + "0 ; 0" + sep + str(int(not motorOn)) + sep + str(helm_angle) + sep + "0" + sep + str(desiredCourse) + sep + str(timeRef) + " ;\n")
				n += 1

	bag.close()

