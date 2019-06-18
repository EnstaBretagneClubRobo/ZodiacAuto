#coding: utf-8

import os
from math import cos, sin, pi

# Brique parameter
Length = 1000.
Height = 100.
OVERSHOOT = 100. #Dépassement (en m)
LINE_SPACE = 10. #Ecart entre chaque ligne (en m)

EARTH_RADIUS = 6371000.
lat0, lon0 = (48.332177854, -4.662854960)

def WGS84_to_cart(lat, lon):
	x = (pi/180.)*EARTH_RADIUS*(lon-lon0)*cos((pi/180.)*lat)
	y = (pi/180.)*EARTH_RADIUS*(lat-lat0)
	return x, y

def cart_to_WGS84(x, y):
	EPSILON=0.00000000001
	lat = y*180./pi/EARTH_RADIUS+lat0
	if abs(lat-90.) < EPSILON or abs(lat+90.) < EPSILON:
		lon = 0
	else:
		lon = (x/EARTH_RADIUS)*(180./pi)/cos((pi/180.)*(lat))+lon0
	return lat, lon

start_point_bloc_1 = (0., 0.)
bloc_number_to_coord_col_1 = [(start_point_bloc_1[0], start_point_bloc_1[1] + i*Height) for i in range(13)]
bloc_number_to_coord_col_2 = [(start_point_bloc_1[0] + Length, start_point_bloc_1[1] + i*Height) for i in range(10)]
bloc_number_to_coord_col_3 = [(start_point_bloc_1[0] + 2*Length, start_point_bloc_1[1] + i*Height) for i in range(5)]
bloc_number_to_coord_col_4 = [(start_point_bloc_1[0] + 3*Length, start_point_bloc_1[1] + i*Height) for i in range(5)]
bloc_number_to_coord_col_5 = [(start_point_bloc_1[0] + 4*Length, start_point_bloc_1[1] + i*Height) for i in range(7)]
bloc_number_to_coord_col_6 = [(start_point_bloc_1[0] + 5*Length, start_point_bloc_1[1] + i*Height) for i in range(12)]

bloc_number_to_coord = bloc_number_to_coord_col_1 + bloc_number_to_coord_col_2 + bloc_number_to_coord_col_3 + bloc_number_to_coord_col_4 + bloc_number_to_coord_col_5 + bloc_number_to_coord_col_6

def genere_one_bloc(start_point, overshoot=OVERSHOOT, line_space=LINE_SPACE):
	WP_L93 = [start_point, (start_point[0]+(Length+overshoot), start_point[1])]
	for n in range(1, int(Height/line_space)+1):
		if n%2 == 0: #From west to east
			WP_L93.append((start_point[0]-overshoot, start_point[1]+n*line_space))
			WP_L93.append((start_point[0]+Length+overshoot, start_point[1]+n*line_space))
		else: #From east to west
			WP_L93.append((start_point[0]+Length+overshoot, start_point[1]+n*line_space))
			WP_L93.append((start_point[0]-overshoot, start_point[1]+n*line_space))
	WP_WGS84 = []
	for WP in WP_L93:
		WP_WGS84.append(cart_to_WGS84(*WP))
	return WP_WGS84

# Creating "briques" folder
if not os.path.exists("briques"):
    os.makedirs("briques")

for n_bloc in range(1, 53):
	with open("briques/brique_" + str(n_bloc) + ".gpx", 'w') as f:
		with open("template.gpx", 'r') as template_f:
			for l in template_f:
				if l.find("$NAME") >= 0:
					f.write(l.replace("$NAME", "brique_" + str(n_bloc)))
				elif l.find("$WP") >= 0:
					WP = genere_one_bloc(bloc_number_to_coord[n_bloc-1])
					for i in range(len(WP)):
						with open("WP.gpx", 'r') as wp_f:
							for l_wp in wp_f:
								if l_wp.find("$LAT") >= 0:
									line_with_lat = l_wp.replace("$LAT", str(WP[i][0]))
									f.write(line_with_lat.replace("$LON", str(WP[i][1])))
								elif l_wp.find("$NAME") >= 0:
									f.write(l_wp.replace("$NAME", str(i+1)))
								else:
									f.write(l_wp)
				else:
					f.write(l)

