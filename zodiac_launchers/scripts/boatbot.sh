#!/bin/bash
# This file must be `source` before any launch.
# The PIDs variable contains every PID of the process launched by this file.

pwd_path=$PWD

# ROS_IP
export ROS_IP=$(hostname -I)
# Check if there is one or several IP returned by hostname
test `echo $ROS_IP | wc -w` = 1
if [ ! $? -eq 0 ] ; then
	echo "IP address cannot be determined, there are several possibilities."
	echo "1) ${ROS_IP%\ *}\n2) ${ROS_IP#*\ }"
	echo "Enter 1 or 2 to choose the good one, or enter a valid IP address."
	read IP_nb
	if [ $IP_nb = 1 ] ; then
		export ROS_IP=${ROS_IP%\ *}
	elif [ $IP_nb = 2 ] ; then
		export ROS_IP=${ROS_IP#*\ }
	else
		export ROS_IP=$IP_nb
	fi
fi
echo "Your IP address is : "$ROS_IP


# Detection of whether the virtual ports are already opened
test `ls -l /dev | grep ttyVUSB | wc -l` = 0
if [ $? -eq 0 ] ; then
	cd
	cd ros_ws/src/ZodiacAuto/OpenCPN2ROS/script
	sudo ./create_ports.bash
fi

# Roslaunch
cd
source ros_ws/devel/setup.bash
roslaunch zodiac_launchers opencpn_interface.launch &

# Return to initial config
cd $pwd_path
unset pwd_path
unset IP_nb

fg
