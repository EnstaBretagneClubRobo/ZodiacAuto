#!/bin/bash

# This file must be `source` before any launch, tweak also ROS_IP and ROS_WS if needed...
# In order to have RTK corrections, enter the password of the NTRIP server (78.24.131.136) as first argument 
# or set RTK_RTCM_SERVER_IP to the IP address of a RTCM server.
# All logs are saved in the folder "recorded_data".

#export now=`date +"%F-%T"`
export now=`date +"%F_%Hh%Mmin%Ss"`
pwd_path=$PWD

# ROS_IP

# export ROS_IP=$(hostname -I)
# # Check if there is one or several IP returned by hostname
# test `echo $ROS_IP | wc -w` = 1
# if [ ! $? -eq 0 ] ; then
# 	echo "IP address cannot be determined, there are several possibilities."
# 	echo -e "1) ${ROS_IP%\ *\ *}\n2) ${ROS_IP#*\ }"
# 	echo "Enter 1 or 2 to choose the good one, or enter a valid IP address."
# 	read IP_nb
# 	if [ $IP_nb = 1 ] ; then
# 		export ROS_IP=${ROS_IP%\ *\ *}
# 	elif [ $IP_nb = 2 ] ; then
# 		export ROS_IP=${ROS_IP#*\ }
# 	else
# 		export ROS_IP=$IP_nb
# 	fi
# fi
export ROS_IP=10.42.0.1
#export ROS_IP=127.0.0.1
echo "Your IP address is : "$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROS_WS=ros_ws
#export RTK_RTCM_SERVER_IP=127.0.0.1

# Detection of whether the real devices are available (assume that /dev/imu is only available on the real robot)
test `ls -l /dev | grep imu | wc -l` = 0
if [ $? -eq 0 ] ; then
	echo "Simulator mode"
	simulator=true
else
	echo "Real mode"
	simulator=false
fi

# Detection of whether the RTK corrections are already launched
#test `pgrep str2str | wc -l` = 0
#if [ ! $? -eq 0 ] ; then
#	echo "RTK corrections are still sending to /dev/gps"
#	launch_ntrip=false
#fi

# NTRIP password
if [ -z $1 ]; then
	echo "Please enter the password as first argument in order to have RTK corrections from the NTRIP server."
	launch_ntrip=false
else
	echo "RTK corrections will be sent from the NTRIP server to /dev/gps"
	launch_ntrip=true
fi

# Detection of whether the virtual ports are already opened
test `ls -l /dev | grep ttyVUSB | wc -l` = 0
if [ $? -eq 0 ] ; then
    echo "OpenCPN link"
    while true; do sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB1,mode=666 PTY,raw,echo=0,link=/dev/ttyVUSB2,mode=666; sleep 5; done &
    sleep 1
    while true; do sudo python ~/mux_server.py -d /dev/ttyVUSB2 -b 4800 -p 5001 -c 1; sleep 5; done &
fi

# cd ${0%*launcher.sh}/../../../..
cd ~/$ROS_WS
. ./devel/setup.${SHELL##*/}

# Rosbag
if [ ! -d ./recorded_data ] ; then
    mkdir recorded_data
fi
mkdir recorded_data/$now
save_path=$PWD/recorded_data/$now

roscd zodiac_launchers
cp ./config/*.yaml $save_path
cp -r `readlink ~/.ros/log/latest` $save_path

if [ "$simulator" = true ] ; then
	while true; do bash -c "cd ~/UxVSim && exec ~/UxVSim/UxVSim"; sleep 5; done &
	sleep 2
	while true; do sudo socat -d -d PTY,raw,echo=0,link=/dev/imu,mode=666 tcp:127.0.0.1:4007; sleep 5; done &
	while true; do sudo socat -d -d PTY,raw,echo=0,link=/dev/gps,mode=666 tcp:127.0.0.1:4001; sleep 5; done &
	while true; do sudo socat -d -d PTY,raw,echo=0,link=/dev/pololu,mode=666 tcp:127.0.0.1:4004; sleep 5; done &
	#while true; do sudo socat -d -d PTY,raw,echo=0,link=/dev/sonar,mode=666 tcp:127.0.0.1:4008; sleep 5; done &
	sleep 1
fi

echo "GPS"
if [ "$simulator" = true ] ; then
	while true; do sudo python ~/mux_server.py -d /dev/gps -b 9600 -p 6001 -c 1; sleep 5; done &
else
	while true; do sudo python ~/mux_server.py -d /dev/gps -b 9600 -p 6001; sleep 5; done &
fi
sleep 1
while true; do sudo socat -d -d PTY,raw,echo=0,link=/dev/gpsv,mode=666 tcp:127.0.0.1:6001; sleep 5; done &
# RTK Corrections
if [ "$launch_ntrip" = true ] ; then
    echo "NTRIP RTK server"
	# The password must be entered as first argument of this script
    while true; do nohup ~/str2str -in "ntrip://ENSTABRE:$1@78.24.131.136:2101/MAC30" -out tcpcli://127.0.0.1:6001 -n 1000 -p 48.418 -4.472 150.0 > $save_path/str2str.out; sleep 5; done &
elif [ ! -z $RTK_RTCM_SERVER_IP ]; then
    echo "RTCM RTK server"
    while true; do nohup ~/str2str -in tcpcli://$RTK_RTCM_SERVER_IP:4001 -out tcpcli://127.0.0.1:6001 > $save_path/str2str.out; sleep 5; done &
fi
sleep 1

roslaunch zodiac_launchers zodiac.launch &

# Return to initial config
cd $pwd_path
unset save_path
unset launch_ntrip
unset simulator
unset RTK_RTCM_SERVER_IP
unset ROS_WS
# unset IP_nb
unset pwd_path

fg
