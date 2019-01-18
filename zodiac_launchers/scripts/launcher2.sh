#!/bin/bash
# This file must be `source` before any launch.
# You can enter the password of the NTRIP server (78.24.131.136) as first argument in order to have RTK corrections.
# All logs are saved in the folder "recorded_data".

export now=`date +"%F-%T"`
pwd_path=$PWD

# Detection of whether the RTK corrections are already launched
test `pgrep str2str | wc -l` = 0
if [ ! $? -eq 0 ] ; then
	echo "RTK corrections are still sending to /dev/gps"
	launch_rtk=false
# NTRIP password
elif [ -z $1 ]; then
	echo "Please enter the password as first argument in order to have RTK corrections from the NTRIP server."
	launch_rtk=false
else
	echo "RTK corrections will be sent to /dev/gps"
	launch_rtk=true
fi

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
echo "Your IP address is : "$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:11311

# Detection of whether the virtual ports are already opened
test `ls -l /dev | grep ttyVUSB | wc -l` = 0
if [ $? -eq 0 ] ; then
    echo "Opening ports ttyVUSB0, 1, 2, 3"
    sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB0,mode=666 PTY,raw,echo=0,link=/dev/ttyVUSB1,mode=666 &
    sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB2,mode=666 PTY,raw,echo=0,link=/dev/ttyVUSB3,mode=666 &
    sleep 2
    sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB1,mode=666 tcp-listen:5001,reuseaddr,fork &
    sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVUSB2,mode=666 tcp-listen:5011,reuseaddr,fork &
    sleep 2
    echo "Port ready"
fi

# cd ${0%*launcher.sh}/../../../..
cd ~/ros_ws
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

# RTK Corrections
# The password must be entered as first argument of this script
if [ "$launch_rtk" = true ] ; then
    nohup ~/str2str -in "ntrip://ENSTABRE:$1@78.24.131.136:2101/MAC30" -out tcpsvr://:21200 -n 1000 -p 48.418 -4.472 150.0 > $save_path/str2strntrip.out &
    nohup ~/str2str -in "tcpcli://localhost:21200" -out serial://gps > $save_path/str2strgpsboatbot.out &
fi 

sleep 1

# Roslaunch
roslaunch zodiac_launchers zodiac.launch &

# Return to initial config
cd $pwd_path
unset pwd_path
unset launch_rtk
unset save_path
# unset IP_nb

fg
