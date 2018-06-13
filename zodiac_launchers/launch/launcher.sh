# This file must be `source` before any launch.
# The PIDs variable contains every PID of the process launched by this file.

export now=`date +"%F-%T"`
pwd_path=$PWD

# NTRIP password
use_rtk=true
if [ -z $1 ]; then
	echo "Please enter the password as first argument in order to have RTK corrections from the NTRIP server."
	use_rtk=false
else
	echo "RTK corrections will be sent to /dev/gps"
fi

# ROS_IP
export ROS_IP=$(hostname -I)
# Check if there is one or several IP returned by hostname
test `echo $ROS_IP | wc -w` = 1
if [ ! $? -eq 0 ] ; then
	echo "IP address cannot be determined, there are several possibilities."
	echo -e "1) ${ROS_IP%\ *\ *}\n2) ${ROS_IP#*\ }"
	echo "Enter 1 or 2 to choose the good one, or enter a valid IP address."
	read IP_nb
	if [ $IP_nb = 1 ] ; then
		export ROS_IP=${ROS_IP%\ *\ *}
	elif [ $IP_nb = 2 ] ; then
		export ROS_IP=${ROS_IP#*\ }
	else
		export ROS_IP=$IP_nb
	fi
fi
echo "Your IP address is : "$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:11311

# cd ${0%*launcher.sh}/../../../..
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
if [ "$use_rtk" = true ] ; then
    nohup ~/str2str -in "ntrip://ENSTABRE:$1@78.24.131.136:2101/MAC30" -out serial://gps -n 1000 -p 48.418 -4.472 150.0 > $save_path/str2str.out &
    PIDs=$!
fi

sleep 1
# Roslaunch
roslaunch zodiac_launchers zodiac.launch &
PIDs+=" $!"

# Return to initial config
cd $pwd_path
unset pwd_path
unset use_rtk
unset save_path
unset IP_nb

fg
